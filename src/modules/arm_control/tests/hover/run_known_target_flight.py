#!/usr/bin/env python3
"""Bounded local-SITL known-target flight; arm execution requires explicit opt-in."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess as sp
import sys
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
import numpy as np
import yaml
import rosgraph
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from super_px4_bridge.msg import CoordinateTruth, KnownFlightSetpoint, KnownFlightStatus
from known_target import KnownTarget, rotation, quintic_segment, local_motion
from native_client import NativeClient

ROOT=Path(__file__).resolve().parents[5]
BUILD=ROOT/'build/px4_sitl_default'
WS=Path('/home/pcz/super_ws')
MODELS=ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
DEMO_OBSTACLES=(
    dict(name='demo_obstacle_wall',center=(1.5,0.,1.3),size=(.3,1.6,2.6),color=(.8,.15,.15,1.)),
    dict(name='demo_obstacle_upper',center=(3.4,2.6,1.3),size=(.7,.7,2.6),color=(.15,.35,.8,1.)),
    dict(name='demo_obstacle_lower',center=(3.4,-2.6,1.3),size=(.7,.7,2.6),color=(.9,.55,.1,1.)))
VEHICLE_OBSTACLE_RADIUS=.45
# Isolated demo-only simulated localization and native gains, not real VIO.
# No production airframe parameters are changed; fixed-demo native timing is
# documented in SO101_REVIEW.md. Sensor freshness and collision guards remain.
SIM_VISION_PARAMS=dict(EKF2_EV_CTRL=15,EKF2_HGT_REF=3,EKF2_GPS_CTRL=0,
    EKF2_BARO_CTRL=0,EKF2_RNG_CTRL=0,EKF2_EV_DELAY=0,
    EKF2_EVP_NOISE=.01,EKF2_EVV_NOISE=.01,EKF2_IMU_POS_Z=-.02,
    # Unused airspeed delay otherwise raises DELAY_MAX back to 100 ms.
    EKF2_ASP_DELAY=0,EKF2_DELAY_MAX=20,EKF2_TAU_VEL=.02,
    MPC_XY_P=1.5,MPC_XY_VEL_P_ACC=5.)
sys.path.insert(0,str(ROOT/'src/modules/arm_control/tests/grasp'))
from run_full_arm_grasp import remove_lidar,set_pad_patch


def values(p):
    return [p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]


def stop(proc):
    if proc is None or proc.poll() is not None: return
    for sig,seconds in ((signal.SIGINT,5),(signal.SIGTERM,3),(signal.SIGKILL,2)):
        try: os.killpg(proc.pid,sig)
        except ProcessLookupError: return
        try: proc.wait(timeout=seconds); return
        except sp.TimeoutExpired: pass


def save(path,value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def add_obstacle(world,description):
    model=ET.SubElement(world,'model',name=description['name'])
    ET.SubElement(model,'static').text='true'
    ET.SubElement(model,'pose').text=' '.join(map(str,[*description['center'],0,0,0]))
    link=ET.SubElement(model,'link',name='link')
    for kind in ('collision','visual'):
        item=ET.SubElement(link,kind,name=kind)
        ET.SubElement(ET.SubElement(ET.SubElement(item,'geometry'),'box'),'size').text=' '.join(map(str,description['size']))
        if kind=='visual':
            material=ET.SubElement(item,'material')
            for tag in ('ambient','diffuse'):
                ET.SubElement(material,tag).text=' '.join(map(str,description['color']))


def obstacle_clearance(position,description,radius=VEHICLE_OBSTACLE_RADIUS):
    point=np.asarray(position,dtype=float);center=np.asarray(description['center'],dtype=float)
    half=np.asarray(description['size'],dtype=float)*.5
    if point.shape!=(3,) or not np.isfinite([*point,*center,*half,radius]).all() or radius<=0:
        raise ValueError('Invalid obstacle-clearance input')
    return float(np.linalg.norm(np.maximum(np.abs(point-center)-half,0))-radius)


def line_deviation(point,start,goal):
    point=np.asarray(point,dtype=float);start=np.asarray(start,dtype=float);goal=np.asarray(goal,dtype=float)
    delta=goal-start;length=float(delta@delta)
    if any(v.shape!=(3,) for v in (point,start,goal)) or not np.isfinite([*point,*start,*goal]).all() or length<=0:
        raise ValueError('Invalid direct-line reference')
    u=float(np.clip((point-start)@delta/length,0,1))
    return float(np.linalg.norm(point-(start+u*delta)))


class Sources:
    def __init__(self):
        self.lock=threading.RLock()
        self.state=None; self.odom=None; self.truth=None; self.manager=''
        self.known_status=None
        self.truth_history=deque(maxlen=200)
        self._truth_cache=None
        self.subscribers=[
            rospy.Subscriber('/mavros/state',State,lambda m:self.update('state',m),queue_size=50),
            rospy.Subscriber('/mavros/local_position/odom',Odometry,lambda m:self.update('odom',m),queue_size=100,tcp_nodelay=True),
            rospy.Subscriber('/grasp_audit/truth',CoordinateTruth,lambda m:self.update('truth',m),queue_size=100,tcp_nodelay=True),
            rospy.Subscriber('/grasp_demo/flight_state',String,lambda m:self.update('manager',m.data),queue_size=20)]
        self.subscribers.append(rospy.Subscriber('/grasp_demo/known_status',KnownFlightStatus,
            lambda m:self.update('known_status',m),queue_size=20))
    def update(self,key,value):
        with self.lock:
            if key=='truth':
                if self.truth_history:
                    old=self.truth_history[-1]
                    identity=lambda t:(t.source_instance,t.world_epoch,t.vehicle_id,t.target_id)
                    if identity(old)!=identity(value) or value.header.stamp<old.header.stamp:
                        self.truth_history.clear()
                    elif value.header.stamp==old.header.stamp:return
                self.truth_history.append(value)
            setattr(self,key,value)
    def truth_samples(self):
        with self.lock:return list(self.truth_history)
    def truth_values(self,truth):
        # Convert each immutable ROS truth message once, not at every idle
        # poll in three observer threads. Freshness remains checked per read.
        with self.lock:
            cached=getattr(self,'_truth_cache',None)
            if cached is not None and cached[0] is truth:return cached[1]
            names=list(truth.link_names)
            if 'yhang550_base::base_link' not in names or 'so101::base_link' not in names:return None
            body=values(truth.link_poses[names.index('yhang550_base::base_link')])
            arm=values(truth.link_poses[names.index('so101::base_link')])
            v=truth.body_linear_velocity_world;w=truth.body_angular_velocity_world
            actual_v=np.array([v.x,v.y,v.z]);actual_w=np.array([w.x,w.y,w.z])
            expected=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
            if list(truth.joint_names)!=expected or len(truth.joint_position)!=6:
                raise RuntimeError('Unexpected measured arm joint layout')
            joints=np.asarray(truth.joint_position,dtype=float)
            if not np.isfinite([*body,*arm,*actual_v,*actual_w,*joints]).all():
                raise RuntimeError('Nonfinite pose/velocity')
            rotation(body[3:]);rotation(arm[3:])
            for array in (actual_v,actual_w,joints):array.setflags(write=False)
            data=dict(body=body,arm=arm,actual_v=actual_v,actual_w=actual_w,joints=joints)
            self._truth_cache=(truth,data)
            return data
    def read(self,*,require_odometry=True):
        with self.lock: state,odom,truth,manager,known=self.state,self.odom,self.truth,self.manager,self.known_status
        now=rospy.Time.now().to_sec()
        if not state or not state.connected or not truth or (require_odometry and not odom): return None
        if truth.header.frame_id!='gazebo_world' or (odom and
                (odom.header.frame_id!='map' or odom.child_frame_id!='base_link')):
            raise RuntimeError('Unexpected world/local/body frame')
        if not truth.body_velocity_valid or not truth.target_present: return None
        ages=[now-odom.header.stamp.to_sec() if odom else None,now-truth.header.stamp.to_sec()]
        # Physical grasp context uses this SAME-step truth, not MAVROS odometry.
        # Native PX4 still independently requires fresh/valid EKF flight state.
        # Flight navigation retains BOTH source checks; do not stall truth
        # publication on an unrelated ROS odometry delivery gap.
        if any(age is None or age<-.02 or age>.1 for age in (ages if require_odometry else ages[1:])): return None
        physical=self.truth_values(truth)
        if physical is None:return None
        local=values(odom.pose.pose) if require_odometry else None
        estimated_body_v=None
        if require_odometry:
            tv=odom.twist.twist.linear; estimated_body_v=[tv.x,tv.y,tv.z]
        local_values=[*local,*estimated_body_v] if require_odometry else []
        if not np.isfinite(local_values).all():
            raise RuntimeError('Nonfinite pose/velocity')
        if require_odometry:rotation(local[3:])
        return dict(now=now,state=state,local=local,estimated_body_v=estimated_body_v,
            manager=manager,truth=truth,ages=ages,known_status=known,**physical)


def make_world(folder,target,env,approach_guard=False,fixed_grasp=False,sim_vision=False,super_obstacle=False):
    expanded=sp.check_output(['gz','sdf','-p',str(MODELS/'yhang550/yhang550.sdf')],
        env=env,text=True,timeout=30)
    model=ET.fromstring(expanded).find('model')
    if not super_obstacle:remove_lidar(model)
    p=model.find('pose')
    if p is None: p=ET.SubElement(model,'pose')
    p.text='0 0 0 0 0 0'
    if model.find("joint[@name='bench_anchor']") is not None: raise RuntimeError('Unexpected fixed bench')
    if sim_vision:
        if model.find('link').get('name')!='yhang550_base::base_link':
            raise RuntimeError('Sim vision requires the known body-origin canonical link')
        if any(e.get('filename')=='libgazebo_vision_plugin.so' for e in model.findall('plugin')):
            raise RuntimeError('Duplicate simulated vision source')
        plugin=ET.SubElement(model,'plugin',name='demo_sim_vision',filename='libgazebo_vision_plugin.so')
        for key,value in dict(robotNamespace='',pubRate=50,randomWalk=0.,noiseDensity=.001,corellationTime=60.).items():
            ET.SubElement(plugin,key).text=str(value)
        bridge=model.find("plugin[@name='mavlink_interface']")
        for key,value in dict(visionSubTopic='/vision_odom',send_odometry='true').items():
            element=bridge.find(key)
            if element is None:element=ET.SubElement(bridge,key)
            element.text=value
    if fixed_grasp:
        from fixed_demo_plan import configure_arm_hold,GRIPPER_P,SUPPORT_HALF
        configure_arm_hold(model)
        bridge=model.find("plugin[@name='mavlink_interface']")
        ET.SubElement(bridge,'enableArmGraspFeedback').text='true'
        controller=model.find("plugin[@name='gripper_controller']")
        if controller is None or abs(float(controller.findtext('cmdMax'))-.01)>1e-8:
            raise RuntimeError('Unexpected production gripper torque cap')
        controller.find('p').text=str(GRIPPER_P)
        ET.SubElement(controller,'commandTimeoutPolicy').text='last_target'
        joint=next(j for j in model.findall('joint') if j.get('name').split('::')[-1]=='gripper')
        joint.find('axis/limit/lower').text='.04';joint.find('axis/limit/upper').text='.3'
        pads=[c for c in model.iter('collision') if c.get('name','').split('::')[-1] in
            ('gripper_fixed_finger_collision','moving_jaw_collision')]
        if len(pads)!=2:raise RuntimeError('Cannot identify both production pads')
        for pad in pads:set_pad_patch(pad,4)
    tree=ET.parse(ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world')
    tree.getroot().set('version','1.7'); world=tree.find('world')
    world.find('physics/ode/solver/iters').text='40'
    world.append(model)
    R=target.object[:3,:3]
    rpy=[math.atan2(R[2,1],R[2,2]),math.asin(np.clip(-R[2,0],-1,1)),math.atan2(R[1,0],R[0,0])]
    cube=ET.parse(MODELS/'test_cube/model.sdf').find('model')
    if fixed_grasp:
        # This calibrated fixture is tilted. Change only its experiment copy,
        # not the production object, mass, or inertia; no artificial attachment.
        for coefficient in cube.findall('.//collision/surface/friction/ode/*'):
            if coefficient.tag in ('mu','mu2'):coefficient.text='1.2'
    p=cube.find('pose')
    if p is None: p=ET.SubElement(cube,'pose')
    p.text=' '.join(map(str,[*target.object[:3,3],*rpy])); world.append(cube)
    support=ET.SubElement(world,'model',name='cube_support'); ET.SubElement(support,'static').text='true'
    down=1 if R[2,2]<0 else -1
    center=target.object[:3,3]+down*R[:,2]*.012
    if fixed_grasp:
        from fixed_demo_plan import SUPPORT_OFFSET_X
        center=center+R[:,0]*SUPPORT_OFFSET_X
    ET.SubElement(support,'pose').text=' '.join(map(str,[*center,*rpy]))
    link=ET.SubElement(support,'link',name='link')
    for kind in ('collision','visual'):
        e=ET.SubElement(link,kind,name=kind)
        ET.SubElement(ET.SubElement(ET.SubElement(e,'geometry'),'box'),'size').text=(
            ' '.join(map(str,SUPPORT_HALF*2)) if fixed_grasp else '.012 .014 .004')
        if fixed_grasp and kind=='collision':
            ode=ET.SubElement(ET.SubElement(ET.SubElement(e,'surface'),'friction'),'ode')
            for coefficient in ('mu','mu2'):ET.SubElement(ode,coefficient).text='1.2'
    if super_obstacle:
        for description in DEMO_OBSTACLES:add_obstacle(world,description)
        gui=world.find('gui')
        if gui is None:gui=ET.SubElement(world,'gui',fullscreen='false')
        camera=ET.SubElement(gui,'camera',name='demo_camera')
        ET.SubElement(camera,'pose').text='3 -8 5 0 0.4 1.57'
        ET.SubElement(camera,'view_controller').text='orbit'
    plugin=ET.SubElement(world,'plugin',name='coordinate_truth',filename='libso101_coordinate_truth.so')
    ET.SubElement(plugin,'vehicleName').text='yhang550'; ET.SubElement(plugin,'targetName').text=target.name
    if approach_guard: ET.SubElement(plugin,'approachGuard').text='true'
    if fixed_grasp: ET.SubElement(plugin,'fixedGraspGuard').text='true'
    tree.write(folder/'scene.world',encoding='unicode',xml_declaration=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('name'); p.add_argument('--execute-flight',action='store_true')
    p.add_argument('--observe-hover-permit',action='store_true',
        help='Test PREPARE/refusal/revocation only; never move the arm or close the jaw')
    p.add_argument('--execute-approach',action='store_true',help='Bounded native arm approach; jaw target never changes')
    p.add_argument('--execute-fixed-grasp',action='store_true',help='Explicit fixed-point controlled contact, close and 40mm tool lift')
    p.add_argument('--target',type=Path)
    p.add_argument('--sim-vision',action='store_true',help='Demo-only low-noise Gazebo odometry fused by EKF; original dynamics/guards retained')
    p.add_argument('--super-obstacle',action='store_true',
        help='Use Velodyne + existing SUPER planner through the bounded multi-obstacle course before fine grasp')
    p.add_argument('--gui',action='store_true',help='Show this owned Gazebo world in gzclient')
    p.add_argument('--hold-seconds',type=float,default=10.,
        help='Flight-only observation window, 10..60 sim seconds; does not change arrival or grasp guards')
    args=p.parse_args()
    speed_factor=float(os.environ.get('PX4_SIM_SPEED_FACTOR','1'))
    if not math.isfinite(speed_factor) or not .25<=speed_factor<=1:
        p.error('This bounded demo supports PX4_SIM_SPEED_FACTOR in .25..1')
    if not math.isfinite(args.hold_seconds) or not 10<=args.hold_seconds<=60:
        p.error('--hold-seconds must be finite and within 10..60')
    if args.hold_seconds!=10 and (args.execute_approach or args.observe_hover_permit or args.execute_fixed_grasp):
        p.error('--hold-seconds override is for flight-only tests, never arm/grasp execution')
    if sum([args.execute_approach,args.observe_hover_permit,args.execute_fixed_grasp])>1:
        p.error('Choose permission, no-close approach OR explicit fixed grasp')
    if args.super_obstacle and not (args.execute_flight and args.execute_fixed_grasp and args.sim_vision):
        p.error('--super-obstacle currently requires --execute-flight --execute-fixed-grasp --sim-vision')
    args.target=args.target or Path(__file__).with_name(
        'known_target_obstacle.json' if args.super_obstacle else
        ('known_target_fixed.json' if args.execute_fixed_grasp else 'known_target.json'))
    template=None
    if args.execute_fixed_grasp:
        from fixed_demo_plan import FixedDemoPlan
        template=FixedDemoPlan(args.target);precheck=template.preview();target=template.target
        if not precheck['sampled_geometry_feasible']:p.error('Fixed demo numerical geometry rejected')
    else:target=KnownTarget(json.loads(args.target.read_text()))
    if not args.execute_flight:
        if args.observe_hover_permit or args.execute_approach or args.execute_fixed_grasp: p.error('Execution requires --execute-flight')
        preview=target.preview();preview['sim_vision_requested']=args.sim_vision
        print(json.dumps(preview,indent=2)); return 0
    if not args.name.replace('_','').isalnum(): p.error('Use a unique safe case name')
    if target.name!='test_cube': p.error('This first fixture supports test_cube only')
    if any(sp.run(['pgrep','-x',n],stdout=sp.DEVNULL).returncode==0
           for n in ('px4','gzserver','gzclient','rosmaster','mavros_node')):
        p.error('Existing simulation detected; refusing to disturb it')
    for port in (11311,11345,4560):
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1',port))==0: p.error('Occupied simulation port '+str(port))
    folder=ROOT/'build/known_target_flight'/args.name
    folder.mkdir(parents=True,exist_ok=False); (folder/'rootfs').mkdir()
    env=os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:11311',ROS_HOSTNAME='localhost',
        GAZEBO_MASTER_URI='http://127.0.0.1:11345',GAZEBO_MODEL_PATH=str(MODELS),
        GAZEBO_PLUGIN_PATH=str(WS/'devel/lib')+':'+str(BUILD/'build_gazebo-classic')+
            ':/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
        LD_LIBRARY_PATH=str(WS/'devel/lib')+':'+str(BUILD/'build_gazebo-classic')+
            ':/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
        GAZEBO_MODEL_DATABASE_URI='',PX4_SIM_MODEL='gazebo-classic_yhang550',PX4_SYS_AUTOSTART='1020')
    if args.sim_vision:
        env.update({'PX4_PARAM_'+key:str(value) for key,value in SIM_VISION_PARAMS.items()})
    env.pop('ROS_IP',None); make_world(folder,target,env,args.execute_approach or args.execute_fixed_grasp,
        args.execute_fixed_grasp,args.sim_vision,args.super_obstacle)
    # Leave the object out of the body planner's final safety envelope.  The
    # existing bounded handover covers the last 0.6 m to the grasp hover.
    planner_goal_world=(target.hover[:3,3]-.6*target.hover[:3,0]) if args.super_obstacle else target.hover[:3,3]
    planner_config=WS/'src/super_px4_bridge/config/px4_sim.yaml'
    if args.super_obstacle:
        config=yaml.safe_load(planner_config.read_text())
        # Carry the folded arm through the obstacle turns at a conservative
        # pace.  Only this generated demo copy changes; no flight PID changes.
        config['traj_opt']['boundary']['max_vel']=.5
        config['traj_opt']['boundary']['max_acc']=.6
        config['traj_opt']['exp_traj']['penna_t']=1.e5
        config['traj_opt']['switch']['print_optimizer_log']=True
        planner_config=folder/'super_demo.yaml'
        planner_config.write_text(yaml.safe_dump(config,sort_keys=False))
    mavros=Path(sp.check_output(['rospack','find','mavros'],env=env,text=True).strip())/'launch'
    mavros_config=yaml.safe_load((mavros/'px4_config.yaml').read_text())
    # PX4 lockstep and Gazebo /clock already share simulation time. Preserve
    # FCU source stamps; NONE would instead stamp at ROS receipt time.
    mavros_config['time']['timesync_mode']='PASSTHROUGH'
    mavros_config['conn']['timesync_rate']=0.
    save(folder/'mavros_config.json',mavros_config) # JSON is valid rosparam YAML.
    mavros_launch=ET.parse(mavros/'px4.launch')
    mavros_launch.find("include/arg[@name='config_yaml']").set('value',str(folder/'mavros_config.json'))
    mavros_launch.write(folder/'mavros.launch',encoding='unicode',xml_declaration=True)
    if template:save(folder/'fixed_demo_plan.json',precheck)
    # Parent ROS probe and owned children must use the same local master.
    os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'],ROS_HOSTNAME='localhost')
    os.environ.pop('ROS_IP',None)
    files=[Path(__file__),Path(__file__).with_name('known_target.py'),args.target,folder/'scene.world',
        folder/'mavros_config.json',folder/'mavros.launch',mavros/'node.launch',mavros/'px4_pluginlists.yaml',
        WS/'src/super_px4_bridge/src/super_flight_manager.cpp',WS/'devel/lib/libso101_coordinate_truth.so']
    if args.sim_vision:
        files += [Path(__file__).with_name('native_client.py'),BUILD/'build_gazebo-classic/libgazebo_vision_plugin.so',
            BUILD/'build_gazebo-classic/libgazebo_mavlink_interface.so']
    if args.observe_hover_permit: files.append(Path(__file__).with_name('hover_permit_probe.py'))
    if args.execute_approach: files.append(Path(__file__).with_name('hover_approach_probe.py'))
    if args.execute_fixed_grasp:
        files += [Path(__file__).with_name('fixed_demo_plan.py'),Path(__file__).with_name('fixed_grasp_probe.py'),
            Path(__file__).with_name('native_client.py'),
            ROOT/'src/modules/arm_control/grasp/FixedGrasp.hpp',ROOT/'src/modules/arm_control/grasp/FixedGraspAdapter.cpp']
    if args.super_obstacle:
        files += [WS/'src/super_px4_bridge/config/px4_sim.yaml',
            WS/'src/super_px4_bridge/launch/super_navigation.launch',
            planner_config,
            WS/'src/super_px4_bridge/src/super_sensor_adapter.cpp',
            WS/'src/super_px4_bridge/src/super_mavros_bridge.cpp']
    if args.execute_approach:
        files += [ROOT/'src/modules/arm_control/grasp/HoverApproach.hpp',
            ROOT/'src/modules/arm_control/grasp/HoverApproachAdapter.cpp',
            WS/'src/super_px4_bridge/msg/KnownFlightStatus.msg',
            WS/'src/super_px4_bridge/msg/KnownFlightSetpoint.msg',
            WS/'src/super_px4_bridge/include/super_px4_bridge/known_flight_handover.h',
            WS/'src/super_px4_bridge/config/grasp_frame_audit.yaml']
    if args.observe_hover_permit or args.execute_approach or args.execute_fixed_grasp:
        files += [BUILD/'bin/px4', ROOT/'src/modules/arm_control/grasp/HoverPermit.hpp',
            ROOT/'src/modules/arm_control/grasp/HoverPermitAdapter.cpp',
            WS/'src/super_px4_bridge/src/coordinate_truth_plugin.cpp',
            WS/'src/super_px4_bridge/msg/CoordinateTruth.msg']
    parameter_overrides={name[len('PX4_PARAM_'):]:value for name,value in env.items()
        if name.startswith('PX4_PARAM_')}
    save(folder/'manifest.json',dict(stage=5 if args.execute_fixed_grasp else (4 if args.execute_approach else (3 if args.observe_hover_permit else 2)),
        flight_hold_seconds=args.hold_seconds,
        sim_speed_factor=speed_factor,
        mavros_timesync_mode='PASSTHROUGH',
        no_close_approach=args.execute_approach,
        hover_prepare_only=args.observe_hover_permit,lidar=args.super_obstacle,vehicle_fixed=False,object_attached=False,
        arm_motion_requests_permitted=args.execute_approach or args.execute_fixed_grasp,
        fixed_grasp_permitted=args.execute_fixed_grasp,gripper_requests_sent=False,grasp_tested=False,
        localization_source=('SIM_VISION_EKF_FIXED_LOCAL_TARGET' if args.sim_vision else
            'LIVE_GAZEBO_TRUTH_CONTROL_TRANSFORM_AND_VELOCITY_OFFSET_COMPENSATION'),
        navigation_source=('VELODYNE_REGISTERED_WITH_MAVROS_ODOMETRY_TO_SUPER' if args.super_obstacle else
            'BOUNDED_KNOWN_QUINTIC'),
        planner_goal_world_m=planner_goal_world.tolist() if args.super_obstacle else None,
        obstacle_course=[dict(name=o['name'],center=o['center'],size=o['size']) for o in DEMO_OBSTACLES]
            if args.super_obstacle else [],
        sim_vision=args.sim_vision,ekf_truth_injected=args.sim_vision,
        px4_parameter_overrides=parameter_overrides,
        flight_pid_modified=any(name.startswith(('MPC_','MC_')) for name in parameter_overrides),
        planned=target.preview(),
        precision_hover=dict(additional_flight_controller=False,
            strategy='BASE_HOLD_AFTER_UNFOLD_THEN_WORLD_TCP_IK'),
        hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}))
    procs=[]; logs=[]; subscribers=None; publisher=None; planner_goal_pub=None; hover_probe=None
    def start(name,cmd,cwd=None):
        log=(folder/(name+'.log')).open('w'); logs.append(log)
        proc=sp.Popen(cmd,env=env,cwd=cwd,stdout=log,stderr=sp.STDOUT,start_new_session=True)
        procs.append(proc); return proc
    started=time.monotonic(); result=dict(success=False,grasp_tested=False)
    armed_seen=False; last_sim=None; progress=time.monotonic(); rows=[]
    def safety():
        if hover_probe and hover_probe.error: raise RuntimeError(hover_probe.error)
        if any(proc.poll() is not None for proc in procs): raise RuntimeError('Own simulation process exited')
        if time.monotonic()-started>(440 if (args.execute_approach or args.execute_fixed_grasp) else 260)/speed_factor: raise RuntimeError('Wall watchdog exceeded')
        if time.monotonic()-progress>20 and last_sim is not None: raise RuntimeError('Sim clock stalled')
    try:
        core=start('roscore',['roscore'])
        deadline=time.monotonic()+20
        while not rosgraph.is_master_online():
            if core.poll() is not None or time.monotonic()>deadline: raise RuntimeError('ROS master unavailable')
            time.sleep(.1)
        sp.run(['rosparam','set','/use_sim_time','true'],env=env,check=True,timeout=10)
        rospy.init_node('known_target_flight_probe',anonymous=True,disable_signals=True)
        subscribers=Sources()
        start('gazebo',['gzserver','--verbose','--seed','123','-s','libgazebo_ros_api_plugin.so',str(folder/'scene.world')])
        if args.gui:start('gazebo_gui',['gzclient'])
        px4_proc=start('px4',[str(BUILD/'bin/px4'),'-d',str(BUILD/'etc')],folder/'rootfs')
        start('mavros',['roslaunch',str(folder/'mavros.launch'),'fcu_url:=udp://:14540@127.0.0.1:14580'])
        while True:
            safety(); row=subscribers.read()
            if row and row['now']>=20: break
            if time.monotonic()-started>100: raise RuntimeError('Sources not ready')
            time.sleep(.1)
        identity=(row['truth'].source_instance,row['truth'].world_epoch,row['truth'].vehicle_id,row['truth'].target_id)
        if (rospy.get_param('/mavros/time/timesync_mode')!='PASSTHROUGH' or
                rospy.get_param('/mavros/conn/timesync_rate')!=0):
            raise RuntimeError('Shared simulation clock configuration mismatch; no takeoff')
        object_pose=row['truth'].target_pose
        save(folder/'preflight_state.json',dict(sim_s=row['now'],body_world=row['body'],
            arm_world=row['arm'],object_world=values(object_pose),
            fixed_grasp_geometry_valid=row['truth'].fixed_grasp_geometry_valid,
            fixed_grasp_clearance_m=row['truth'].fixed_grasp_clearance,
            fixed_grasp_forbidden_contact=row['truth'].fixed_grasp_forbidden_contact))
        if row['truth'].target_name!=target.name or not row['truth'].target_id or np.linalg.norm(
            np.asarray(values(object_pose)[:3])-target.object[:3,3])>.004:
            raise RuntimeError('Actual fixed object position does not match the known target input')
        if args.execute_fixed_grasp:
            actual=rotation(values(object_pose)[3:])
            angle=math.acos(float(np.clip((np.trace(target.object[:3,:3].T@actual)-1)*.5,-1,1)))
            if angle>math.radians(2):raise RuntimeError('Fixed cube did not settle in its calibrated support pose; no takeoff')
        if args.sim_vision:
            native=NativeClient(BUILD,env,transport='unix',expected_pid=px4_proc.pid)
            odometry=native.status('vehicle_visual_odometry');fusion=native.status('estimator_status_flags')
            save(folder/'sim_vision_preflight.json',dict(odometry=odometry,fusion=fusion))
            age=rospy.Time.now().to_sec()-odometry.get('timestamp_sample',0)*1e-6
            if (not -.02<=age<=.15 or odometry.get('pose_frame')!=1 or odometry.get('velocity_frame')!=3 or
                not all(len(odometry.get(key,[]))==size and np.isfinite(odometry[key]).all()
                    for key,size in (('position',3),('velocity',3),('q',4))) or
                not all(fusion.get(key) for key in ('cs_ev_pos','cs_ev_hgt','cs_ev_vel','cs_ev_yaw')) or
                fusion.get('cs_gnss_pos') or fusion.get('cs_gnss_vel') or fusion.get('cs_baro_hgt')):
                raise RuntimeError('Sim vision odometry/fusion preflight failed; no takeoff')
            # The generated model starts at world zero. The vision plugin and
            # MAVROS both use the same gravity-aligned ENU axes, not body tilt.
            origin_residual=float(np.linalg.norm(np.asarray(row['local'][:3])-row['body'][:3]))
            save(folder/'sim_vision_preflight.json',dict(odometry=odometry,fusion=fusion,
                frame_alignment='GENERATED_WORLD_ORIGIN_AND_COMMON_ENU_AXES',origin_residual_m=origin_residual))
            if origin_residual>.02:raise RuntimeError('Sim vision world/local origin mismatch; no takeoff')
            print('Sim vision fusion verified; isolated demo parameters recorded in manifest.',flush=True)
        takeoff_local=row['local'][2]+target.hover[2,3]-row['body'][2]
        if not .3<=takeoff_local<=2.5: raise RuntimeError('Takeoff coordinate invalid')
        horizontal_limit=6.0 if args.super_obstacle else 1.0
        if np.linalg.norm(target.hover[:2,3]-np.array(row['body'][:2]))>horizontal_limit or not 1.0<=target.hover[2,3]<=2.0:
            raise RuntimeError('Flight fixture exceeds its bounded horizontal/height envelope')
        if args.observe_hover_permit:
            from hover_permit_probe import HoverPermitProbe
            hover_probe=HoverPermitProbe(BUILD,env,subscribers,target,folder); hover_probe.start()
            deadline=time.monotonic()+10
            while hover_probe.stage in ('unarmed','unarmed_ack'):
                safety()
                if time.monotonic()>deadline: raise RuntimeError('Unarmed refusal test timeout')
                time.sleep(.01)
        if args.execute_approach:
            if not row['truth'].approach_geometry_valid:
                raise RuntimeError('Known fixture geometry guard unavailable; no takeoff')
            from hover_approach_probe import HoverApproachProbe
            hover_probe=HoverApproachProbe(BUILD,env,subscribers,target,folder); hover_probe.start()
        if args.execute_fixed_grasp:
            if not row['truth'].fixed_grasp_geometry_valid or row['truth'].fixed_grasp_forbidden_contact:
                raise RuntimeError('Fine grasp geometry guard unavailable; no takeoff')
            from fixed_grasp_probe import FixedGraspProbe
            hover_probe=FixedGraspProbe(BUILD,env,subscribers,template,folder,expected_pid=px4_proc.pid);hover_probe.start()
        if args.super_obstacle:
            start('super_navigation',['roslaunch','super_px4_bridge','super_navigation.launch',
                'start_flight_manager:=false','planner_config:='+str(planner_config)])
        manager=['rosrun','super_px4_bridge','super_flight_manager_node','_auto_arm:=true',
            '_takeoff_height:='+str(takeoff_local),'_hover_stable_time:=2.0','__name:=known_flight_manager_probe']
        manager.append('_allow_terminal_known_handover:=true' if args.super_obstacle else '_known_goal_mode:=true')
        if args.super_obstacle:manager += ['_super_cmd_timeout:=1.0','_map_warmup_time:=3.0']
        start('flight_manager',manager)
        publisher=rospy.Publisher('/grasp_demo/known_setpoint',KnownFlightSetpoint,queue_size=1)
        if args.super_obstacle:planner_goal_pub=rospy.Publisher('/planning/user_goal',PoseStamped,queue_size=1)
        route_start=None; route_point=None; stable_since=None; hold_since=None; last_publish=-math.inf
        planner_goal_sent=False;planner_goal_time=None;planner_track_seen=False;planner_start=None
        min_obstacle_clearance=math.inf;max_direct_deviation=0.
        last_command=None # Audit only; never used as feedback or to renew a lease.
        with (folder/'telemetry.jsonl').open('x') as out:
            while True:
                safety(); row=subscribers.read()
                if row is None:
                    time.sleep(.005); continue
                now=row['now']
                if last_sim!=now: last_sim=now; progress=time.monotonic()
                current=(row['truth'].source_instance,row['truth'].world_epoch,row['truth'].vehicle_id,row['truth'].target_id)
                if current!=identity: raise RuntimeError('World/entity identity changed')
                state=row['state']; armed_seen=armed_seen or state.armed
                if armed_seen and (not state.armed or state.mode!='OFFBOARD'):
                    raise RuntimeError('Arming/mode lost; will not override it')
                body=np.array(row['body'][:3]); R=rotation(row['body'][3:])
                tilt=max(abs(math.atan2(R[2,1],R[2,2])),abs(math.asin(np.clip(-R[2,0],-1,1))))
                if armed_seen and tilt>math.radians(15): raise RuntimeError('Tilt guard exceeded')
                fold_error=float(np.max(np.abs(row['joints']-[-.07,-1.175,1.45,1.3,-1.57,.3])))
                if fold_error>.08 and not ((args.execute_approach or args.execute_fixed_grasp) and hover_probe.motion_started):
                    raise RuntimeError('Folded arm departed initial target before approach')
                if now>(300 if (args.execute_approach or args.execute_fixed_grasp) else 120): raise RuntimeError('Simulation-time limit exceeded')
                if args.super_obstacle and row['manager']=='TRACK_SUPER':planner_track_seen=True
                if args.super_obstacle and not planner_goal_sent and row['manager']=='HOVER_READY' and planner_goal_pub.get_num_connections():
                    goal=PoseStamped();goal.header.stamp=rospy.Time.now();goal.header.frame_id='world'
                    goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=map(float,planner_goal_world)
                    goal.pose.orientation.z=math.sin(target.yaw*.5);goal.pose.orientation.w=math.cos(target.yaw*.5)
                    planner_goal_pub.publish(goal);planner_goal_sent=True;planner_goal_time=now;planner_start=body.copy()
                    print('HOVER_READY: multi-obstacle SUPER goal published; arm remains folded.',flush=True)
                if args.super_obstacle and planner_goal_sent and not planner_track_seen and now-planner_goal_time>8:
                    raise RuntimeError('SUPER did not produce an accepted trajectory; arm remains folded')
                if args.super_obstacle and planner_goal_sent:
                    current_clearance=min(obstacle_clearance(body,o) for o in DEMO_OBSTACLES)
                    min_obstacle_clearance=min(min_obstacle_clearance,current_clearance)
                    max_direct_deviation=max(max_direct_deviation,line_deviation(body,planner_start,planner_goal_world))
                    if current_clearance<0:raise RuntimeError('Vehicle obstacle envelope intersected the demo course')
                if args.super_obstacle and route_start is None and row['manager']=='FINAL_HOVER':
                    route_start=now;route_point=body.copy()
                    print('SUPER terminal hover accepted; bounded precision handover begins.',flush=True)
                if not args.super_obstacle and route_start is None and row['manager']=='HOVER_READY':
                    route_start=now; route_point=body.copy()
                    print('HOVER_READY: known world route begins; arm remains folded.',flush=True)
                if route_start is not None and now-last_publish>=.02:
                    wp,wv=quintic_segment(route_point,target.hover[:3,3],now-route_start,6)
                    if args.sim_vision:
                        lp,lv,yaw=wp,wv,target.yaw
                    else:
                        lp,lv,yaw=local_motion(wp,wv,target.yaw,row['body'],row['local'],
                            row['actual_v'],row['estimated_body_v'])
                    cmd=KnownFlightSetpoint(); cmd.header.stamp=rospy.Time.now(); cmd.header.frame_id='map'; cmd.task_id=1
                    if args.execute_approach or args.execute_fixed_grasp:
                        cmd.task_id,cmd.plan_id,cmd.target_id,cmd.scene_id=hover_probe.plan.ids
                        cmd.terminal_hold=now-route_start>=6
                    cmd.setpoint.header=cmd.header; cmd.setpoint.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
                    cmd.setpoint.type_mask=PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE
                    cmd.setpoint.position.x,cmd.setpoint.position.y,cmd.setpoint.position.z=map(float,lp)
                    cmd.setpoint.velocity.x,cmd.setpoint.velocity.y,cmd.setpoint.velocity.z=map(float,lv)
                    cmd.setpoint.yaw=yaw; publisher.publish(cmd); last_publish=now
                    last_command=dict(sim_s=now,odom_stamp_s=row['now']-row['ages'][0],
                        truth_stamp_s=row['truth'].header.stamp.to_sec(),
                        position_local=lp.tolist(),velocity_local=lv.tolist(),
                        world_velocity=wv.tolist())
                error=float(np.linalg.norm(body-target.hover[:3,3])); speed=float(np.linalg.norm(row['actual_v']))
                record=dict(sim_s=now,armed=state.armed,mode=state.mode,manager=row['manager'],
                    body_world=row['body'],body_speed=speed,hover_error_m=error,tilt_rad=tilt,
                    body_linear_velocity_world=row['actual_v'].tolist(),
                    body_angular_velocity_world=row['actual_w'].tolist(),
                    body_local=row['local'],estimated_velocity_body=row['estimated_body_v'],
                    flight_command=last_command,
                    source_ages_s=row['ages'],fold_error_rad=fold_error,
                    truth_stamp_s=row['truth'].header.stamp.to_sec(),
                    source_instance=row['truth'].source_instance,world_epoch=row['truth'].world_epoch,
                    obstacle_clearance_m=(min(obstacle_clearance(body,o) for o in DEMO_OBSTACLES)
                        if args.super_obstacle and planner_goal_sent else None),
                    direct_line_deviation_m=(line_deviation(body,planner_start,planner_goal_world)
                        if args.super_obstacle and planner_goal_sent else None),
                    close_permitted=bool(args.execute_fixed_grasp and hover_probe and hover_probe.native.get('close_authorized')))
                if not rows or now-rows[-1]['sim_s']>=.05:
                    rows.append(record); out.write(json.dumps(record,allow_nan=False)+'\n'); out.flush()
                good=(route_start is not None and now-route_start>=6 and row['manager']=='TRACK_KNOWN'
                    and error<=.03 and speed<=.05 and tilt<=math.radians(5))
                if not good: stable_since=None
                elif stable_since is None: stable_since=now
                elif now-stable_since>=2 and hold_since is None:
                    hold_since=now
                    print('Physical world hover reached; guarded grasp coordinator continues.' if args.execute_fixed_grasp else
                        f'Physical world goal reached; observing {args.hold_seconds:g}-second no-close hold.',flush=True)
                if (hold_since is not None and now-hold_since>=args.hold_seconds and
                        (hover_probe is None or hover_probe.complete)):
                    held=[r for r in rows if r['sim_s']>=hold_since]
                    navigation_pass=bool(not args.super_obstacle or (planner_track_seen and
                        min_obstacle_clearance>=.02 and max_direct_deviation>=.5))
                    hold_pass=bool(navigation_pass and all(r['hover_error_m']<=.05 and
                        r['tilt_rad']<=math.radians(5) for r in held))
                    result.update(success=hold_pass,flight_goal_validated=hold_pass,
                        coarse_arrival_observed=True,hover_grasp_permission_validated=False,
                        super_obstacle_navigation_validated=navigation_pass if args.super_obstacle else False,
                        super_track_observed=planner_track_seen,
                        minimum_vehicle_obstacle_clearance_m=(min_obstacle_clearance if args.super_obstacle else None),
                        maximum_direct_line_deviation_m=(max_direct_deviation if args.super_obstacle else None),
                        hold_max_fold_error_rad=max(r['fold_error_rad'] for r in held),
                        actual_object_position_checked=True,hold_sim_seconds=now-hold_since,
                        hold_required_s=args.hold_seconds,
                        hold_max_error_m=max(r['hover_error_m'] for r in held),
                        hold_mean_error_m=float(np.mean([r['hover_error_m'] for r in held])),
                        goal_world_m=target.hover[:3,3].tolist(),final_body_world=row['body'])
                    break
                if row['manager']=='FAILSAFE_HOVER': raise RuntimeError('Flight input lease failed; no automatic resumption')
                time.sleep(.004)
        if hover_probe:
            key='fixed_grasp' if args.execute_fixed_grasp else ('hover_approach' if args.execute_approach else 'hover_prepare')
            result[key]=hover_probe.finish(); hover_probe=None
            if not result[key].get('grasp_validated' if args.execute_fixed_grasp else ('approach_validated' if args.execute_approach else 'hover_prepare_validated')):
                raise RuntimeError('Hover PREPARE tests incomplete')
        graph=rosgraph.Master('/known_target_graph').getSystemState()
        save(folder/'ros_graph.json',dict(publishers=graph[0],subscribers=graph[1],services=graph[2]))
        pubs=[nodes for topic,nodes in graph[0] if topic=='/mavros/setpoint_raw/local']
        if len(pubs)!=1 or len(pubs[0])!=1: raise RuntimeError('Multiple flight command publishers')
        result['sole_flight_publisher']=pubs[0][0]
    except (Exception,KeyboardInterrupt) as error:
        result.update(success=False,error=str(error) or type(error).__name__,final_sim_s=last_sim)
    finally:
        if hover_probe:
            try: result['fixed_grasp' if args.execute_fixed_grasp else ('hover_approach' if args.execute_approach else 'hover_prepare')]=hover_probe.finish()
            except Exception as error:
                result.update(success=False,hover_probe_error=str(error))
                result['hover_prepare_events']=hover_probe.events
        if publisher: publisher.unregister()
        if subscribers:
            for sub in subscribers.subscribers: sub.unregister()
        for proc in reversed(procs): stop(proc)
        for log in logs: log.close()
        request_count=(hover_probe.approach_request_count if (args.execute_approach or args.execute_fixed_grasp) and hover_probe else
            result.get('fixed_grasp',{}).get('grasp_requests',0) if args.execute_fixed_grasp else
            result.get('hover_approach',{}).get('approach_request_count',0))
        result.update(armed_seen=armed_seen,arm_motion_requests_sent=bool(request_count),
            arm_approach_request_count=request_count,
            gripper_requests_sent=bool(args.execute_fixed_grasp and (
                hover_probe.close_requested if hover_probe else result.get('fixed_grasp',{}).get('close_requested',False))),
            grasp_tested=bool(args.execute_fixed_grasp and (
                hover_probe.close_requested if hover_probe else result.get('fixed_grasp',{}).get('close_requested',False))),
            own_processes_stopped=True,wall_elapsed_s=time.monotonic()-started)
        save(folder/'result.json',result); print(json.dumps(result,indent=2,allow_nan=False),flush=True)
    return 0 if result['success'] else 2


if __name__=='__main__': raise SystemExit(main())
