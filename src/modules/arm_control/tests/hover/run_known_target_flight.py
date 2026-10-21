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
import rosgraph
import rospy
from geometry_msgs.msg import Pose
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from super_px4_bridge.msg import CoordinateTruth, KnownFlightSetpoint, KnownFlightStatus
from known_target import KnownTarget, rotation, quintic_segment, local_motion

ROOT=Path(__file__).resolve().parents[5]
BUILD=ROOT/'build/px4_sitl_default'
WS=Path('/home/pcz/super_ws')
MODELS=ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
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


class Sources:
    def __init__(self):
        self.lock=threading.RLock()
        self.state=None; self.odom=None; self.truth=None; self.manager=''
        self.known_status=None
        self.truth_history=deque(maxlen=200)
        self.subscribers=[
            rospy.Subscriber('/mavros/state',State,lambda m:self.update('state',m),queue_size=50),
            rospy.Subscriber('/mavros/local_position/odom',Odometry,lambda m:self.update('odom',m),queue_size=100),
            rospy.Subscriber('/grasp_audit/truth',CoordinateTruth,lambda m:self.update('truth',m),queue_size=100),
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
    def read(self):
        with self.lock: state,odom,truth,manager,known=self.state,self.odom,self.truth,self.manager,self.known_status
        now=rospy.Time.now().to_sec()
        if not state or not state.connected or not odom or not truth: return None
        if truth.header.frame_id!='gazebo_world' or odom.header.frame_id!='map' or odom.child_frame_id!='base_link':
            raise RuntimeError('Unexpected world/local/body frame')
        if not truth.body_velocity_valid or not truth.target_present: return None
        ages=[now-odom.header.stamp.to_sec(),now-truth.header.stamp.to_sec()]
        if any(age<-.02 or age>.1 for age in ages): return None
        names=list(truth.link_names)
        if 'yhang550_base::base_link' not in names or 'so101::base_link' not in names: return None
        body=values(truth.link_poses[names.index('yhang550_base::base_link')])
        arm=values(truth.link_poses[names.index('so101::base_link')])
        local=values(odom.pose.pose)
        v=truth.body_linear_velocity_world; w=truth.body_angular_velocity_world
        actual_v=np.array([v.x,v.y,v.z]); actual_w=np.array([w.x,w.y,w.z])
        expected=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
        if list(truth.joint_names)!=expected or len(truth.joint_position)!=6:
            raise RuntimeError('Unexpected measured arm joint layout')
        joint_values=np.asarray(truth.joint_position,dtype=float)
        tv=odom.twist.twist.linear; estimated_body_v=[tv.x,tv.y,tv.z]
        if not np.isfinite([*body,*arm,*local,*actual_v,*actual_w,*estimated_body_v,*joint_values]).all():
            raise RuntimeError('Nonfinite pose/velocity')
        rotation(body[3:]); rotation(local[3:]); rotation(arm[3:])
        return dict(now=now,state=state,body=body,arm=arm,local=local,actual_v=actual_v,
            actual_w=actual_w,estimated_body_v=estimated_body_v,manager=manager,
            truth=truth,ages=ages,joints=joint_values,known_status=known)


def make_world(folder,target,env,approach_guard=False,fixed_grasp=False):
    expanded=sp.check_output(['gz','sdf','-p',str(MODELS/'yhang550/yhang550.sdf')],
        env=env,text=True,timeout=30)
    model=ET.fromstring(expanded).find('model'); remove_lidar(model)
    p=model.find('pose')
    if p is None: p=ET.SubElement(model,'pose')
    p.text='0 0 0 0 0 0'
    if model.find("joint[@name='bench_anchor']") is not None: raise RuntimeError('Unexpected fixed bench')
    if fixed_grasp:
        from fixed_demo_plan import configure_arm_hold
        configure_arm_hold(model)
        bridge=model.find("plugin[@name='mavlink_interface']")
        ET.SubElement(bridge,'enableArmGraspFeedback').text='true'
        controller=model.find("plugin[@name='gripper_controller']")
        controller.find('p').text='.25'
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
        ET.SubElement(ET.SubElement(ET.SubElement(e,'geometry'),'box'),'size').text='.004 .024 .004' if fixed_grasp else '.012 .014 .004'
        if fixed_grasp and kind=='collision':
            ode=ET.SubElement(ET.SubElement(ET.SubElement(e,'surface'),'friction'),'ode')
            for coefficient in ('mu','mu2'):ET.SubElement(ode,coefficient).text='1.2'
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
    p.add_argument('--execute-fixed-grasp',action='store_true',help='Explicit fixed-point controlled contact, close and 30mm lift')
    p.add_argument('--target',type=Path)
    args=p.parse_args()
    if sum([args.execute_approach,args.observe_hover_permit,args.execute_fixed_grasp])>1:
        p.error('Choose permission, no-close approach OR explicit fixed grasp')
    args.target=args.target or Path(__file__).with_name('known_target_fixed.json' if args.execute_fixed_grasp else 'known_target.json')
    template=None
    if args.execute_fixed_grasp:
        from fixed_demo_plan import FixedDemoPlan
        template=FixedDemoPlan(args.target);precheck=template.preview();target=template.target
        if not precheck['sampled_geometry_feasible']:p.error('Fixed demo numerical geometry rejected')
    else:target=KnownTarget(json.loads(args.target.read_text()))
    if not args.execute_flight:
        if args.observe_hover_permit or args.execute_approach or args.execute_fixed_grasp: p.error('Execution requires --execute-flight')
        print(json.dumps(target.preview(),indent=2)); return 0
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
    env.pop('ROS_IP',None); make_world(folder,target,env,args.execute_approach or args.execute_fixed_grasp,args.execute_fixed_grasp)
    if template:save(folder/'fixed_demo_plan.json',precheck)
    # Parent ROS probe and owned children must use the same local master.
    os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'],ROS_HOSTNAME='localhost')
    os.environ.pop('ROS_IP',None)
    files=[Path(__file__),Path(__file__).with_name('known_target.py'),args.target,folder/'scene.world',
        WS/'src/super_px4_bridge/src/super_flight_manager.cpp',WS/'devel/lib/libso101_coordinate_truth.so']
    if args.observe_hover_permit: files.append(Path(__file__).with_name('hover_permit_probe.py'))
    if args.execute_approach: files.append(Path(__file__).with_name('hover_approach_probe.py'))
    if args.execute_fixed_grasp:
        files += [Path(__file__).with_name('fixed_demo_plan.py'),Path(__file__).with_name('fixed_grasp_probe.py'),
            Path(__file__).with_name('native_client.py'),
            ROOT/'src/modules/arm_control/grasp/FixedGrasp.hpp',ROOT/'src/modules/arm_control/grasp/FixedGraspAdapter.cpp']
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
        no_close_approach=args.execute_approach,
        hover_prepare_only=args.observe_hover_permit,lidar=False,vehicle_fixed=False,object_attached=False,
        arm_motion_requests_permitted=args.execute_approach or args.execute_fixed_grasp,
        fixed_grasp_permitted=args.execute_fixed_grasp,gripper_requests_sent=False,grasp_tested=False,
        localization_source='LIVE_GAZEBO_TRUTH_CONTROL_TRANSFORM_AND_VELOCITY_OFFSET_COMPENSATION',
        ekf_truth_injected=False,px4_parameter_overrides=parameter_overrides,
        flight_pid_modified=any(name.startswith(('MPC_','MC_')) for name in parameter_overrides),
        planned=target.preview(),
        precision_hover=dict(additional_flight_controller=False,
            strategy='BASE_HOLD_AFTER_UNFOLD_THEN_WORLD_TCP_IK'),
        hashes={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files}))
    procs=[]; logs=[]; subscribers=None; publisher=None; hover_probe=None
    def start(name,cmd,cwd=None):
        log=(folder/(name+'.log')).open('w'); logs.append(log)
        proc=sp.Popen(cmd,env=env,cwd=cwd,stdout=log,stderr=sp.STDOUT,start_new_session=True)
        procs.append(proc); return proc
    started=time.monotonic(); result=dict(success=False,grasp_tested=False)
    armed_seen=False; last_sim=None; progress=time.monotonic(); rows=[]
    def safety():
        if hover_probe and hover_probe.error: raise RuntimeError(hover_probe.error)
        if any(proc.poll() is not None for proc in procs): raise RuntimeError('Own simulation process exited')
        if time.monotonic()-started>(440 if (args.execute_approach or args.execute_fixed_grasp) else 260): raise RuntimeError('Wall watchdog exceeded')
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
        px4_proc=start('px4',[str(BUILD/'bin/px4'),'-d',str(BUILD/'etc')],folder/'rootfs')
        start('mavros',['roslaunch','mavros','px4.launch','fcu_url:=udp://:14540@127.0.0.1:14580'])
        while True:
            safety(); row=subscribers.read()
            if row and row['now']>=20: break
            if time.monotonic()-started>100: raise RuntimeError('Sources not ready')
            time.sleep(.1)
        identity=(row['truth'].source_instance,row['truth'].world_epoch,row['truth'].vehicle_id,row['truth'].target_id)
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
        takeoff_local=row['local'][2]+target.hover[2,3]-row['body'][2]
        if not .3<=takeoff_local<=2.5: raise RuntimeError('Takeoff coordinate invalid')
        if np.linalg.norm(target.hover[:2,3]-np.array(row['body'][:2]))>1.0 or not 1.0<=target.hover[2,3]<=2.0:
            raise RuntimeError('First-stage flight fixture requires <=1m horizontal travel and 1..2m world hover height')
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
        start('flight_manager',['rosrun','super_px4_bridge','super_flight_manager_node',
            '_known_goal_mode:=true','_auto_arm:=true','_takeoff_height:='+str(takeoff_local),
            '_hover_stable_time:=2.0','__name:=known_flight_manager_probe'])
        publisher=rospy.Publisher('/grasp_demo/known_setpoint',KnownFlightSetpoint,queue_size=1)
        route_start=None; route_point=None; stable_since=None; hold_since=None; last_publish=-math.inf
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
                if route_start is None and row['manager']=='HOVER_READY':
                    route_start=now; route_point=body.copy()
                    print('HOVER_READY: known world route begins; arm remains folded.',flush=True)
                if route_start is not None and now-last_publish>=.02:
                    wp,wv=quintic_segment(route_point,target.hover[:3,3],now-route_start,6)
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
                error=float(np.linalg.norm(body-target.hover[:3,3])); speed=float(np.linalg.norm(row['actual_v']))
                record=dict(sim_s=now,armed=state.armed,mode=state.mode,manager=row['manager'],
                    body_world=row['body'],body_speed=speed,hover_error_m=error,tilt_rad=tilt,
                    body_linear_velocity_world=row['actual_v'].tolist(),
                    body_angular_velocity_world=row['actual_w'].tolist(),
                    source_ages_s=row['ages'],fold_error_rad=fold_error,
                    truth_stamp_s=row['truth'].header.stamp.to_sec(),
                    source_instance=row['truth'].source_instance,world_epoch=row['truth'].world_epoch,
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
                        'Physical world goal reached; observing 10-second no-close hold.',flush=True)
                if (hold_since is not None and now-hold_since>=10 and
                        (hover_probe is None or hover_probe.complete)):
                    held=[r for r in rows if r['sim_s']>=hold_since]
                    hold_pass=bool(all(r['hover_error_m']<=.05 and r['tilt_rad']<=math.radians(5) for r in held))
                    result.update(success=hold_pass,flight_goal_validated=hold_pass,
                        coarse_arrival_observed=True,hover_grasp_permission_validated=False,
                        hold_max_fold_error_rad=max(r['fold_error_rad'] for r in held),
                        actual_object_position_checked=True,hold_sim_seconds=now-hold_since,
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
        result.update(success=False,error=str(error),final_sim_s=last_sim)
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
