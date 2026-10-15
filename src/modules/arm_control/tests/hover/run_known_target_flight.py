#!/usr/bin/env python3
"""Stage 2 bounded local-SITL known-target flight; never sends arm/gripper targets."""
import argparse
import csv
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
import numpy as np
import rosgraph
import rospy
from geometry_msgs.msg import Pose
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from super_px4_bridge.msg import CoordinateTruth, KnownFlightSetpoint
from known_target import KnownTarget, rotation, quintic_segment, local_motion

ROOT=Path(__file__).resolve().parents[5]
BUILD=ROOT/'build/px4_sitl_default'
WS=Path('/home/pcz/super_ws')
MODELS=ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
sys.path.insert(0,str(ROOT/'src/modules/arm_control/tests/grasp'))
from run_full_arm_grasp import remove_lidar


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
        self.subscribers=[
            rospy.Subscriber('/mavros/state',State,lambda m:self.update('state',m),queue_size=50),
            rospy.Subscriber('/mavros/local_position/odom',Odometry,lambda m:self.update('odom',m),queue_size=100),
            rospy.Subscriber('/grasp_audit/truth',CoordinateTruth,lambda m:self.update('truth',m),queue_size=100),
            rospy.Subscriber('/grasp_demo/flight_state',String,lambda m:self.update('manager',m.data),queue_size=20)]
    def update(self,key,value):
        with self.lock: setattr(self,key,value)
    def read(self):
        with self.lock: state,odom,truth,manager=self.state,self.odom,self.truth,self.manager
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
            truth=truth,ages=ages,joints=joint_values)


def make_world(folder,target,env):
    expanded=sp.check_output(['gz','sdf','-p',str(MODELS/'yhang550/yhang550.sdf')],
        env=env,text=True,timeout=30)
    model=ET.fromstring(expanded).find('model'); remove_lidar(model)
    p=model.find('pose')
    if p is None: p=ET.SubElement(model,'pose')
    p.text='0 0 0 0 0 0'
    if model.find("joint[@name='bench_anchor']") is not None: raise RuntimeError('Unexpected fixed bench')
    tree=ET.parse(ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world')
    tree.getroot().set('version','1.7'); world=tree.find('world')
    world.find('physics/ode/solver/iters').text='40'
    world.append(model)
    R=target.object[:3,:3]
    rpy=[math.atan2(R[2,1],R[2,2]),math.asin(np.clip(-R[2,0],-1,1)),math.atan2(R[1,0],R[0,0])]
    cube=ET.parse(MODELS/'test_cube/model.sdf').find('model')
    p=cube.find('pose')
    if p is None: p=ET.SubElement(cube,'pose')
    p.text=' '.join(map(str,[*target.object[:3,3],*rpy])); world.append(cube)
    support=ET.SubElement(world,'model',name='cube_support'); ET.SubElement(support,'static').text='true'
    down=1 if R[2,2]<0 else -1
    center=target.object[:3,3]+down*R[:,2]*.012
    ET.SubElement(support,'pose').text=' '.join(map(str,[*center,*rpy]))
    link=ET.SubElement(support,'link',name='link')
    for kind in ('collision','visual'):
        e=ET.SubElement(link,kind,name=kind)
        ET.SubElement(ET.SubElement(ET.SubElement(e,'geometry'),'box'),'size').text='.012 .014 .004'
    plugin=ET.SubElement(world,'plugin',name='coordinate_truth',filename='libso101_coordinate_truth.so')
    ET.SubElement(plugin,'vehicleName').text='yhang550'; ET.SubElement(plugin,'targetName').text=target.name
    tree.write(folder/'scene.world',encoding='unicode',xml_declaration=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('name'); p.add_argument('--execute-flight',action='store_true')
    p.add_argument('--observe-hover-permit',action='store_true',
        help='Test PREPARE/refusal/revocation only; never move the arm or close the jaw')
    p.add_argument('--target',type=Path,default=Path(__file__).with_name('known_target.json'))
    args=p.parse_args()
    target=KnownTarget(json.loads(args.target.read_text()))
    if not args.execute_flight:
        if args.observe_hover_permit: p.error('--observe-hover-permit requires --execute-flight')
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
    env.pop('ROS_IP',None); make_world(folder,target,env)
    # Parent ROS probe and owned children must use the same local master.
    os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'],ROS_HOSTNAME='localhost')
    os.environ.pop('ROS_IP',None)
    files=[Path(__file__),Path(__file__).with_name('known_target.py'),args.target,folder/'scene.world',
        WS/'src/super_px4_bridge/src/super_flight_manager.cpp',WS/'devel/lib/libso101_coordinate_truth.so']
    if args.observe_hover_permit: files.append(Path(__file__).with_name('hover_permit_probe.py'))
    if args.observe_hover_permit:
        files += [BUILD/'bin/px4', ROOT/'src/modules/arm_control/grasp/HoverPermit.hpp',
            ROOT/'src/modules/arm_control/grasp/HoverPermitAdapter.cpp',
            WS/'src/super_px4_bridge/src/coordinate_truth_plugin.cpp',
            WS/'src/super_px4_bridge/msg/CoordinateTruth.msg']
    save(folder/'manifest.json',dict(stage=3 if args.observe_hover_permit else 2,
        hover_prepare_only=args.observe_hover_permit,lidar=False,vehicle_fixed=False,object_attached=False,
        arm_motion_requests_sent=False,gripper_requests_sent=False,grasp_tested=False,
        localization_source='LIVE_GAZEBO_TRUTH_CONTROL_TRANSFORM_AND_VELOCITY_OFFSET_COMPENSATION',
        ekf_truth_injected=False,flight_pid_modified=False,planned=target.preview(),
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
        if time.monotonic()-started>260: raise RuntimeError('Wall watchdog exceeded')
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
        start('px4',[str(BUILD/'bin/px4'),'-d',str(BUILD/'etc')],folder/'rootfs')
        start('mavros',['roslaunch','mavros','px4.launch','fcu_url:=udp://:14540@127.0.0.1:14580'])
        while True:
            safety(); row=subscribers.read()
            if row and row['now']>=20: break
            if time.monotonic()-started>100: raise RuntimeError('Sources not ready')
            time.sleep(.1)
        identity=(row['truth'].source_instance,row['truth'].world_epoch,row['truth'].vehicle_id,row['truth'].target_id)
        object_pose=row['truth'].target_pose
        if row['truth'].target_name!=target.name or not row['truth'].target_id or np.linalg.norm(
            np.asarray(values(object_pose)[:3])-target.object[:3,3])>.004:
            raise RuntimeError('Actual fixed object position does not match the known target input')
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
                if fold_error>.08: raise RuntimeError('Folded arm departed initial target')
                if now>120: raise RuntimeError('Simulation-time limit exceeded')
                if route_start is None and row['manager']=='HOVER_READY':
                    route_start=now; route_point=body.copy()
                    print('HOVER_READY: known world route begins; arm remains folded.',flush=True)
                if route_start is not None and now-last_publish>=.02:
                    wp,wv=quintic_segment(route_point,target.hover[:3,3],now-route_start,6)
                    lp,lv,yaw=local_motion(wp,wv,target.yaw,row['body'],row['local'],
                        row['actual_v'],row['estimated_body_v'])
                    cmd=KnownFlightSetpoint(); cmd.header.stamp=rospy.Time.now(); cmd.header.frame_id='map'; cmd.task_id=1
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
                    close_permitted=False)
                if not rows or now-rows[-1]['sim_s']>=.05:
                    rows.append(record); out.write(json.dumps(record,allow_nan=False)+'\n'); out.flush()
                good=(route_start is not None and now-route_start>=6 and row['manager']=='TRACK_KNOWN'
                    and error<=.03 and speed<=.05 and tilt<=math.radians(5))
                if not good: stable_since=None
                elif stable_since is None: stable_since=now
                elif now-stable_since>=2 and hold_since is None:
                    hold_since=now; print('Physical world goal reached; observing 10-second no-close hold.',flush=True)
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
            result['hover_prepare']=hover_probe.finish(); hover_probe=None
            if not result['hover_prepare']['hover_prepare_validated']:
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
            try: result['hover_prepare']=hover_probe.finish()
            except Exception as error:
                result.update(success=False,hover_probe_error=str(error))
                result['hover_prepare_events']=hover_probe.events
        if publisher: publisher.unregister()
        if subscribers:
            for sub in subscribers.subscribers: sub.unregister()
        for proc in reversed(procs): stop(proc)
        for log in logs: log.close()
        result.update(armed_seen=armed_seen,arm_motion_requests_sent=False,gripper_requests_sent=False,
            grasp_tested=False,own_processes_stopped=True,wall_elapsed_s=time.monotonic()-started)
        save(folder/'result.json',result); print(json.dumps(result,indent=2,allow_nan=False),flush=True)
    return 0 if result['success'] else 2


if __name__=='__main__': raise SystemExit(main())
