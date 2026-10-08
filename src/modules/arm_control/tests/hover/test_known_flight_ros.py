#!/usr/bin/env python3
"""Synthetic isolated ROS contract test; NO PX4/Gazebo/MAVROS or physical flight."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess as sp
import time
import rosgraph
import rospy
from mavros_msgs.msg import State, PositionTarget
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from super_px4_bridge.msg import KnownFlightSetpoint

ROOT=Path(__file__).resolve().parents[5]
FOLDER=ROOT/'build/known_target_flight/contract_ros_20261008_01'
PORT=11361


def stop(p):
    if p is None or p.poll() is not None: return
    for sig,seconds in ((signal.SIGINT,4),(signal.SIGTERM,2),(signal.SIGKILL,2)):
        try: os.killpg(p.pid,sig)
        except ProcessLookupError: return
        try: p.wait(timeout=seconds); return
        except sp.TimeoutExpired: pass


def main():
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1',PORT))==0: raise RuntimeError('Contract port occupied')
    FOLDER.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy(); env.update(ROS_MASTER_URI='http://127.0.0.1:'+str(PORT),ROS_HOSTNAME='localhost')
    env.pop('ROS_IP',None); os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'],ROS_HOSTNAME='localhost'); os.environ.pop('ROS_IP',None)
    logs=[]; core=manager=None; pubs=[]; subs=[]; states=[]; raw=[]; tick=1.; checks={}
    try:
        log=(FOLDER/'core.log').open('w'); logs.append(log)
        core=sp.Popen(['roscore','-p',str(PORT)],env=env,stdout=log,stderr=sp.STDOUT,start_new_session=True)
        deadline=time.monotonic()+15
        while not rosgraph.is_master_online():
            if core.poll() is not None or time.monotonic()>deadline: raise RuntimeError('No isolated master')
            time.sleep(.1)
        sp.run(['rosparam','set','/use_sim_time','true'],env=env,check=True,timeout=10)
        rospy.init_node('synthetic_known_contract',anonymous=True,disable_signals=True)
        clock=rospy.Publisher('/clock',Clock,queue_size=5)
        state=rospy.Publisher('/mavros/state',State,queue_size=5)
        odom=rospy.Publisher('/mavros/local_position/odom',Odometry,queue_size=5)
        target=rospy.Publisher('/grasp_demo/known_setpoint',KnownFlightSetpoint,queue_size=10)
        pubs=[clock,state,odom,target]
        subs=[rospy.Subscriber('/grasp_demo/flight_state',String,lambda m:states.append(m.data),queue_size=100),
            rospy.Subscriber('/mavros/setpoint_raw/local',PositionTarget,lambda m:raw.append(m),queue_size=100)]
        log=(FOLDER/'manager.log').open('w'); logs.append(log)
        manager=sp.Popen(['rosrun','super_px4_bridge','super_flight_manager_node','_known_goal_mode:=true',
            '_auto_arm:=false','_takeoff_height:=1.0','_prestream_time:=0.1','_hover_stable_time:=0.1',
            '_map_warmup_time:=0.1','__name:=isolated_known_contract_manager'],env=env,
            stdout=log,stderr=sp.STDOUT,start_new_session=True)
        def command(task,x,frame='map',stamp=None):
            m=KnownFlightSetpoint(); m.header.stamp=rospy.Time.from_sec(tick if stamp is None else stamp)
            m.header.frame_id=frame; m.task_id=task; m.setpoint.header=m.header
            m.setpoint.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
            m.setpoint.type_mask=PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE
            m.setpoint.position.x=x; m.setpoint.position.z=1
            return m
        def pump(commands=()):
            nonlocal tick
            if manager.poll() is not None: raise RuntimeError('Isolated manager exited')
            tick+=.05
            clock.publish(Clock(rospy.Time.from_sec(tick)))
            s=State(); s.header.stamp=rospy.Time.from_sec(tick); s.connected=True; s.armed=True; s.mode='OFFBOARD'; state.publish(s)
            o=Odometry(); o.header.stamp=s.header.stamp; o.header.frame_id='map'; o.child_frame_id='base_link'
            o.pose.pose.position.z=1; o.pose.pose.orientation.w=1; odom.publish(o)
            for cmd in commands: target.publish(cmd)
            time.sleep(.02)
        deadline=time.monotonic()+12
        while (not states or states[-1]!='HOVER_READY') and time.monotonic()<deadline: pump()
        if not states or states[-1]!='HOVER_READY': raise RuntimeError('Synthetic hover not ready')
        for _ in range(5): pump([command(41,.2,'gazebo_world')])
        checks['wrong_frame_rejected']=states[-1]=='HOVER_READY'
        for _ in range(8): pump([command(41,.2)])
        checks['valid_task_accepted']=states[-1]=='TRACK_KNOWN' and raw and abs(raw[-1].position.x-.2)<1e-6
        for _ in range(5): pump([command(41,.2),command(42,.6)])
        checks['other_task_cannot_replace']=abs(raw[-1].position.x-.2)<1e-6
        old=tick-.1
        for _ in range(5): pump([command(41,.2),command(41,.7,stamp=old)])
        checks['old_packet_cannot_replace']=abs(raw[-1].position.x-.2)<1e-6
        for _ in range(12): pump()
        checks['stream_loss_enters_failsafe']=states[-1]=='FAILSAFE_HOVER'
        for _ in range(8): pump([command(41,.2)])
        checks['no_auto_resume']=states[-1]=='FAILSAFE_HOVER' and abs(raw[-1].position.x)<1e-6
        result=dict(success=all(checks.values()),checks=checks,synthetic_only=True,physical_flight_tested=False,
            px4_started=False,gazebo_started=False,mavros_started=False,source_master=env['ROS_MASTER_URI'])
    finally:
        for p in pubs: p.unregister()
        for s in subs: s.unregister()
        stop(manager); stop(core)
        for log in logs: log.close()
    (FOLDER/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    if not result['success']: raise SystemExit(1)


if __name__=='__main__': main()
