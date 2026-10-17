#!/usr/bin/env python3
"""Unarmed native IK inspection. Never sends targets, PREPARE or grasp requests."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess as sp
import time
import numpy as np
from fixed_grasp_target import ROOT,AUDIT,load_target,parse_native
from known_target import pose
from native_client import NativeClient


def evidence_json(value):
    """Retain idle 'not measured' sentinels without weakening runtime checks."""
    if isinstance(value,dict): return {key:evidence_json(item) for key,item in value.items()}
    if isinstance(value,(list,tuple)): return [evidence_json(item) for item in value]
    if isinstance(value,float) and not np.isfinite(value): return str(value)
    return value


def snapshot(client):
    return {topic:client.status(topic) for topic in
        ('arm_control_status','arm_grasp_status','arm_approach_status')}


def assert_unchanged(before,after):
    """Compare published targets AND cached IK/ownership state after each query."""
    baseline=before['arm_control_status']; current=after['arm_control_status']
    for row in (baseline,current):
        if not row.get('feedback_valid') or not np.isfinite(row.get('feedback_age',np.inf)) or row['feedback_age']>.2:
            raise RuntimeError('Arm feedback not ready')
        q=np.asarray(row.get('joint_target',[]))
        if q.shape!=(6,) or not np.isfinite(q).all(): raise RuntimeError('Invalid joint targets')
    for key in ('target_sequence','ik_status','ik_error'):
        if baseline[key]!=current[key]: raise RuntimeError('IK inspection changed '+key)
    delta=float(np.max(np.abs(np.asarray(current['joint_target'])-baseline['joint_target'])))
    if delta>1e-6: raise RuntimeError('IK inspection changed a joint target')
    for topic in ('arm_grasp_status','arm_approach_status'):
        previous,state=before[topic],after[topic]
        if (previous['ownership_locked'] or state['ownership_locked'] or state['phase']!=previous['phase']
                or state['last_request_id']!=previous['last_request_id']):
            raise RuntimeError('A motion/grasp request was issued')
    return dict(joint_target_max_delta_rad=delta,target_sequence_unchanged=True,
        ik_cache_unchanged=True,request_and_ownership_unchanged=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name',nargs='?',default='preview_only')
    parser.add_argument('--native-sim',action='store_true',help='Start an unarmed local inspection session')
    parser.add_argument('--target',type=Path,default=Path(__file__).with_name('known_target_approach.json'))
    args=parser.parse_args(); fixed=load_target(args.target)
    if not args.native_sim:
        print(json.dumps(fixed.preview(),indent=2)); return 0
    if not args.name.replace('_','').isalnum(): parser.error('Use a safe unique case name')
    # ROS and all control-environment imports happen ONLY after explicit opt-in.
    import rospy
    import rosgraph
    from run_known_target_flight import BUILD,WS,MODELS,Sources,make_world,stop,save as save_json
    def save(path,value): save_json(path,evidence_json(value))
    for name in ('px4','gzserver','gzclient','rosmaster','mavros_node'):
        if sp.run(['pgrep','-x',name],stdout=sp.DEVNULL).returncode==0:
            parser.error('Existing simulation detected; not disturbing it')
    for port in (11311,11345,4560):
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1',port))==0: parser.error('Occupied port '+str(port))
    folder=ROOT/'build/fixed_target_checks'/args.name
    folder.mkdir(parents=True,exist_ok=False); (folder/'rootfs').mkdir(); (folder/'native').mkdir()
    env=os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:11311',ROS_HOSTNAME='localhost',
        GAZEBO_MASTER_URI='http://127.0.0.1:11345',GAZEBO_MODEL_PATH=str(MODELS),
        GAZEBO_PLUGIN_PATH=str(WS/'devel/lib')+':'+str(BUILD/'build_gazebo-classic')+
            ':/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
        LD_LIBRARY_PATH=str(WS/'devel/lib')+':'+str(BUILD/'build_gazebo-classic')+
            ':/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins',
        GAZEBO_MODEL_DATABASE_URI='',PX4_SIM_MODEL='gazebo-classic_yhang550',PX4_SYS_AUTOSTART='1020')
    env.pop('ROS_IP',None)
    os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'],ROS_HOSTNAME='localhost');os.environ.pop('ROS_IP',None)
    make_world(folder,fixed.known,env,approach_guard=True)
    files=[Path(__file__),Path(__file__).with_name('fixed_grasp_target.py'),Path(__file__).with_name('native_client.py'),args.target,AUDIT,
        Path(__file__).with_name('known_target.py'),Path(__file__).with_name('run_known_target_flight.py'),
        ROOT/'src/modules/arm_control/tests/grasp/run_full_arm_grasp.py',
        MODELS/'so101/so101.sdf',
        BUILD/'bin/px4',ROOT/'src/modules/arm_control/diagnostics/IKPreview.cpp',
        ROOT/'src/modules/arm_control/arm_control.cpp',ROOT/'src/modules/arm_control/arm_control.hpp',folder/'scene.world']
    save(folder/'manifest.json',dict(read_only_native_ik=True,flight_allowed=False,arm_requests_allowed=False,
        gripper_requests_allowed=False,min_clearance_m=.016,contact_exemptions=[],
        planned=fixed.preview(),hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}))
    procs=[];logs=[];sources=None;started=time.monotonic()
    result=dict(inspection_verified=False,flight_tested=False,grasp_tested=False,
        motion_requests_sent=False,gripper_requests_sent=False)
    def start(name,cmd,cwd=None):
        log=(folder/(name+'.log')).open('w');logs.append(log)
        proc=sp.Popen(cmd,env=env,cwd=cwd,stdout=log,stderr=sp.STDOUT,start_new_session=True)
        procs.append(proc);return proc
    def safety():
        if time.monotonic()-started>130: raise RuntimeError('Inspection watchdog')
        if any(p.poll() is not None for p in procs): raise RuntimeError('Owned process exited')
        row=sources.read() if sources else None
        if row and row['state'].armed: raise RuntimeError('Unexpected arming; no flight controller was started')
        return row
    try:
        start('roscore',['roscore'])
        while not rosgraph.is_master_online(): safety();time.sleep(.1)
        sp.run(['rosparam','set','/use_sim_time','true'],env=env,check=True,timeout=10)
        rospy.init_node('fixed_target_ik_probe',anonymous=True,disable_signals=True);sources=Sources()
        start('gazebo',['gzserver','--verbose','--seed','123','-s','libgazebo_ros_api_plugin.so',str(folder/'scene.world')])
        start('px4',[str(BUILD/'bin/px4'),'-d',str(BUILD/'etc')],folder/'rootfs')
        start('mavros',['roslaunch','mavros','px4.launch','fcu_url:=udp://:14540@127.0.0.1:14580'])
        while True:
            row=safety()
            if row and row['now']>=20:break
            time.sleep(.05)
        client=NativeClient(BUILD,env)
        baseline=snapshot(client); assert_unchanged(baseline,baseline)
        save(folder/'before.json',baseline);result['initial_state']=baseline
        actual_arm=pose(row['arm'][:3],row['arm'][3:]);actual_goal=fixed.in_arm(actual_arm)[:3,3]
        planned_goal=fixed.in_arm(fixed.planned_arm)[:3,3]
        requests=[('planned_hover_hypothesis',planned_goal),('actual_unarmed_snapshot',actual_goal),
            ('outside_workspace',[.5,0,.05]),('inside_workspace_unreachable',[.39,.24,.39]),
            ('repeat_planned_hover',planned_goal)]
        checks={};checkpoints={}
        for label,goal in requests:
            safety();output=client.client('arm_control','ik_check',*map(float,goal))
            # Save raw output before parsing so a formatting failure is diagnosable.
            (folder/'native'/(label+'.log')).write_text(output)
            after=snapshot(client);save(folder/'native'/(label+'_state.json'),after)
            checkpoints[label]=assert_unchanged(baseline,after)
            checks[label]=dict(input_position_arm_m=list(map(float,goal)),native=parse_native(output))
            save(folder/'native_checks.json',checks);save(folder/'invariants.json',checkpoints)
        for label,values in [('nonfinite',['nan','0','.07']),('infinite',['0','inf','.07']),
                ('trailing_text',['0.2bad','0','.07']),('missing_argument',['.2','0'])]:
            safety()
            bad=sp.run([str(BUILD/'bin/px4-arm_control'),'ik_check',*values],env=env,
                stdout=sp.PIPE,stderr=sp.STDOUT,text=True,timeout=4)
            (folder/'native'/(label+'.log')).write_text(bad.stdout)
            after=snapshot(client);save(folder/'native'/(label+'_state.json'),after)
            checkpoints[label]=dict(assert_unchanged(baseline,after),refused=bad.returncode!=0)
            save(folder/'invariants.json',checkpoints)
            if bad.returncode==0:raise RuntimeError('Malformed input accepted: '+label)
        if not checks['planned_hover_hypothesis']['native']['ik_success']:raise RuntimeError('Planned fixed point IK failed')
        if checks['actual_unarmed_snapshot']['native']['workspace_valid']:raise RuntimeError('Grounded pose should not validate the planned hover hypothesis')
        if checks['outside_workspace']['native']['workspace_valid']:raise RuntimeError('Workspace refusal failed')
        if not checks['inside_workspace_unreachable']['native']['workspace_valid']:raise RuntimeError('Unreachable test did not exercise native IK')
        if checks['inside_workspace_unreachable']['native']['ik_success']:raise RuntimeError('Unreachable input accepted')
        if checks['repeat_planned_hover']['native']!=checks['planned_hover_hypothesis']['native']:
            raise RuntimeError('Repeated read-only query was not deterministic')
        after=snapshot(client);checkpoints['final']=assert_unchanged(baseline,after)
        save(folder/'after.json',after);save(folder/'invariants.json',checkpoints)
        final=safety();graph=rosgraph.Master('/fixed_target_graph').getSystemState()
        pubs=[nodes for topic,nodes in graph[0] if topic=='/mavros/setpoint_raw/local']
        if pubs:raise RuntimeError('Unexpected flight setpoint publisher')
        save(folder/'ros_graph.json',dict(publishers=graph[0],subscribers=graph[1]))
        result.update(inspection_verified=True,native_checks=checks,invariants=checkpoints,
            nominal_ik_valid=True,min_clearance_m=.016,clearance_or_path_tested=False,
            grasp_target_accepted=False,motion_permitted=False,closing_permitted=False,
            final_state=after,malformed_inputs_refused=True,
            actual_arm_pose_world=actual_arm.tolist(),actual_pose_is_unarmed_ground=True,
            final_armed=final['state'].armed,final_sim_s=final['now'],flight_setpoint_publishers=pubs)
    except (Exception,KeyboardInterrupt) as exc:result['error']=str(exc)
    finally:
        if sources:
            for sub in sources.subscribers:sub.unregister()
        for proc in reversed(procs):stop(proc)
        for log in logs:log.close()
        result.update(own_processes_stopped=True,wall_elapsed_s=time.monotonic()-started)
        save(folder/'result.json',result);print(json.dumps(evidence_json(result),indent=2,allow_nan=False))
    return 0 if result['inspection_verified'] else 2


if __name__=='__main__':raise SystemExit(main())
