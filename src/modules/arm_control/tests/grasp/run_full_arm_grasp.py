#!/usr/bin/env python3
"""Full yhang550/SO101 experiment: PX4 IK, real contacts, arm-only retrieval.

First run --calibrate without an object, then --calibration <case>/calibration.json.
Default object mode only measures/aligns. --grasp explicitly closes the jaws;
--grasp --retrieve additionally tests arm retrieval. No firmware state machine.
The vehicle is fixed to world for this bench test. No object attachment/teleport.
Phase durations use simulation time; wall time is only a deadlock watchdog.
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import subprocess as sp
import time
import xml.etree.ElementTree as ET
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
MODELS=ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
BUILD=ROOT/'build/px4_sitl_default'
PICKUP=np.array([.18,.015,.07])
RETRACT=np.array([.12,.015,.03])
INSPECTION=np.array([.17,.035,.05])
# Candidate grasp reference from the tested collision proxies, NOT the
# instantaneous midpoint of the open fingers or an independently calibrated TCP.
FINGER_CENTER=np.array([.0025,-.000218,-.088])

def grasp_reference(cube_mm):
    reference=FINGER_CENTER.copy()
    if cube_mm<20:
        # Fixed inner pad plane: -14 mm + 12 mm / 2 = -8 mm.
        # The smaller cube must actually reach that face; the 20 mm
        # reference cannot be reused unchanged or only one jaw contacts it.
        reference[0]=-.008+cube_mm*.0005
    return reference

def rotation(q):
    q=np.asarray(q,dtype=float)
    norm=np.linalg.norm(q)
    if not np.isfinite(q).all() or not math.isfinite(norm) or norm<1e-9:
        raise RuntimeError('invalid pose quaternion')
    w,x,y,z=q/norm
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def gripper_pose(row):
    return np.array([row[k] for k in ['gx','gy','gz']]), rotation([row[k] for k in ['gw','gqx','gqy','gqz']])

def cube_local(row):
    pos,R=gripper_pose(row)
    return R.T@(np.array([row[k] for k in ['cx','cy','cz']])-pos)

def box_bounds(position,R,local_center,half_size):
    """Conservative world AABB for a box whose local axes match R."""
    center=np.asarray(position)+R@np.asarray(local_center)
    extent=np.abs(R)@np.asarray(half_size)
    return center-extent,center+extent

def set_pad_patch(collision,radius_mm):
    """Finite, force-dependent torsional pad friction, not an attachment.

    ODE combines patch radii by MAX and torsional coefficients by MIN.
    Only the two finger surfaces are changed; cube/support torsion stays
    at its zero-radius default. 4 mm is a demo pad model, not a calibration.
    """
    friction=collision.find('surface/friction')
    if friction is None: raise RuntimeError('pad friction element missing')
    torsion=friction.find('torsional')
    if torsion is None: torsion=ET.SubElement(friction,'torsional')
    for name,value in [('coefficient','.8'),('use_patch_radius','true'),('patch_radius',str(radius_mm*.001))]:
        element=torsion.find(name)
        if element is None: element=ET.SubElement(torsion,name)
        element.text=value

def contact_summary(data):
    """Weight windows by received physics frames, not by CSV row count."""
    count=sum(r['contact_frames'] for r in data)
    fields=['bilateral_fraction','cube_support_fraction','cube_other_fraction','support_arm_fraction',
            'fixed_normal_N','moving_normal_N','finger_vertical_N',
            'contact_torque_x_Nm','contact_torque_y_Nm','contact_torque_z_Nm',
            'fixed_contact_points','moving_contact_points']
    return {key:sum(r.get(key,0)*r['contact_frames'] for r in data)/max(1,count) for key in fields}

def retention_checks(contact,late_drift,max_drift,min_lift):
    """Separate physical demo retention from optional precision requirements.

    Entry to this phase already requires an actual >=25 mm lift, no support,
    and the continuous payload guard. Slip is recorded even when demo passes.
    """
    measured=[contact[k] for k in ['bilateral_fraction','cube_support_fraction','fixed_normal_N','moving_normal_N']]
    if not all(math.isfinite(v) for v in [*measured,late_drift,max_drift,min_lift]):
        return False,False
    retained=(contact['bilateral_fraction']>=.9 and contact['cube_support_fraction']<=.001
              and min_lift>0 and min(contact['fixed_normal_N'],contact['moving_normal_N'])>.01)
    strict=retained and late_drift<=.005 and max_drift<=.010 and min_lift>=.025
    # NumPy coordinates can propagate np.bool_ through these comparisons.
    # Keep the experiment result/events JSON-compatible after physical completion.
    return bool(retained),bool(strict)

def rows(path):
    if not path.exists(): return []
    result=[]
    with path.open() as stream:
        for r in csv.DictReader(stream):
            try:
                if None in r or any(v is None for v in r.values()): continue
                result.append({k:float(v) for k,v in r.items()})
            except ValueError: continue
    return result

def stop(p):
    if p is not None and p.poll() is None:
        p.send_signal(signal.SIGINT)
        try: p.wait(timeout=5)
        except sp.TimeoutExpired:
            p.kill();p.wait(timeout=5)

def build_world(folder,env,calibration,iterations=40,cube_mm=20,gripper_p=None,solver='quick',patch_radius_mm=0):
    # Expand the same production model used by normal SITL, not a two-link fixture.
    expanded=sp.check_output(['gz','sdf','-p',str(MODELS/'yhang550/yhang550.sdf')],env=env,text=True,timeout=30)
    model=ET.fromstring(expanded).find('model')
    if gripper_p is not None:
        controllers=model.findall("plugin[@name='gripper_controller']")
        if len(controllers)!=1 or controllers[0].find('p') is None:
            raise RuntimeError('cannot uniquely identify gripper P in expanded model')
        controllers[0].find('p').text=str(gripper_p)
    if patch_radius_mm>0:
        pads=[c for c in model.iter('collision') if c.get('name','').split('::')[-1]
              in ['gripper_fixed_finger_collision','moving_jaw_collision']]
        if len(pads)!=2: raise RuntimeError('cannot uniquely identify both finger pads')
        for pad in pads: set_pad_patch(pad,patch_radius_mm)
    pose=model.find('pose')
    if pose is None: pose=ET.SubElement(model,'pose')
    pose.text='0 0 1.2 0 0 0'
    anchor=ET.SubElement(model,'joint',name='bench_anchor',type='fixed')
    ET.SubElement(anchor,'parent').text='world'
    ET.SubElement(anchor,'child').text='yhang550_base::base_link'
    tree=ET.parse(ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world')
    tree.getroot().set('version','1.7')
    world=tree.find('world')
    world.find('physics/ode/solver/iters').text=str(iterations) # bench-only contact setting
    world.find('physics/ode/solver/type').text=solver
    world.append(model)
    ET.SubElement(world,'plugin',name='clock',filename=str(folder/'libgrasp_clock.so'))
    if calibration is not None:
        c=calibration['cube_world_position'];q=calibration['gripper_world_quaternion']
        R=rotation(q)
        # SDF pose is RPY; the contact box is aligned with the measured finger frame.
        pitch=math.asin(max(-1.,min(1.,-R[2,0])))
        roll=math.atan2(R[2,1],R[2,2]);yaw=math.atan2(R[1,0],R[0,0])
        cube=ET.parse(MODELS/'test_cube/model.sdf').getroot().find('model')
        # Bench-only size comparison. Keep mass unchanged and recompute the
        # solid-box inertia consistently; do not edit the production asset.
        edge=cube_mm*.001
        cube_link=cube.find('link')
        for box in cube_link.findall('collision/geometry/box')+cube_link.findall('visual/geometry/box'):
            box.find('size').text=' '.join([str(edge)]*3)
        mass=float(cube_link.find('inertial/mass').text)
        inertia=cube_link.find('inertial/inertia')
        for axis in ['ixx','iyy','izz']: inertia.find(axis).text=str(mass*edge*edge/6)
        pose=cube.find('pose')
        if pose is None: pose=ET.SubElement(cube,'pose')
        pose.text=' '.join(map(str,[*c,roll,pitch,yaw]));world.append(cube)
        support=ET.SubElement(world,'model',name='cube_support')
        ET.SubElement(support,'static').text='true'
        # Support the lower cube face, not one tilted corner. A thin plate stays
        # outside the gripper body; a tall central pedestal overlaps this upside-down mount.
        down=1. if R[2,2]<0 else -1.
        support_center=np.array(c)+down*R[:,2]*(edge*.5+.002)
        ET.SubElement(support,'pose').text=' '.join(map(str,[*support_center,roll,pitch,yaw]))
        link=ET.SubElement(support,'link',name='link')
        for kind in ['collision','visual']:
            shape=ET.SubElement(link,kind,name=kind)
            ET.SubElement(ET.SubElement(ET.SubElement(shape,'geometry'),'box'),'size').text='.012 .014 .004'
    tree.write(folder/'scene.world',encoding='unicode',xml_declaration=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('name')
    mode=p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--calibrate',action='store_true')
    mode.add_argument('--calibration',type=Path)
    p.add_argument('--port',type=int,default=11356)
    p.add_argument('--gui',action='store_true')
    p.add_argument('--align-only',action='store_true',help='Measure and align fingers to the fixed cube; stop before closing')
    p.add_argument('--grasp',action='store_true',help='Explicitly enable closing and bilateral-contact verification')
    p.add_argument('--retrieve',action='store_true',help='With --grasp, explicitly enable arm retrieval after contact')
    p.add_argument('--retrieve-route',choices=['vertical','stow'],default='vertical',help='Vertical lift only, or additionally retract toward the body')
    p.add_argument('--hold-seconds',type=float,default=30.,help='Unassisted retention interval in SIMULATION seconds')
    p.add_argument('--lift-mm',type=float,default=35.,help='Requested upward motion; cube must actually rise at least 25 mm')
    p.add_argument('--side-clear-mm',type=float,default=70.,help='Exit the fixed support along base +Y before lifting; 0 reproduces vertical interference')
    p.add_argument('--iterations',type=int,default=40,help='Bench-only ODE solver iterations; does not modify production worlds')
    p.add_argument('--acceptance',choices=['demo','strict'],default='demo',help='Demo verifies physical retention; strict also requires <=5 mm final and <=10 mm peak drift')
    p.add_argument('--cube-mm',type=float,default=20.,help='Bench-only cube edge in mm; mass is unchanged and box inertia is recomputed')
    p.add_argument('--gripper-p',type=float,help='Optional bench-only gripper P override; torque cap is unchanged')
    p.add_argument('--solver',choices=['quick','world'],default='quick',help='Bench-only ODE solver; iterations apply only to quick')
    p.add_argument('--patch-radius-mm',type=float,default=0.,help='Bench-only finite torsional finger-pad radius, coefficient 0.8; 0 preserves the baseline')
    p.add_argument('--arrival-mm',type=float,default=12.,help='Demo-only actual EE tolerance; strict PX4 arrival is logged separately')
    args=p.parse_args()
    if not math.isfinite(args.arrival_mm) or not 1 <= args.arrival_mm <= 15: p.error('arrival-mm must be 1..15')
    if not 1024 <= args.port <= 65535: p.error('port must be 1024..65535')
    if args.retrieve and not args.grasp: p.error('retrieve requires grasp')
    if args.align_only and args.grasp: p.error('align-only cannot be combined with grasp')
    if args.calibrate and (args.align_only or args.grasp): p.error('calibration is unloaded; choose no object action')
    if not math.isfinite(args.hold_seconds) or not 5 <= args.hold_seconds <= 180: p.error('hold-seconds must be 5..180')
    if not math.isfinite(args.lift_mm) or not 25 <= args.lift_mm <= 45: p.error('lift-mm must be 25..45')
    if not math.isfinite(args.side_clear_mm) or not 0 <= args.side_clear_mm <= 100: p.error('side-clear-mm must be 0..100')
    if not 20 <= args.iterations <= 240: p.error('iterations must be 20..240')
    if not math.isfinite(args.cube_mm) or not 16 <= args.cube_mm <= 20: p.error('cube-mm must be 16..20')
    if args.gripper_p is not None and (not math.isfinite(args.gripper_p) or not .03 <= args.gripper_p <= .06):
        p.error('gripper-p must be 0.03..0.06')
    if not math.isfinite(args.patch_radius_mm) or not 0 <= args.patch_radius_mm <= 5:
        p.error('patch-radius-mm must be 0..5')
    if not args.name.replace('_','').isalnum(): p.error('use a unique alphanumeric/underscore name')
    finger_center=grasp_reference(args.cube_mm)
    body=ET.parse(MODELS/'so101/so101.sdf').find("model/link[@name='gripper_link']/collision[@name='gripper_body_collision']")
    body_pose=np.array([float(v) for v in body.find('pose').text.split()])
    body_half=np.array([float(v) for v in body.find('geometry/box/size').text.split()])*.5
    if np.linalg.norm(body_pose[3:])>1e-9: p.error('rotated body collision proxy needs an updated clearance calculation')
    folder=ROOT/'build/full_arm_grasp'/args.name
    if folder.exists(): p.error('existing case; choose a new name to preserve its evidence')
    for port in [args.port,4560]:
        with socket.socket() as sock:
            sock.settimeout(.2)
            if sock.connect_ex(('127.0.0.1',port))==0: p.error(f'port {port} occupied; stop your simulation first')
    active=sp.check_output(['pgrep','-a','-x','px4'],text=True) if sp.run(['pgrep','-x','px4'],stdout=sp.DEVNULL).returncode==0 else ''
    if active: p.error('PX4 already running; this test will not interrupt it')
    calibration=json.loads(args.calibration.read_text()) if args.calibration else None
    if calibration and (np.linalg.norm(np.array(calibration['pickup_base'])-PICKUP)>1e-6
                        or calibration.get('model_sha256')!=hashlib.sha256((MODELS/'so101/so101.sdf').read_bytes()).hexdigest()):
        p.error('calibration target/model mismatch; recalibrate')
    folder.mkdir(parents=True);(folder/'rootfs').mkdir()
    env=os.environ.copy()
    env.update(GAZEBO_MASTER_URI=f'http://127.0.0.1:{args.port}',
               GAZEBO_MODEL_PATH=str(MODELS),
               GAZEBO_PLUGIN_PATH=f'/home/pcz/super_ws/devel/lib:{BUILD}/build_gazebo-classic:/opt/ros/noetic/lib',
               LD_LIBRARY_PATH=f'/opt/ros/noetic/lib:/home/pcz/super_ws/devel/lib:{BUILD}/build_gazebo-classic',
               PX4_SIM_MODEL='gazebo-classic_yhang550',PX4_SYS_AUTOSTART='1020')
    flags=shlex.split(sp.check_output(['pkg-config','--cflags','--libs','gazebo'],text=True))
    with (folder/'build.log').open('w') as log:
        for source,output,extra in [('full_arm_probe.cpp','observer',[]),('grasp_probe.cpp','libgrasp_clock.so',['-DSO101_CLOCK_PLUGIN','-shared','-fPIC'])]:
            sp.run(['g++','-std=c++17',str(HERE/source),'-o',str(folder/output),*extra,*flags,'-pthread'],check=True,stdout=log,stderr=sp.STDOUT,timeout=120)
    build_world(folder,env,calibration,args.iterations,args.cube_mm,args.gripper_p,args.solver,args.patch_radius_mm)
    source_paths=[HERE/'run_full_arm_grasp.py',HERE/'full_arm_probe.cpp',HERE/'grasp_probe.cpp',folder/'scene.world',BUILD/'bin/px4',
                  MODELS/'so101/so101.sdf',MODELS/'yhang550/yhang550.sdf',MODELS/'yhang550_base/yhang550_base.sdf',MODELS/'test_cube/model.sdf',
                  Path('/home/pcz/super_ws/devel/lib/libso101_joint_position_controller.so'),BUILD/'build_gazebo-classic/libgazebo_mavlink_interface.so']
    manifest=dict(mode='calibration' if args.calibrate else ('retrieve' if args.retrieve else ('contact' if args.grasp else 'alignment')),pickup_base=PICKUP.tolist(),inspection_base=INSPECTION.tolist(),retract_base=RETRACT.tolist(),vehicle_fixed=True,object_attached=False,clock='simulation',seed=123,arrival_tolerance_m=args.arrival_mm*.001,retrieve_route=args.retrieve_route,hold_seconds=args.hold_seconds,lift_mm=args.lift_mm,side_clear_mm=args.side_clear_mm,motion_step_m=.003,solver_iterations=args.iterations,solver=args.solver,acceptance=args.acceptance,cube_mm=args.cube_mm,gripper_p_override=args.gripper_p,pad_torsional_radius_m=args.patch_radius_mm*.001,pad_torsional_coefficient=.8 if args.patch_radius_mm>0 else None,
                  sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in source_paths})
    (folder/'manifest.json').write_text(json.dumps(manifest,indent=2))
    server=px4=observer=gui=None
    result=dict(success=False,flight_validated=False)
    wall_start=time.monotonic();last_sim=-1;last_progress=wall_start
    events=[];status_log=[]
    hold_start=None
    files=[]
    def logfile(name):
        f=(folder/name).open('w');files.append(f);return f
    def client(module,*values):
        command=[str(BUILD/'bin'/f'px4-{module}'),*map(str,values)]
        proc=sp.run(command,env=env,stdout=sp.PIPE,stderr=sp.STDOUT,text=True,timeout=4)
        if proc.returncode: raise RuntimeError(f'{module} failed: {proc.stdout}')
        return proc.stdout
    def sample():
        nonlocal last_sim,last_progress
        if time.monotonic()-wall_start>600: raise RuntimeError('wall watchdog timeout')
        if server.poll() is not None or px4.poll() is not None: raise RuntimeError('simulation process exited')
        if observer is not None and observer.poll() is not None: raise RuntimeError('pose/contact observer exited')
        data=rows(folder/'poses.csv')
        r=data[-1] if data else None
        if r and r['sim_s']>last_sim:
            last_sim=r['sim_s'];last_progress=time.monotonic()
        if r and r['gripper_seen']:
            if abs(r['sim_s']-r['pose_s'])>.2: raise RuntimeError('Gazebo pose stale')
            if not all(math.isfinite(r[k]) for k in ['gx','gy','gz','gw','gqx','gqy','gqz']):
                raise RuntimeError('nonfinite gripper pose')
        if r and r['cube_seen'] and not all(math.isfinite(r[k]) for k in ['cx','cy','cz','cw','cqx','cqy','cqz']):
            raise RuntimeError('nonfinite object pose')
        if r and r.get('jaw_seen'):
            if not all(math.isfinite(r[k]) for k in ['jaw_angle','jaw_speed']) or r['jaw_receive_age_s']>.2:
                raise RuntimeError('gripper state nonfinite/stale')
        if time.monotonic()-last_progress>30: raise RuntimeError('simulation clock stalled')
        return r
    def event(name,row,**details):
        e=dict(event=name,sim_s=row['sim_s'],**details);events.append(e);print(json.dumps(e),flush=True)
    def status():
        text=client('listener','arm_control_status','1');values={}
        for line in text.splitlines():
            match=re.match(r'\s+(\w+):\s+(.+)',line)
            if not match: continue
            key,value=match.groups();value=value.strip()
            if value.startswith('['): values[key]=[float(x) for x in value.strip('[]').split(',')]
            elif value in ['True','False']: values[key]=value=='True'
            else:
                try: values[key]=float(value.split()[0])
                except ValueError: pass
        values['gazebo_sim_s']=last_sim;status_log.append(values)
        return values
    def healthy(s):
        if not s.get('feedback_valid') or s.get('feedback_age',math.inf)>.2:
            raise RuntimeError('PX4 feedback invalid/stale')
        for key in ['ee_measured','joint_target','joint_command']:
            if not all(math.isfinite(x) for x in s.get(key,[math.nan])): raise RuntimeError('nonfinite PX4 state')
    def move(point,jaw,label,max_step=None,validate=None):
        r=sample();start=r['sim_s'];last_send=-math.inf;sent_sequence=None;settled_since=None
        event(label+'_started',r,requested_base=point.tolist(),gripper_rad=jaw)
        while True:
            r=sample();s=status();healthy(s)
            if validate: validate(r)
            error=np.linalg.norm(np.asarray(s['ee_target'])-point)
            if sent_sequence is None or error>.0002:
                if r['sim_s']-last_send>=.5:
                    commanded=point
                    if max_step is not None:
                        delta=point-np.array(s['ee_target']);length=np.linalg.norm(delta)
                        commanded=np.array(s['ee_target'])+delta*min(1.,max_step/max(length,1e-9))
                    sent_sequence=s['target_sequence'];client('arm_control','target',*commanded,jaw);last_send=r['sim_s']
            elif s['target_sequence']>sent_sequence:
                qualified=(s['position_error']<=args.arrival_mm*.001 and s['max_joint_error']<=.08 and s['max_joint_speed']<=.05)
                if not qualified: settled_since=None
                elif settled_since is None: settled_since=r['sim_s']
                elif r['sim_s']-settled_since>=.5:
                    event(label+'_arrived',r,position_error_m=s['position_error'],joint_target=s['joint_target'],strict_px4_arrival=s['position_reached'])
                    return r
            if r['sim_s']-start>(60 if max_step is not None else 35): raise RuntimeError(label+' IK/arrival timeout')
            time.sleep(.08)
    def wait_sim(seconds,validate=None):
        first=sample();start=first['sim_s']
        while True:
            r=sample()
            if validate: validate(r)
            if r['sim_s']-start>=seconds: return r
            time.sleep(.05)
    def align_cube():
        r=sample();start=r['sim_s'];settled=0
        event('measured_alignment_started',r)
        for step in range(100):
            r=sample();s=status();healthy(s)
            if not r['cube_seen']: raise RuntimeError('cube pose unavailable')
            pos,R=gripper_pose(r)
            center=pos+R@finger_center
            error=np.array([r[k] for k in ['cx','cy','cz']])-center
            distance=float(np.linalg.norm(error))
            event('alignment_sample',r,step=step,error_m=distance,error_world=error.tolist(),accepted_base=s['ee_target'])
            if distance>.10: raise RuntimeError('cube left the reachable inspection region')
            if r['cube_other_fraction']>.2 or r['support_arm_fraction']>.2:
                raise RuntimeError('alignment obstructed by support/non-finger collision')
            normal_aligned=args.cube_mm>=20 or abs((R.T@error)[0])<=.0003
            if distance<=.003 and normal_aligned and s['max_joint_speed']<.05:
                settled+=1
                if settled>=3:
                    target=np.array(s['ee_target'])
                    (folder/'grasp_target.json').write_text(json.dumps(dict(target_base=target.tolist(),gripper_open=.6,gripper_close=-.15,center_error_m=distance,reference_gripper=finger_center.tolist(),cube_mm=args.cube_mm,sim_s=r['sim_s']),indent=2))
                    event('grasp_target_measured',r,target_base=target.tolist(),center_error_m=distance)
                    return target,r
                wait_sim(.6);continue
            settled=0
            # Invert the production upside-down mount. Correct the ACCEPTED
            # target rather than resetting to measured FK, so static tracking
            # bias can be compensated without modifying the firmware/PID.
            delta=error*np.array([1.,-1.,-1.])*.6
            norm=float(np.linalg.norm(delta))
            if norm>.003: delta*=.003/norm
            target=np.array(s['ee_target'])+delta
            sequence=s['target_sequence']
            client('arm_control','target',*target,.6)
            wait_sim(1.)
            after=status();healthy(after)
            if after['target_sequence']<=sequence:
                raise RuntimeError('measured micro-target rejected by IK/limits')
            if np.linalg.norm(np.array(after['ee_target'])-target)>.0002:
                raise RuntimeError('micro-target was altered by workspace limits')
            if sample()['sim_s']-start>120: raise RuntimeError('measured alignment sim-time limit')
        raise RuntimeError('measured alignment iteration limit')
    try:
        server=sp.Popen(['gzserver','--verbose','--seed','123',str(folder/'scene.world')],env=env,stdout=logfile('gazebo.log'),stderr=sp.STDOUT)
        px4=sp.Popen([str(BUILD/'bin/px4'),'-d',str(BUILD/'etc')],cwd=folder/'rootfs',env=env,stdout=logfile('px4.log'),stderr=sp.STDOUT)
        observer=sp.Popen([str(folder/'observer')],env=env,stdout=logfile('poses.csv'),stderr=logfile('observer.log'))
        if args.gui: gui=sp.Popen(['gzclient'],env=env,stdout=logfile('gui.log'),stderr=sp.STDOUT)
        ready=None
        while ready is None:
            r=sample()
            if r and r['sim_s']>3 and r['gripper_seen']:
                try:
                    s=status()
                    if s.get('command_initialized') and s.get('feedback_valid'): ready=r
                except RuntimeError: pass
            time.sleep(.1)
        event('feedback_ready',ready)
        move(INSPECTION,.6,'inspection')
        if args.calibrate:
            move(PICKUP,.6,'pickup')
        r=wait_sim(2)
        s=status();healthy(s)
        pos,R=gripper_pose(r)
        tool_world=pos+R@np.array([-.0079,-.000218121,-.0981274])
        measured=np.array(s['ee_measured'])
        fk_world=np.array([measured[0],-measured[1],1.15-measured[2]])
        frame_error=float(np.linalg.norm(tool_world-fk_world))
        event('world_frame_verified',r,error_m=frame_error)
        if frame_error>.003: raise RuntimeError('Gazebo/PX4 world-frame mismatch')
        if args.calibrate:
            data=dict(pickup_base=PICKUP.tolist(),gripper_world_position=pos.tolist(),
                      gripper_world_quaternion=[r[k] for k in ['gw','gqx','gqy','gqz']],
                      cube_world_position=(pos+R@finger_center).tolist(),
                      reference_gripper=finger_center.tolist(),cube_mm=args.cube_mm,
                      model_sha256=hashlib.sha256((MODELS/'so101/so101.sdf').read_bytes()).hexdigest())
            (folder/'calibration.json').write_text(json.dumps(data,indent=2))
            result.update(success=True,execution_status='calibration_complete',calibration=data)
            event('calibrated',r,**data)
        else:
            pickup,r=align_cube()
            if args.align_only or not args.grasp:
                result.update(success=True,execution_status='alignment_complete',grasp_tested=False,target_base=pickup.tolist())
                return
            if not r['cube_seen']: raise RuntimeError('cube pose unavailable')
            pos,R=gripper_pose(r)
            relative=R.T@(np.array([r[k] for k in ['cx','cy','cz']])-pos)
            event('cube_alignment',r,cube_in_gripper=relative.tolist())
            if np.linalg.norm(relative-finger_center)>.012: raise RuntimeError('cube/finger alignment error exceeds 12 mm')
            move(pickup,-.15,'closing_command')
            close_start=sample()['sim_s']
            while True:
                r=wait_sim(.2);s=status();healthy(s)
                recent=[x for x in rows(folder/'poses.csv') if last_sim-1<=x['sim_s']<=last_sim]
                contact=contact_summary(recent)
                if (last_sim-close_start>=2 and contact['bilateral_fraction']>=.9
                        and abs(r['jaw_speed'])<.02 and r['jaw_receive_age_s']<.2
                        and min(contact['fixed_normal_N'],contact['moving_normal_N'])>.01):
                    break
                if last_sim-close_start>12: raise RuntimeError('jaw closure/bilateral-force confirmation timeout')
            reference=recent[-1];cube0=np.array([reference[k] for k in ['cx','cy','cz']]);local0=cube_local(reference)
            event('bilateral_contact',reference,contact=contact,cube_local=local0.tolist(),actual_jaw_angle=reference['jaw_angle'])
            if not args.retrieve:
                result.update(success=True,execution_status='bilateral_contact_verified',grasp_validated=False,support_removed=False,contact=contact)
                return
            lost_since=None
            def payload_guard(r):
                nonlocal lost_since
                if np.linalg.norm(cube_local(r)-local0)>.045: raise RuntimeError('gross payload separation')
                if r['cube_other_fraction']>.2 or r['support_arm_fraction']>.2:
                    raise RuntimeError('payload motion obstructed by non-finger/support collision')
                if r['bilateral_fraction']<.5:
                    if lost_since is None: lost_since=r['sim_s']
                    elif r['sim_s']-lost_since>.5: raise RuntimeError('sustained loss of bilateral contact')
                else: lost_since=None
            # Upward-facing fingers cannot lift vertically through their support.
            # Physically exit sideways first; never delete/move the support or bind the cube.
            cleared=pickup+np.array([0.,args.side_clear_mm*.001,0.])
            if args.side_clear_mm>0:
                move(cleared,-.15,'side_clear',max_step=.003,validate=payload_guard)
                outside=wait_sim(1,validate=payload_guard)
                if abs(outside['cy']-cube0[1])<.020 or outside['cube_support_fraction']>.01:
                    raise RuntimeError('payload did not physically exit its fixed support')
                # Actual body envelope, not cube center clearance. In the
                # upside-down mount, the wide body can still strike the table
                # from below even after both fingers/object have left it.
                g,R=gripper_pose(outside)
                _,body_max=box_bounds(g,R,body_pose[:3],body_half)
                support_R=rotation(calibration['gripper_world_quaternion'])
                down=1. if support_R[2,2]<0 else -1.
                support_center=np.array(calibration['cube_world_position'])+down*support_R[:,2]*(args.cube_mm*.0005+.002)
                support_min,_=box_bounds(support_center,support_R,[0,0,0],[.006,.007,.002])
                clearance=support_min[1]-body_max[1]
                event('support_exited',outside,world_displacement_y=outside['cy']-cube0[1],body_clearance_y_m=float(clearance))
                if clearance<.003:
                    raise RuntimeError('gripper body not 3 mm clear of support before lift; increase side-clear-mm')
            lifted=cleared+np.array([0.,0.,-args.lift_mm*.001])
            if lifted[2]<.02: raise RuntimeError('lift would cross the configured workspace bound')
            move(lifted,-.15,'lift',max_step=.003,validate=payload_guard)
            if args.retrieve_route=='stow':
                # Keep lateral clearance until the arm has retracted past the
                # table. A direct diagonal return sweeps the gripper body
                # through the support, even though the object is above it.
                stow_clear=np.array([RETRACT[0],cleared[1],lifted[2]])
                move(stow_clear,-.15,'stow_clear',max_step=.003,validate=payload_guard)
                move(RETRACT,-.15,'retract',max_step=.003,validate=payload_guard)
            settled=wait_sim(2,validate=payload_guard)
            local_settled=cube_local(settled)
            lift=float(settled['cz']-cube0[2])
            if lift<.025 or settled['cube_support_fraction']>.01:
                raise RuntimeError('object not lifted at least 25 mm clear of its support')
            event('unsupported_hold_started',settled,cube_lift_m=lift,
                  settling_shift_m=float(np.linalg.norm(local_settled-local0)),hold_seconds=args.hold_seconds)
            hold_start=settled['sim_s'];last_health=hold_start
            def hold_guard(r):
                nonlocal last_health
                payload_guard(r)
                if r['cube_support_fraction']>.01: raise RuntimeError('object re-contacted its support')
                if args.acceptance=='strict' and np.linalg.norm(cube_local(r)-local_settled)>.010:
                    raise RuntimeError('payload drift exceeds 10 mm during unsupported hold')
                if r['sim_s']-last_health>=1:
                    healthy(status());last_health=r['sim_s']
            end=wait_sim(args.hold_seconds,validate=hold_guard)
            held=[r for r in rows(folder/'poses.csv') if hold_start<=r['sim_s']<=end['sim_s']]
            contact=contact_summary(held)
            late_drift=float(np.linalg.norm(cube_local(end)-local_settled))
            max_drift=max(float(np.linalg.norm(cube_local(r)-local_settled)) for r in held)
            min_lift=float(min(r['cz']-cube0[2] for r in held))
            retained,strict_retention=retention_checks(contact,late_drift,max_drift,min_lift)
            success=strict_retention if args.acceptance=='strict' else retained
            result.update(success=success,execution_status='completed',grasp_validated=success,
                          acceptance=args.acceptance,strict_retention_passed=strict_retention,
                          hold_sim_seconds=end['sim_s']-hold_start,cube_lift_m=float(end['cz']-cube0[2]),
                          min_hold_lift_m=min_lift,settling_shift_m=float(np.linalg.norm(local_settled-local0)),
                          late_relative_drift_m=late_drift,max_hold_drift_m=max_drift,contact=contact)
            event('retention_complete',end,**result)
    except (RuntimeError,sp.SubprocessError) as exc:
        result.update(execution_status='aborted',reason=str(exc));print(str(exc),flush=True)
    finally:
        if px4 is not None and px4.poll() is None:
            try: client('shutdown')
            except (RuntimeError,sp.SubprocessError): pass
        for proc in [px4,observer,gui,server]: stop(proc)
        for f in files: f.close()
        result.update(events=events,elapsed_wall_s=time.monotonic()-wall_start)
        telemetry=rows(folder/'poses.csv')
        if telemetry:
            result['telemetry_sim_time_monotonic']=all(b['sim_s']>a['sim_s'] for a,b in zip(telemetry,telemetry[1:]))
            result['observed_object_finite']=all(all(math.isfinite(r[k]) for k in ['cx','cy','cz','cw','cqx','cqy','cqz']) for r in telemetry if r['cube_seen'])
            result['max_nonfinger_contact_fraction']=max(r['cube_other_fraction'] for r in telemetry)
            result['max_support_arm_contact_fraction']=max(r['support_arm_fraction'] for r in telemetry)
            aligned=[e for e in events if e['event']=='grasp_target_measured']
            if aligned: result['grasp_reference_error_m']=aligned[-1]['center_error_m']
            if hold_start is not None:
                held=[r for r in telemetry if r['sim_s']>=hold_start]
                if held:
                    result['observed_hold_sim_seconds']=held[-1]['sim_s']-hold_start
                    result['max_hold_drift_m']=max(float(np.linalg.norm(cube_local(r)-local_settled)) for r in held)
                    result['last_relative_drift_m']=float(np.linalg.norm(cube_local(held[-1])-local_settled))
                    result['min_hold_lift_m']=float(min(r['cz']-cube0[2] for r in held))
                    result['hold_contact']=contact_summary(held)
        (folder/'summary.json').write_text(json.dumps(result,indent=2))
        (folder/'px4_status.json').write_text(json.dumps(status_log,indent=2))
        print(json.dumps(result),flush=True)
    if not result['success']: raise SystemExit(1)

if __name__=='__main__': main()
