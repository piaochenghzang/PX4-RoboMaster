"""Shared SITL scene and process helpers; importing never starts a demo.

Generated worlds retain truth plugins for legacy regression and offline audit.
Each explicit demo entry point selects and verifies its own control sources.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess as sp
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[5]
BUILD=ROOT/'build/px4_sitl_default'
WS=Path('/home/pcz/super_ws')
MODELS=ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models'
DEMO_OBSTACLES=(
    dict(name='demo_obstacle_wall',center=(1.5,0.,1.3),size=(.3,1.6,2.6),color=(.8,.15,.15,1.)),
    dict(name='demo_obstacle_upper',center=(3.4,2.6,1.3),size=(.7,.7,2.6),color=(.15,.35,.8,1.)),
    dict(name='demo_obstacle_lower',center=(3.4,-2.6,1.3),size=(.7,.7,2.6),color=(.9,.55,.1,1.)))
LIDAR_DEMO_OBSTACLES=(
    dict(name='demo_obstacle_wall',center=(.55,.90,1.3),size=(.12,.30,2.6),color=(.8,.15,.15,1.)),
    dict(name='demo_obstacle_upper',center=(2.6,1.7,1.3),size=(.5,.5,2.6),color=(.15,.35,.8,1.)),
    dict(name='demo_obstacle_lower',center=(2.6,-1.7,1.3),size=(.5,.5,2.6),color=(.9,.55,.1,1.)))
# Room surfaces provide nonparallel planes for LIO, outside the flight envelope.
LIO_ROOM=tuple(dict(name='demo_obstacle_room_'+str(i),center=c,size=z,color=(.6,.6,.6,1.))
    for i,(c,z) in enumerate([((-3.,0.,2.),(.1,8.,4.)),((6.,0.,2.),(.1,8.,4.)),
        ((1.5,-4.,2.),(9.,.1,4.)),((1.5,4.,2.),(9.,.1,4.)),((1.5,0.,4.5),(9.,8.,.1))]))
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

sys.path.insert(0, str(ROOT/'src/modules/arm_control/tests/grasp'))
from sim_model import remove_lidar, set_pad_patch


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


def make_world(folder,target,env,approach_guard=False,fixed_grasp=False,sim_vision=False,super_obstacle=False,lidar_geometry=None,fastlio=False,pillar_course=False):
    expanded=sp.check_output(['gz','sdf','-p',str(MODELS/'yhang550/yhang550.sdf')],
        env=env,text=True,timeout=30)
    model=ET.fromstring(expanded).find('model')
    if not super_obstacle or lidar_geometry is not None:remove_lidar(model)
    p=model.find('pose')
    if p is None: p=ET.SubElement(model,'pose')
    p.text='0 0 0 0 0 0' # Load sensor origins at zero; raise the paused vehicle before physics.
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
        if pillar_course:
            from pillar_course import obstacles
            scene_obstacles=obstacles()
        else:scene_obstacles=LIDAR_DEMO_OBSTACLES if lidar_geometry is not None else DEMO_OBSTACLES
        for description in scene_obstacles:add_obstacle(world,description)
        gui=world.find('gui')
        if gui is None:gui=ET.SubElement(world,'gui',fullscreen='false')
        camera=ET.SubElement(gui,'camera',name='demo_camera')
        ET.SubElement(camera,'pose').text='5.5 -10 8 0 0.65 1.57' if pillar_course else '3 -8 5 0 0.4 1.57'
        ET.SubElement(camera,'view_controller').text='orbit'
    plugin=ET.SubElement(world,'plugin',name='coordinate_truth',filename='libso101_coordinate_truth.so')
    ET.SubElement(plugin,'vehicleName').text='yhang550'; ET.SubElement(plugin,'targetName').text=target.name
    if approach_guard: ET.SubElement(plugin,'approachGuard').text='true'
    if fixed_grasp: ET.SubElement(plugin,'fixedGraspGuard').text='true'
    if lidar_geometry is not None:
        from lidar_grasp_demo import adapt_world
        physics_report=adapt_world(world,model,target,lidar_geometry)
        physics_report['moving_imu_is_gravity_cue_not_lio']=not fastlio
        save(folder/'payload_physics.json',physics_report)
    if fastlio:
        ET.SubElement(world.find("plugin[@name='demo_lidar']"),'demoLioImu').text='true'
        if pillar_course:
            from pillar_course import room
            scene_room=room()
        else:scene_room=LIO_ROOM
        for description in scene_room:add_obstacle(world,description)
    tree.write(folder/'scene.world',encoding='unicode',xml_declaration=True)
