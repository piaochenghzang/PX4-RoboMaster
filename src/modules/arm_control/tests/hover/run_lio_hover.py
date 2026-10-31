#!/usr/bin/env python3
"""LiDAR/LIO flight and native grasp demo; truth is scored only OFFLINE."""
import argparse,json,math,os,signal,subprocess as sp,time,threading,xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
import numpy as np
import rospy,rosgraph,yaml
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,PositionTarget
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from super_px4_bridge.msg import KnownFlightSetpoint,KnownFlightStatus
from known_target import rotation,quintic_segment,pose
from lidar_grasp_demo import LidarDemoPlan,TargetObservation
from lio_tool_observer import LioToolObserver
from native_client import NativeClient
from sim_scene import ROOT,BUILD,WS,MODELS,make_world,save,stop,SIM_VISION_PARAMS,write_lio_launch

PARAMS=dict(SIM_VISION_PARAMS,EKF2_EV_CTRL=11,EKF2_MAG_TYPE=5,EKF2_EVP_NOISE=.05,
    EKF2_EVA_NOISE=.03,EKF2_DELAY_MAX=150,EKF2_TAU_VEL=.05,
    # Measured collective in the previous loaded-vehicle hover; experiment only.
    MPC_THR_HOVER=.84)
LIO_VELOCITY_STD=.010  # PX4 EV minimum; ideal simulation, not hardware calibration.
NAV_TRACKING_BUDGET=.09  # Body tracking + measured folded-arm corner deviation.


def score_recording(folder,begin,end,goal):
    # This function runs only AFTER every owned simulator/control process stops.
    import rosbag
    from grasp_audit.geometry import interpolate_pose
    truth=[];targets=[];grippers=[];estimated=[];alignment=None
    with rosbag.Bag(str(folder/'audit.bag')) as bag:
        for topic,m,_ in bag.read_messages():
            if topic=='/grasp_audit/truth':
                i=list(m.link_names).index('yhang550_base::base_link');p=m.link_poses[i]
                truth.append((m.header.stamp.to_sec(),[p.position.x,p.position.y,p.position.z,
                    p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]))
                i=list(m.link_names).index('so101::gripper_link');p=m.link_poses[i]
                grippers.append((m.header.stamp.to_sec(),[p.position.x,p.position.y,p.position.z,
                    p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]))
                if m.target_present:
                    p=m.target_pose
                    targets.append((m.header.stamp.to_sec(),[p.position.x,p.position.y,p.position.z,
                        p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]))
            elif topic=='/lidar_slam/odom':estimated.append(m)
            elif topic=='/fastlio_sim/alignment':alignment=json.loads(m.data)
    if not alignment or alignment['source']!='FIRST_LIO_BODY_POSE':raise RuntimeError('Unexpected LIO origin source')
    stamp=alignment['calibration_stamp_s'];physical,_=interpolate_pose(truth,stamp,.03)
    first=min(estimated,key=lambda m:abs(m.header.stamp.to_sec()-stamp))
    if abs(first.header.stamp.to_sec()-stamp)>.005:raise RuntimeError('Initial LIO pose missing from recording')
    p=first.pose.pose;q=p.orientation;R=rotation([q.w,q.x,q.y,q.z])
    yaw=math.atan2(physical[1,0],physical[0,0])-math.atan2(R[1,0],R[0,0])
    c,s=math.cos(yaw),math.sin(yaw);W_M=np.eye(4);W_M[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]]
    W_M[:3,3]=physical[:3,3]-W_M[:3,:3]@np.array([p.position.x,p.position.y,p.position.z])
    errors=[];hold=[]
    for m in estimated:
        t=m.header.stamp.to_sec()
        if not begin<=t<=end:continue
        try:actual,_=interpolate_pose(truth,t,.03)
        except ValueError:continue
        p=m.pose.pose.position;model=(W_M@np.r_[[p.x,p.y,p.z],1])[:3]
        errors.append(float(np.linalg.norm(actual[:3,3]-model)))
        if t>=end-5:hold.append(float(np.linalg.norm(actual[:3,3]-(W_M@np.r_[goal,1])[:3])))
    if len(errors)<50 or len(hold)<30:raise RuntimeError('Offline scoring coverage insufficient')
    result=dict(independent_truth_used_after_shutdown_only=True,position_error_mean_m=float(np.mean(errors)),
        position_error_max_m=max(errors),physical_final_goal_mean_error_m=float(np.mean(hold)),
        physical_final_goal_max_error_m=max(hold),samples=len(errors),alignment_source=alignment['source'],
        passed=max(hold)<.12)
    observation=folder/'perceived_target_map.json'
    if observation.exists():
        detected=json.loads(observation.read_text())
        source=detected['source_observation']
        if detected['frame']!='map' or source['localization']!='FAST_LIO_BODY_MAP_POSE':
            raise RuntimeError('Unexpected target coordinate source')
        t=source['observation_end_ns']*1e-9
        actual,_=interpolate_pose(targets,t,.03)
        predicted=W_M@np.array(detected['map_from_object'])
        translation=float(np.linalg.norm(predicted[:3,3]-actual[:3,3]))
        angle=math.degrees(math.acos(float(np.clip((np.trace(predicted[:3,:3].T@actual[:3,:3])-1)/2,-1,1))))
        body_actual,_=interpolate_pose(truth,t,.03)
        body_estimate=np.array(source['body_map_at_reference'])
        relative_estimate=np.linalg.inv(body_estimate)@np.array(detected['map_from_object'])
        relative_actual=np.linalg.inv(body_actual)@actual
        relative_error=float(np.linalg.norm(relative_estimate[:3,3]-relative_actual[:3,3]))
        target_score=dict(position_error_m=translation,orientation_error_deg=angle,
            body_relative_position_error_m=relative_error,
            # First LIO target initialization check, not a fine-grasp permission.
            passed=translation<.02 and relative_error<.02 and angle<5.,scoring_only_after_shutdown=True)
        save(folder/'target_offline_score.json',target_score)
        result['target']=target_score;result['passed']=result['passed'] and target_score['passed']
    tool_file=folder/'tool_relative.jsonl'
    if tool_file.exists():
        from grasp_audit.geometry import inverse,rpy_transform,residual
        config=json.loads((folder/'tool_geometry_used.json').read_text())
        ref=np.eye(4);ref[:3,3]=config['reference_gripper']
        tool=rpy_transform(config['tool_pose']);rows=[json.loads(line) for line in tool_file.read_text().splitlines()]
        tcp_errors=[];reference_errors=[];relative_errors=[];fk_errors=[]
        for row in rows:
            t=row['stamp_s'];body_actual,_=interpolate_pose(truth,t,.03)
            gripper_actual,_=interpolate_pose(grippers,t,.03);object_actual,_=interpolate_pose(targets,t,.03)
            tcp_errors.append(residual(gripper_actual@tool,W_M@np.array(row['map_from_tcp'])))
            reference_errors.append(residual(gripper_actual@ref,W_M@np.array(row['map_from_grasp_reference'])))
            relative_errors.append(residual(inverse(gripper_actual@ref)@object_actual,np.array(row['grasp_reference_from_target'])))
            fk_errors.append(residual(inverse(body_actual)@gripper_actual,
                inverse(np.array(row['map_from_body']))@np.array(row['map_from_gripper'])))
        if len(rows)<20 or rows[-1]['stamp_s']-rows[0]['stamp_s']<2.5:raise RuntimeError('Insufficient FK observation coverage')
        if any(row['map_from_target']!=rows[0]['map_from_target'] for row in rows):raise RuntimeError('Locked target changed')
        maxima=lambda values:dict(position_m=max(v[0] for v in values),rotation_deg=math.degrees(max(v[1] for v in values)))
        fk_score=dict(samples=len(rows),duration_s=rows[-1]['stamp_s']-rows[0]['stamp_s'],
            tcp_max_error=maxima(tcp_errors),grasp_reference_max_error=maxima(reference_errors),
            reference_to_target_max_error=maxima(relative_errors),body_relative_fk_max_error=maxima(fk_errors),
            max_pair_age_s=max(row['source_age_s'] for row in rows),target_unchanged=True,
            joint_time_basis='ROS_SIM_CLOCK_APPROXIMATE_PHYSICS_SAMPLE',scoring_only_after_shutdown=True,
            # Coordinate observation acceptance only; this does not enable motion.
            passed=max(v[0] for v in tcp_errors+reference_errors+relative_errors)<.02 and
                max(v[1] for v in tcp_errors+relative_errors)<math.radians(5) and
                max(v[0] for v in fk_errors)<.002 and max(v[1] for v in fk_errors)<.001)
        save(folder/'tool_offline_score.json',fk_score)
        result['tool']=fk_score;result['passed']=result['passed'] and fk_score['passed']
    return result



def score_payload_recording(folder,approach):
    """Offline physical acceptance; never used to advance robot actions."""
    import rosbag
    close=next(e['sim_s'] for e in approach['events'] if e['event']=='lio_controlled_close_requested')
    done=next(e['sim_s'] for e in approach['events'] if e['event']=='contact_hold_and_lio_fk_extraction_completed')
    baseline=[];held=[]
    with rosbag.Bag(str(folder/'audit.bag')) as bag:
        for _,m,_ in bag.read_messages(topics=['/grasp_audit/truth']):
            t=m.header.stamp.to_sec()
            if not m.target_present:continue
            z=m.target_pose.position.z
            if close-.5<=t<=close:baseline.append(z)
            if done-4.5<=t<=done:held.append((t,z,bool(m.fixed_grasp_forbidden_contact)))
    if len(baseline)<10 or len(held)<100:raise RuntimeError('Offline payload scoring coverage insufficient')
    initial=float(np.median(baseline));minimum=min(z-initial for _,z,_ in held)
    result=dict(scoring_only_after_shutdown=True,ground_truth_control_inputs=False,
        actual_payload_lift_min_m=minimum,held_duration_s=held[-1][0]-held[0][0],
        forbidden_contact=any(x[2] for x in held),samples=len(held),
        passed=minimum>=.025 and held[-1][0]-held[0][0]>=4.3 and not any(x[2] for x in held))
    save(folder/'grasp_offline_score.json',result)
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('name');ap.add_argument('--gui',action='store_true');ap.add_argument('--observe-target',action='store_true',help='LIO-paired target and FK observation');ap.add_argument('--inspect-ik',action='store_true',help='Read-only native IK and local clearance checks after FK observation');ap.add_argument('--navigate-candidate',action='store_true',help='Fly folded to the LIO-derived candidate');ap.add_argument('--approach-lio',action='store_true',help='Includes observation, IK, navigation and velocity fusion; LIO/FK unfold and 40mm preapproach, never close')
    ap.add_argument('--grasp-lio',action='store_true',help='Same LIO route, then native contact-only close, 40mm tool extraction and hold')
    ap.add_argument('--fuse-lio-velocity',action='store_true',help='Test source-midpoint LIO pose+velocity through standard MAVROS odometry')
    args=ap.parse_args()
    # A later stage includes its prerequisites. One LIO approach command
    # selects the validated pose+velocity transport and all read-only checks.
    if args.grasp_lio:args.approach_lio=True
    if args.fuse_lio_velocity and not args.approach_lio:ap.error('--fuse-lio-velocity requires --approach-lio')
    if args.approach_lio:args.navigate_candidate=args.fuse_lio_velocity=True
    if args.navigate_candidate:args.inspect_ik=True
    if args.inspect_ik:args.observe_target=True
    if not args.name.replace('_','').isalnum():ap.error('Use a unique safe name')
    if any(sp.run(['pgrep','-x',n],stdout=sp.DEVNULL).returncode==0 for n in ('px4','gzserver','gzclient','rosmaster','mavros_node','fastlio_mapping')):
        ap.error('Existing simulation; refusing to disturb it')
    folder=ROOT/'build/lio_hover'/args.name;folder.mkdir(parents=True,exist_ok=False);(folder/'rootfs').mkdir()
    env=os.environ.copy();speed=float(env.get('PX4_SIM_SPEED_FACTOR','.35'))
    if not .25<=speed<=1:ap.error('Invalid simulator speed')
    env['PX4_SIM_SPEED_FACTOR']=str(speed)
    env.update(ROS_MASTER_URI='http://127.0.0.1:11311',ROS_HOSTNAME='localhost',GAZEBO_MASTER_URI='http://127.0.0.1:11345',
        GAZEBO_MODEL_PATH=str(MODELS),GAZEBO_MODEL_DATABASE_URI='',PX4_SIM_MODEL='gazebo-classic_yhang550',PX4_SYS_AUTOSTART='1020')
    libraries=str(WS/'devel/lib')+':'+str(BUILD/'build_gazebo-classic')+':/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins'
    env.update(GAZEBO_PLUGIN_PATH=libraries,LD_LIBRARY_PATH=libraries)
    prefix=ROOT/'build/fastlio_ws/devel'
    env['CMAKE_PREFIX_PATH']=str(prefix)+':'+env.get('CMAKE_PREFIX_PATH','')
    env['ROS_PACKAGE_PATH']=str(prefix.parent/'src')+':'+env.get('ROS_PACKAGE_PATH','')
    env['PYTHONPATH']=str(prefix/'lib/python3/dist-packages')+':'+env.get('PYTHONPATH','')
    parameters=dict(PARAMS)
    if args.fuse_lio_velocity:parameters.update(EKF2_EV_CTRL=15,EKF2_EVV_NOISE=LIO_VELOCITY_STD,EKF2_DELAY_MAX=250)
    env.update({'PX4_PARAM_'+k:str(v) for k,v in parameters.items()});env.pop('ROS_IP',None)
    plan=LidarDemoPlan(Path(__file__).with_name('known_target_lidar.json'))
    make_world(folder,plan.target,env,fixed_grasp=True,lidar_geometry=plan.geometry_profile,fastlio=True)
    scene=ET.parse(folder/'scene.world');model=scene.find("world/model[@name='yhang550']")
    model.find('pose').text='0 0 .45 0 0 0'
    bridge=model.find("plugin[@name='mavlink_interface']")
    for el in list(bridge):
        if el.tag in ('enableArmGraspFeedback','armGraspContactOnly','send_odometry','visionSubTopic'):bridge.remove(el)
    if args.approach_lio:
        ET.SubElement(bridge,'enableArmGraspFeedback').text='true'
        ET.SubElement(bridge,'armGraspContactOnly').text='true'
    if any(e.get('filename')=='libgazebo_vision_plugin.so' for e in model.findall('plugin')):raise RuntimeError('Ground-truth localization source present')
    scene.write(folder/'scene.world',encoding='unicode',xml_declaration=True)
    mavros=Path(sp.check_output(['rospack','find','mavros'],env=env,text=True).strip())/'launch'
    cfg=yaml.safe_load((mavros/'px4_config.yaml').read_text());cfg['time']['timesync_mode']='PASSTHROUGH';cfg['conn']['timesync_rate']=0.
    if args.fuse_lio_velocity:cfg['odometry']['fcu']['odom_parent_id_des']='map'
    lio_launch=WS/'src/super_px4_bridge/launch/fastlio_sim.launch'
    if args.approach_lio:
        write_lio_launch(lio_launch,folder/'fastlio.launch');lio_launch=folder/'fastlio.launch'
    save(folder/'mavros_config.json',cfg);launch=ET.parse(mavros/'px4.launch')
    launch.find("include/arg[@name='config_yaml']").set('value',str(folder/'mavros_config.json'));launch.write(folder/'mavros.launch')
    save(folder/'manifest.json',dict(stage='LIO_CONTACT_FIXED_GRASP' if args.grasp_lio else 'LIO_FK_NO_CLOSE_APPROACH' if args.approach_lio else 'LIO_CANDIDATE_NAVIGATION_NO_ARM_MOTION' if args.navigate_candidate else 'LIO_TARGET_OBSERVATION_NO_ARM_MOTION' if args.observe_target else 'LIO_LOCAL_FLIGHT_NO_ARM_MOTION',ground_truth_control_inputs=False,
        gazebo_vision_plugin=False,lio_alignment_source='FIRST_LIO_BODY_POSE',ekf_parameters=parameters,
        target_observation=args.observe_target,encoder_fk_observation=args.observe_target,read_only_ik_inspection=args.inspect_ik,candidate_navigation=args.navigate_candidate,lio_fk_approach=args.approach_lio,closing_allowed=args.grasp_lio,grasp_feedback_source='SIM_CONTACT_NO_OBJECT_POSE' if args.approach_lio else 'DISABLED',external_velocity_fused=args.fuse_lio_velocity,
        external_odometry_source='LIO_TWO_POSE_SOURCE_MIDPOINT' if args.fuse_lio_velocity else 'LIO_SCAN_END_POSE_ONLY',
        external_odometry_stddev=dict(position_m=.05,orientation_rad=.03,velocity_m_s=LIO_VELOCITY_STD,angular_velocity_rad_s=.05) if args.fuse_lio_velocity else None,
        registered_cloud_source='UNDISTORTED_POINTS' if args.approach_lio else 'ESTIMATOR_VOXEL_POINTS',
        observation_local_goal_m=[.3,0,.9],sim_speed_factor=speed))
    os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'],ROS_HOSTNAME='localhost');os.environ.pop('ROS_IP',None)
    procs=[];logs=[];subs=[];pub=None;fcu_pub=None;latest={};result=dict(success=False);begin=end=None;observation=None
    external_pairs=deque(maxlen=2);external_stamps=deque(maxlen=20);external_count=0;external_max_age=0.
    tool_observer=None;tool_log=None;tool_start=None;tool_count=0
    inspection_thread=None;inspection={}
    clouds=deque(maxlen=10);candidate_route=None;candidate_held=None;candidate_baseline=None
    candidate_samples=[];last_tool_row=None
    approach_probe=None;fine_bias=np.zeros(3);fine_velocity=np.zeros(3);last_bias=None
    def odom_pose(message):
        p=message.pose.pose;q=p.orientation
        return pose([p.position.x,p.position.y,p.position.z],[q.w,q.x,q.y,q.z])
    def check_online_graph():
        graph=rosgraph.Master('/lio_hover_graph').getSystemState()
        if any(node!='/lio_hover_audit_recorder' for topic,nodes in graph[1] if topic=='/grasp_audit/truth' for node in nodes):
            raise RuntimeError('Truth entered observation/control')
        for topic in ('/lidar_slam/odom','/cloud_registered'):
            if [nodes for name,nodes in graph[0] if name==topic]!=[['/fastlio_sim_adapter']]:
                raise RuntimeError('Unexpected LIO data publisher: '+topic)
        check_external_graph(graph)
        return dict(publishers=graph[0],subscribers=graph[1])
    def check_external_graph(graph):
        topic='/mavros/odometry/out' if args.fuse_lio_velocity else '/mavros/vision_pose/pose_cov'
        node=rospy.get_name() if args.fuse_lio_velocity else '/fastlio_sim_adapter'
        if [nodes for name,nodes in graph[0] if name==topic]!=[[node]]:raise RuntimeError('Unexpected FCU odometry publisher')
        other='/mavros/vision_pose/pose_cov' if args.fuse_lio_velocity else '/mavros/odometry/out'
        if any(nodes for name,nodes in graph[0] if name==other):raise RuntimeError('Duplicate FCU external input')
    def receive_lio(message):
        nonlocal external_count,external_max_age
        latest['lio']=message
        if fcu_pub is None:return
        try:
            from lio_control_sample import midpoint_body_odometry
            from tf.transformations import quaternion_from_matrix
            if message.header.frame_id!='map' or message.child_frame_id!='base_link':raise ValueError('EXTERNAL_FRAME')
            p=message.pose.pose;q=p.orientation;t=message.header.stamp.to_sec()
            external_pairs.append((t,[p.position.x,p.position.y,p.position.z,q.w,q.x,q.y,q.z]))
            if len(external_pairs)<2:return
            row=midpoint_body_odometry(external_pairs);age=rospy.Time.now().to_sec()-row['stamp_s']
            if not -.02<=age<=.25:raise ValueError('EXTERNAL_MIDPOINT_STALE')
            out=Odometry();out.header.frame_id='map';out.child_frame_id='base_link';out.header.stamp=rospy.Time.from_sec(row['stamp_s'])
            out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,row['body'][:3,3])
            out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,quaternion_from_matrix(row['body']))
            out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,row['linear_v_body'])
            out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,row['angular_v_body'])
            # Conservative experiment noise, not FAST-LIO's internal covariance.
            out.pose.covariance=np.diag([.05**2]*3+[.03**2]*3).ravel().tolist()
            out.twist.covariance=np.diag([LIO_VELOCITY_STD**2]*3+[.05**2]*3).ravel().tolist()
            fcu_pub.publish(out);external_stamps.append(row['stamp_s']);external_count+=1;external_max_age=max(external_max_age,age)
        except (ValueError,TypeError) as exc:latest['external_error']=str(exc)
    def start(name,cmd,cwd=None):
        log=(folder/(name+'.log')).open('w');logs.append(log)
        p=sp.Popen(cmd,env=env,cwd=cwd,stdout=log,stderr=sp.STDOUT,start_new_session=True);procs.append(p);return p
    started=time.monotonic()
    def health():
        if latest.get('external_error'):raise RuntimeError('LIO external odometry: '+latest['external_error'])
        if any(p.poll() is not None for p in procs):raise RuntimeError('Own process exited')
        if time.monotonic()-started>(500 if args.grasp_lio else 400 if args.approach_lio else 200)/speed:raise RuntimeError('Wall watchdog')
    try:
        core=start('roscore',['roscore']);deadline=time.monotonic()+20
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline:raise RuntimeError('ROS unavailable')
            time.sleep(.1)
        sp.run(['rosparam','set','/use_sim_time','true'],env=env,check=True)
        rospy.init_node('lio_hover_probe',disable_signals=True)
        if args.fuse_lio_velocity:fcu_pub=rospy.Publisher('/mavros/odometry/out',Odometry,queue_size=5)
        if args.observe_target:
            from grasp_audit.geometry import rpy_transform
            from check_installation import transform
            config=yaml.safe_load((WS/'src/super_px4_bridge/config/grasp_frame_audit.yaml').read_text())
            if not np.allclose(rpy_transform(config['mount_pose']),transform(plan.geometry_profile['vehicle']['body_from_arm_xyz_rpy_deg']),atol=1e-6):
                raise RuntimeError('Arm mounting configurations disagree')
            config={k:config[k] for k in ('joint_names','mount_pose','joint_origins','tool_pose')}
            config['reference_gripper']=plan.geometry_profile['vehicle']['grasp_reference_gripper_m']
            save(folder/'tool_geometry_used.json',config)
            tool_observer=LioToolObserver(config,config['reference_gripper'])
            tool_log=(folder/'tool_relative.jsonl').open('w')
        for topic,typ,key in [('/mavros/state',State,'state'),('/mavros/local_position/odom',Odometry,'odom'),
            ('/grasp_demo/flight_state',String,'manager'),
            ('/grasp_demo/known_status',KnownFlightStatus,'known')]:
            subs.append(rospy.Subscriber(topic,typ,lambda m,k=key:latest.__setitem__(k,m),queue_size=50,tcp_nodelay=True))
        subs.append(rospy.Subscriber('/lidar_slam/odom',Odometry,receive_lio,queue_size=50,tcp_nodelay=True))
        if args.navigate_candidate:
            subs.append(rospy.Subscriber('/cloud_registered',PointCloud2,clouds.append,queue_size=5,tcp_nodelay=True))
        start('audit',['rosbag','record','--lz4','-O',str(folder/'audit.bag'),'/grasp_audit/truth','/lidar_slam/odom',
            '/clock','/mavros/local_position/odom','/mavros/odometry/out','/mavros/state','/so101/joint_states','/fastlio_sim/alignment','/grasp_demo/known_setpoint','__name:=lio_hover_audit_recorder'])
        start('gazebo',['gzserver','--verbose','--seed','123','-s','libgazebo_ros_api_plugin.so',str(folder/'scene.world')])
        if args.gui:start('gui',['gzclient'])
        px4=start('px4',[str(BUILD/'bin/px4'),'-d',str(BUILD/'etc')],folder/'rootfs')
        start('mavros',['roslaunch',str(folder/'mavros.launch'),'fcu_url:=udp://:14540@127.0.0.1:14580'])
        while rospy.Time.now().to_sec()<5:health();time.sleep(.05)
        start('fastlio',['roslaunch',str(lio_launch),'alignment_source:=lio_origin','publish_to_fcu:='+str(not args.fuse_lio_velocity).lower()])
        native=NativeClient(BUILD,env,transport='unix',expected_pid=px4.pid)
        if args.fuse_lio_velocity:
            # PX4 needs 500 valid responses before honoring remote source
            # timestamps. Accelerate its own standard stream, not ROS clocks.
            sp.run([str(BUILD/'bin/px4-mavlink'),'stream','-u','14580','-s','TIMESYNC','-r','100'],env=env,check=True,timeout=4)
        sync_samples=set()
        while True:
            health();now=rospy.Time.now().to_sec();odom=latest.get('odom');lio=latest.get('lio')
            if now>40:raise RuntimeError('LIO/EKF startup timeout')
            if odom and lio and now-lio.header.stamp.to_sec()<.15 and now-odom.header.stamp.to_sec()<.15:
                flags=native.status('estimator_status_flags')
                if all(flags.get(k) for k in ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw')) and bool(flags.get('cs_ev_vel'))==args.fuse_lio_velocity and not any(flags.get(k) for k in ('cs_gnss_pos','cs_gnss_vel','cs_baro_hgt','cs_mag_hdg','cs_mag_3d')):
                    raw=native.status('vehicle_visual_odometry')
                    external={k:raw[k] for k in ('timestamp','timestamp_sample','pose_frame','position','q')}
                    if args.fuse_lio_velocity:
                        external.update({k:raw[k] for k in ('velocity_frame','velocity','velocity_variance')})
                        sync=native.status('timesync_status')
                        stamp=1e-6*(raw['timestamp_sample']-sync['estimated_offset'])
                        stamp_error=min((abs(stamp-t) for t in list(external_stamps)),default=math.inf)
                        age=1e-6*(raw['timestamp']-raw['timestamp_sample'])
                        # This PX4 MavlinkTimesync default-constructs Timesync
                        # with protocol UNKNOWN. Verify source times directly.
                        if not .04<=age<=.25 or stamp_error>.002:
                            sync_samples.clear();time.sleep(.1);continue
                        sync_samples.add(raw['timestamp_sample'])
                        if len(sync_samples)<3:time.sleep(.1);continue
                        external.update(source_stamp_error_s=stamp_error,source_age_s=age,timesync=sync,matched_distinct_source_samples=len(sync_samples))
                    save(folder/'ekf_preflight.json',dict(flags=flags,external_pose=external,external_velocity_fused=args.fuse_lio_velocity));break
            time.sleep(.1)
        graph=rosgraph.Master('/lio_hover_graph').getSystemState();save(folder/'graph_before_flight.json',dict(publishers=graph[0],subscribers=graph[1]))
        if any(node!='/lio_hover_audit_recorder' for topic,nodes in graph[1] if topic=='/grasp_audit/truth' for node in nodes):
            raise RuntimeError('Truth entered a control node')
        check_external_graph(graph)
        if args.approach_lio:
            if rospy.get_param('/publish/dense_publish_en',False) is not True:
                raise RuntimeError('Fine geometry requires undistorted raw registered points')
            save(folder/'lio_cloud_source.json',dict(source='UNDISTORTED_POINTS',dense_publish_en=True,
                filter_size_surf_m=rospy.get_param('/filter_size_surf'),filter_size_map_m=rospy.get_param('/filter_size_map'),
                estimator_voxel_parameters_unchanged=True))
            from check_fixed_target_native import evidence_json
            contact=native.status('arm_grasp_feedback')
            unknown=[contact.get(k) for k in ('object_height_world','body_support_clearance','vertical_force_n')]
            unknown+=contact.get('object_center_gripper',[])
            if (contact.get('source')!=2 or contact.get('profile_id')!=1 or contact.get('target_id')!=1
                    or not int(contact.get('capabilities',0))&1 or int(contact['capabilities'])&~3
                    or len(unknown)!=6 or not all(isinstance(x,float) and math.isnan(x) for x in unknown)):
                raise RuntimeError('Contact-only feedback contract failed')
            age=(1e6*rospy.Time.now().to_sec()-contact['timestamp_sample'])*1e-6
            if not -.02<=age<=.3:raise RuntimeError('Contact-only feedback stale')
            save(folder/'contact_preflight.json',evidence_json(contact))
            result['contact_only_feedback_verified']=True
        start('manager',['rosrun','super_px4_bridge','super_flight_manager_node','_known_goal_mode:=true','_auto_arm:=true',
            '_takeoff_height:=.9','_hover_stable_time:=2.0','__name:=lio_hover_manager'])
        pub=rospy.Publisher('/grasp_demo/known_setpoint',KnownFlightSetpoint,queue_size=1)
        route=None;origin=None;ready=None;held=None;last=-math.inf;goal=np.array([.3,0,.9]);begin=now
        with (folder/'telemetry.jsonl').open('w') as log:
            while True:
                health();now=rospy.Time.now().to_sec();odom=latest.get('odom');lio=latest.get('lio');state=latest.get('state');manager=latest.get('manager')
                if now>(400 if args.grasp_lio else 320 if args.approach_lio else 170 if args.navigate_candidate else 110):raise RuntimeError('Simulation-time flight deadline')
                if route is None and now-begin>30 and manager and manager.data!='HOVER_READY':
                    raise RuntimeError('Takeoff did not reach stable hover within 30 simulation seconds')
                if not (odom and lio and state and manager):time.sleep(.01);continue
                if not -.02<=now-lio.header.stamp.to_sec()<.2:raise RuntimeError('LIO input stale')
                if not -.02<=now-odom.header.stamp.to_sec()<.2:raise RuntimeError('EKF output stale')
                p=odom.pose.pose;q=p.orientation;R=rotation([q.w,q.x,q.y,q.z]);xyz=np.array([p.position.x,p.position.y,p.position.z])
                xmax,zmax=(2.,2.) if args.navigate_candidate else (1.2,1.5)
                if not np.isfinite(xyz).all() or abs(xyz[0])>xmax or abs(xyz[1])>.8 or not -.2<xyz[2]<zmax or R[2,2]<math.cos(math.radians(15)):raise RuntimeError('Estimated flight envelope exceeded')
                v=odom.twist.twist.linear;speed_now=np.linalg.norm([v.x,v.y,v.z])
                active=state.connected and state.armed and state.mode=='OFFBOARD' and -.02<=now-state.header.stamp.to_sec()<1.5
                if manager.data in ('HOVER_READY','TRACK_KNOWN') and not active:raise RuntimeError('Active OFFBOARD state lost')
                if manager.data=='HOVER_READY' and active and route is None:
                    if ready is None:ready=now
                    elif now-ready>=5:
                        route=now;origin=odom_pose(lio)[:3,3] if args.approach_lio else xyz.copy()
                        print('LIO hover accepted; 0.3m local movement starts',flush=True)
                flight_sample=None
                if args.approach_lio and route is not None:
                    from lio_control_sample import flight_control_sample
                    try:flight_sample=flight_control_sample(tool_observer,odom.header.stamp.to_sec(),now)
                    except ValueError as exc:
                        if str(exc) not in ('LIO_PAIR_GAP','LIO_NOT_BRACKETED'):raise
                    if flight_sample is None:
                        held=candidate_held=None
                        time.sleep(.002)
                        continue
                if route is not None and now-last>=.02:
                    if candidate_route is None:wp,wv=quintic_segment(origin,goal,now-route,6.);yaw=0.
                    else:wp,wv,yaw=candidate_route.sample(now)
                    cmd=KnownFlightSetpoint();cmd.header.stamp=rospy.Time.now();cmd.header.frame_id='map';cmd.task_id=1
                    if args.approach_lio:
                        from lio_approach_probe import alignment_bias,alignment_velocity
                        local_lio=flight_sample['body'][:3,3]
                        elapsed=.02 if last_bias is None else now-last_bias
                        fine_bias,_=alignment_bias(fine_bias,xyz,local_lio,elapsed);last_bias=now
                        ev=odom.twist.twist.linear
                        ekf_v=R@np.array([ev.x,ev.y,ev.z]);lio_v=flight_sample['linear_v_map']
                        # EV fusion improves the filter, but does not make its
                        # output identical to the measured LIO velocity. Apply
                        # only that residual so position and damping share the
                        # same reference; do not add position-bias derivative.
                        fine_velocity=alignment_velocity(fine_velocity,ekf_v,lio_v,elapsed)
                        # EKF-LIO velocity difference already supplies the
                        # changing position offset's feedforward. Adding its
                        # finite-difference derivative again doubles it.
                        wp=wp+fine_bias;wv=wv+fine_velocity
                    if approach_probe is not None:
                        cmd.task_id,cmd.plan_id,cmd.target_id,cmd.scene_id=approach_probe.ids;cmd.terminal_hold=True
                    cmd.setpoint.header=cmd.header;cmd.setpoint.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
                    cmd.setpoint.type_mask=PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE
                    cmd.setpoint.position.x,cmd.setpoint.position.y,cmd.setpoint.position.z=map(float,wp)
                    cmd.setpoint.velocity.x,cmd.setpoint.velocity.y,cmd.setpoint.velocity.z=map(float,wv);cmd.setpoint.yaw=float(yaw);pub.publish(cmd);last=now
                error=float(np.linalg.norm(xyz-goal))
                good=active and route is not None and now-route>=6 and error<.05 and speed_now<.08
                if args.approach_lio and flight_sample is not None:
                    from lio_approach_probe import filter_tracking_errors
                    filter_error,filter_speed=filter_tracking_errors(xyz,R@np.array([v.x,v.y,v.z]),goal,fine_bias,fine_velocity)
                    initial_lio_error=float(np.linalg.norm(flight_sample['body'][:3,3]-goal))
                    initial_lio_speed=float(np.linalg.norm(flight_sample['linear_v_map']))
                    good=(active and route is not None and now-route>=6 and filter_error<.05 and filter_speed<.08
                        and initial_lio_error<.05 and initial_lio_speed<.08)
                if not good:held=None
                elif held is None:held=now
                row=dict(sim_s=now,manager=manager.data,estimated_body=xyz.tolist(),goal_error_m=error,lio_age_s=now-lio.header.stamp.to_sec(),
                    observation_window=observation.last_window_check if observation else None,
                    lio_position_correction_map_m=fine_bias.tolist(),lio_velocity_correction_map_m_s=fine_velocity.tolist(),
                    flight_pair=None if flight_sample is None else dict(filter_stamp_s=flight_sample['stamp_s'],
                        lio_before_stamp_s=flight_sample['lio_before_stamp_s'],lio_after_stamp_s=flight_sample['lio_after_stamp_s'],
                        last_lio_stamp_s=flight_sample['last_lio_stamp_s'],prediction_s=flight_sample['prediction_s'],
                        mode=flight_sample['mode']))
                log.write(json.dumps(row)+'\n');log.flush()
                if held is not None and now-held>=8:
                    result.update(control_flight_completed=True,final_estimated_error_m=error)
                    if not args.observe_target:end=now;break
                    if observation is None:
                        observation=TargetObservation(plan.geometry_profile,folder,body_source='lio')
                        graph=rosgraph.Master('/lio_hover_graph').getSystemState()
                        save(folder/'graph_observation.json',dict(publishers=graph[0],subscribers=graph[1]))
                        if any(node!='/lio_hover_audit_recorder' for topic,nodes in graph[1] if topic=='/grasp_audit/truth' for node in nodes):
                            raise RuntimeError('Truth entered observation/control')
                        print('Stable local flight; observing raw cloud with LIO body poses',flush=True)
                if observation is not None and not result.get('target_observation_completed'):
                    v=lio.twist.twist.linear
                    estimate=observation.poll(good and np.linalg.norm([v.x,v.y,v.z])<.02)
                    if estimate is not None and not result.get('target_observation_completed'):
                        detected=dict(frame=estimate['frame'],map_from_object=estimate['transform'].tolist(),
                            source_observation=estimate['report']['source_observation'],estimate_path=str(estimate['path']/'estimate.json'))
                        save(folder/'perceived_target_map.json',detected)
                        tool_observer.lock_target(detected);result['target_observation_completed']=True
                        print('Target locked in LIO map; reading encoder FK for 3 seconds',flush=True)
                if result.get('target_observation_completed') and approach_probe is None:
                    row=tool_observer.sample(now)
                    if row is not None:
                        last_tool_row=row
                        if not result.get('fk_observation_completed'):
                            tool_log.write(json.dumps(row,allow_nan=False)+'\n');tool_log.flush();tool_count+=1
                        if tool_start is None:tool_start=row['stamp_s']
                        if row['stamp_s']-tool_start>=3. and tool_count>=20:
                            result['fk_observation_completed']=True
                            if not args.inspect_ik:end=now;break
                            if inspection_thread is None:
                                from lio_ik_inspection import inspect_native
                                frozen_row=json.loads(json.dumps(row,allow_nan=False))
                                def run_inspection():
                                    try:
                                        checked=inspect_native(native,frozen_row,config,plan.geometry_profile,folder/'ik_inspection')
                                        from candidate_hover import inspect_hover
                                        checked['candidate_hover']=inspect_hover(native,frozen_row,config,plan.geometry_profile,
                                            np.r_[plan.expected_q,frozen_row['joint_position_rad'][5]],folder/'candidate_hover')
                                        from unfold_precheck import inspect_unfold
                                        checked['unfold_precheck']=inspect_unfold(native,frozen_row,config,plan.geometry_profile,
                                            np.r_[plan.expected_q,frozen_row['joint_position_rad'][5]],folder/'unfold_precheck')
                                        if args.navigate_candidate:
                                            from candidate_hover import derive_hover
                                            from lio_candidate_route import CandidateRoute,check_frame_agreement
                                            from lio_route_clearance import body_collision_boxes,check_route
                                            from scan_metrics import cloud_array
                                            from check_fixed_target_native import snapshot
                                            if not checked['unfold_precheck']['native_unfold_branch_valid']:
                                                raise RuntimeError('Unfold branch did not pass')
                                            candidate=derive_hover(frozen_row['map_from_target'],config,
                                                np.r_[plan.expected_q,frozen_row['joint_position_rad'][5]])
                                            local=latest['odom'];mapped=latest['lio'];stamp=rospy.Time.now().to_sec()
                                            if (abs(local.header.stamp.to_sec()-mapped.header.stamp.to_sec())>.125 or
                                                max(stamp-local.header.stamp.to_sec(),stamp-mapped.header.stamp.to_sec())>.2):
                                                raise RuntimeError('Candidate navigation pose pairing is stale')
                                            agreement=check_frame_agreement(odom_pose(mapped),odom_pose(local))
                                            if not agreement['valid']:raise RuntimeError('LIO / EKF local origins disagree')
                                            start=odom_pose(local);desired=candidate['map_from_body_hover']
                                            preview=CandidateRoute(start[:3,3],desired[:3,3],
                                                math.atan2(start[1,0],start[0,0]),candidate['yaw_rad'],stamp)
                                            scans=list(clouds)
                                            if not scans or scans[-1].header.frame_id!='map' or not 0<=stamp-scans[-1].header.stamp.to_sec()<=.3:
                                                raise RuntimeError('Fresh registered map returns required')
                                            arrays=[]
                                            for cloud in scans:
                                                if cloud.header.frame_id!='map':raise RuntimeError('Registered cloud frame')
                                                a=cloud_array(cloud);arrays.append(np.column_stack([a[k] for k in ('x','y','z')]))
                                            points=np.concatenate(arrays)
                                            if len(points)<1000:raise RuntimeError('Insufficient observed map returns')
                                            if args.approach_lio:
                                                nearby=points[np.linalg.norm(points-desired[:3,3],axis=1)<.9]
                                                _,fine_indices=np.unique(np.floor(nearby/.002).astype(np.int64),axis=0,return_index=True)
                                                np.save(folder/'approach_map_returns.npy',nearby[fine_indices])
                                            # Fine approach retains 2mm cells: sqrt(3)*2mm
                                            # plus 4mm LIO allowance fits inside 8mm.
                                            cell=.002 if args.approach_lio else .02
                                            point_margin=.008 if args.approach_lio else .04
                                            _,indices=np.unique(np.floor(points/cell).astype(np.int64),axis=0,return_index=True)
                                            points=points[indices];np.save(folder/'candidate_map_returns.npy',points)
                                            baseline=snapshot(native)
                                            boxes=body_collision_boxes(folder/'scene.world',config,
                                                baseline['arm_control_status']['joint_target'])
                                            guard=check_route(preview,boxes,points,tracking_margin=NAV_TRACKING_BUDGET,point_margin=point_margin)
                                            save(folder/'candidate_navigation_precheck.json',dict(frame_agreement=agreement,
                                                observed_returns=guard,source='LIO_REGISTERED_CLOUD_PLUS_CONFIGURED_COLLISION_SHAPES',
                                                latest_cloud_stamp_s=scans[-1].header.stamp.to_sec(),candidate_body_map=desired.tolist(),
                                                route_start_map_m=preview.start.tolist(),start_yaw_rad=preview.start_yaw,goal_yaw_rad=candidate['yaw_rad']))
                                            if not guard['valid']:raise RuntimeError('Observed navigation corridor is obstructed')
                                            inspection.update(candidate=candidate,baseline=baseline,route=preview)
                                        inspection['result']=checked
                                    except Exception as e:inspection['error']=str(e)
                                inspection_thread=threading.Thread(target=run_inspection,daemon=True);inspection_thread.start()
                                print('Read-only native IK queries; flight hold lease continues',flush=True)
                if args.inspect_ik and inspection_thread is not None and not inspection_thread.is_alive():
                    if 'error' in inspection:raise RuntimeError('Read-only IK inspection: '+inspection['error'])
                    result['ik_inspection']=inspection['result']
                    if not args.navigate_candidate:end=now;break
                    from lio_candidate_route import CandidateRoute,check_frame_agreement
                    from check_fixed_target_native import snapshot,assert_unchanged
                    if candidate_route is None:
                        save(folder/'graph_candidate_navigation.json',check_online_graph())
                        desired=inspection['candidate']['map_from_body_hover'];goal=desired[:3,3].copy()
                        # Keep the exact prechecked curve; shift only its clock.
                        candidate_route=inspection['route'];candidate_route.began=now
                        candidate_baseline=inspection['baseline']
                        print('LIO-derived candidate accepted; folded vehicle climbs then translates',flush=True)
                    if abs(lio.header.stamp.to_sec()-odom.header.stamp.to_sec())>.125:
                        # An asynchronous source update is not a valid pair.
                        # Abstain from arrival/approach decisions until paired;
                        # the independent 200ms source lease above still aborts.
                        candidate_held=None
                        time.sleep(.002)
                        continue
                    agreement=check_frame_agreement(odom_pose(lio),odom_pose(odom))
                    if not agreement['valid']:raise RuntimeError('LIO / EKF frame disagreement during navigation')
                    lio_pose=odom_pose(lio);error_lio=float(np.linalg.norm(lio_pose[:3,3]-goal))
                    commanded,_,command_yaw=candidate_route.sample(now)
                    tracking_error=float(np.linalg.norm(lio_pose[:3,3]-commanded))
                    arm_deviation=None
                    if approach_probe is None:
                        if last_tool_row is None or not 0<=now-last_tool_row['stamp_s']<=.2:raise RuntimeError('Folded arm feedback stale')
                        joint_delta=np.abs(np.array(last_tool_row['joint_position_rad'])-
                            candidate_baseline['arm_control_status']['joint_target'])
                        arm_deviation=float(np.array([.51,.45,.33,.20,.13])@joint_delta[:5]+.13*joint_delta[5])
                    c,s=math.cos(command_yaw),math.sin(command_yaw)
                    desired_rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])
                    rotation_error=math.degrees(math.acos(float(np.clip((np.trace(desired_rotation.T@lio_pose[:3,:3])-1)/2,-1,1))))
                    reserve=arm_deviation if approach_probe is None else 0.
                    if tracking_error+reserve>NAV_TRACKING_BUDGET or rotation_error>5:raise RuntimeError('Candidate corridor tracking allowance exceeded')
                    if not clouds:raise RuntimeError('Registered navigation returns missing')
                    # Callbacks can append a newer cloud after this loop's
                    # cached `now`. Compare against the current ROS clock.
                    cloud_age=rospy.Time.now().to_sec()-clouds[-1].header.stamp.to_sec()
                    if -.002<=cloud_age<0:
                        candidate_held=None
                        time.sleep(.002)
                        continue
                    if not 0<=cloud_age<=.3:
                        raise RuntimeError('Registered navigation returns lost: age_s='+str(cloud_age))
                    yaw_lio=math.atan2(lio_pose[1,0],lio_pose[0,0]);yaw_error=abs(math.atan2(math.sin(yaw_lio-inspection['candidate']['yaw_rad']),math.cos(yaw_lio-inspection['candidate']['yaw_rad'])))
                    filter_error=float(np.linalg.norm(xyz-goal));filter_speed=float(speed_now)
                    lv=lio.twist.twist.linear;lio_speed=float(np.linalg.norm([lv.x,lv.y,lv.z]))
                    if args.approach_lio:
                        from lio_approach_probe import filter_tracking_errors
                        ev=odom.twist.twist.linear
                        filter_error,filter_speed=filter_tracking_errors(xyz,R@np.array([ev.x,ev.y,ev.z]),goal,fine_bias,fine_velocity)
                    sample=dict(sim_s=now,error_lio_m=error_lio,error_ekf_m=filter_error,speed_m_s=filter_speed,yaw_error_rad=yaw_error,
                        raw_filter_goal_error_m=float(np.linalg.norm(xyz-goal)),raw_filter_speed_m_s=float(speed_now),lio_speed_m_s=lio_speed,
                        frame_position_error_m=agreement['position_error_m'],tracking_error_m=tracking_error,arm_deviation_bound_m=arm_deviation,attitude_error_deg=rotation_error)
                    candidate_samples.append(sample)
                    ready=candidate_route.finished(now) and active and error_lio<.05 and filter_error<.05 and filter_speed<.05 and lio_speed<.05 and yaw_error<math.radians(5)
                    if not ready:candidate_held=None
                    elif candidate_held is None:candidate_held=now
                    if not result.get('candidate_navigation') and candidate_held is not None and now-candidate_held>=5:
                        invariants=assert_unchanged(candidate_baseline,snapshot(native))
                        result['candidate_navigation']=dict(completed=True,goal_map_m=goal.tolist(),duration_s=now-candidate_route.began,
                            final_sample=sample,invariants=invariants,ground_truth_control_inputs=False,arm_motion_requested=False)
                        save(folder/'candidate_navigation_samples.json',candidate_samples)
                        save(folder/'graph_candidate_completed.json',check_online_graph())
                        if not args.approach_lio:end=now;break
                        from lio_approach_probe import LioApproachProbe
                        approach_probe=LioApproachProbe(native,tool_observer,config,plan.geometry_profile,
                            inspection['candidate'],inspection['result']['unfold_precheck']['final_q'],
                            folder/'lio_approach',latest,folder/'scene.world',np.load(folder/'approach_map_returns.npy'),grasp=args.grasp_lio)
                        approach_probe.start();print('LIO/FK native '+('contact grasp' if args.grasp_lio else 'no-close approach')+' adapter started; matching terminal flight binding follows',flush=True)
                    if approach_probe is not None:
                        if approach_probe.error:raise RuntimeError('LIO approach: '+approach_probe.error)
                        if approach_probe.complete:
                            result['lio_approach']=approach_probe.finish()
                            result['fine_command_bias_m']=fine_bias.tolist();result['fine_velocity_bias_m_s']=fine_velocity.tolist();end=now;break
                if manager.data=='FAILSAFE_HOVER':raise RuntimeError('Manager input lease failed')
                time.sleep(.005)
    except (Exception,KeyboardInterrupt) as e:result['error']=str(e) or type(e).__name__
    finally:
        # Complete bounded owned-process cleanup and save the result even if
        # another interrupt arrives while a child is already shutting down.
        old_interrupt=signal.signal(signal.SIGINT,signal.SIG_IGN)
        if approach_probe is not None and 'lio_approach' not in result:
            try:result['lio_approach']=approach_probe.finish()
            except Exception as e:result['approach_cleanup_error']=str(e)
        if inspection_thread is not None:
            inspection_thread.join(timeout=15)
            if inspection_thread.is_alive():result['inspection_cleanup_error']='IK inspection worker did not stop'
        if observation is not None:
            try:observation.close()
            except Exception as e:result['observation_cleanup_error']=str(e)
        if tool_observer is not None:tool_observer.close()
        if tool_log is not None:tool_log.close()
        if pub:pub.unregister()
        if fcu_pub:fcu_pub.unregister()
        for sub in subs:sub.unregister()
        for p in reversed(procs):stop(p)
        for log in logs:log.close()
        if candidate_samples:save(folder/'candidate_navigation_samples.json',candidate_samples)
        signal.signal(signal.SIGINT,old_interrupt)
    all_stopped=all(p.poll() is not None for p in procs)
    if all_stopped and result.get('control_flight_completed') and (not args.observe_target or result.get('fk_observation_completed')) and end is not None:
        try:
            result['offline_score']=score_recording(folder,begin,end,goal)
            result['success']=result['offline_score']['passed'] and (not args.observe_target or result['offline_score'].get('target',{}).get('passed',False) and result['offline_score'].get('tool',{}).get('passed',False))
        except Exception as e:result['offline_score_error']=str(e)
    result.update(ground_truth_control_inputs=False,arm_motion_requested=bool(approach_probe and approach_probe.motion_requested),
        external_velocity_fused=args.fuse_lio_velocity,external_odometry_samples=external_count,external_odometry_max_age_s=external_max_age,
        own_processes_stopped=all_stopped,wall_elapsed_s=time.monotonic()-started)
    if not result['own_processes_stopped'] or result.get('observation_cleanup_error') or result.get('inspection_cleanup_error'):result['success']=False
    if args.inspect_ik and not result.get('ik_inspection',{}).get('inspection_verified'):result['success']=False
    if args.navigate_candidate and not result.get('candidate_navigation',{}).get('completed'):result['success']=False
    if args.approach_lio and not result.get('lio_approach',{}).get('completed'):result['success']=False
    if args.grasp_lio and all_stopped and result.get('lio_approach',{}).get('completed'):
        try:
            result['payload_offline_score']=score_payload_recording(folder,result['lio_approach'])
            result['success']=result['success'] and result['payload_offline_score']['passed']
        except Exception as e:result['success']=False;result['payload_offline_score_error']=str(e)
    save(folder/'result.json',result);print(json.dumps(result,indent=2),flush=True)
    return 0 if result['success'] else 2

if __name__=='__main__':raise SystemExit(main())
