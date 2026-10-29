#!/usr/bin/env python3
"""LIO-only local flight input. Ground truth is recorded, then scored OFFLINE."""
import argparse,json,math,os,subprocess as sp,time,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rospy,rosgraph,yaml
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State,PositionTarget
from std_msgs.msg import String
from super_px4_bridge.msg import KnownFlightSetpoint
from known_target import rotation,quintic_segment
from lidar_grasp_demo import LidarDemoPlan,TargetObservation
from native_client import NativeClient
from run_known_target_flight import ROOT,BUILD,WS,MODELS,make_world,save,stop,SIM_VISION_PARAMS

PARAMS=dict(SIM_VISION_PARAMS,EKF2_EV_CTRL=11,EKF2_MAG_TYPE=5,EKF2_EVP_NOISE=.05,
    EKF2_EVA_NOISE=.03,EKF2_DELAY_MAX=150,EKF2_TAU_VEL=.15,
    # Measured collective in the previous loaded-vehicle hover; experiment only.
    MPC_THR_HOVER=.84)


def score_recording(folder,begin,end,goal):
    # This function runs only AFTER every owned simulator/control process stops.
    import rosbag
    from grasp_audit.geometry import interpolate_pose
    truth=[];targets=[];estimated=[];alignment=None
    with rosbag.Bag(str(folder/'audit.bag')) as bag:
        for topic,m,_ in bag.read_messages():
            if topic=='/grasp_audit/truth':
                i=list(m.link_names).index('yhang550_base::base_link');p=m.link_poses[i]
                truth.append((m.header.stamp.to_sec(),[p.position.x,p.position.y,p.position.z,
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
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('name');ap.add_argument('--gui',action='store_true');ap.add_argument('--observe-target',action='store_true',help='LIO-paired target observation only; never move the arm');args=ap.parse_args()
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
    env.update({'PX4_PARAM_'+k:str(v) for k,v in PARAMS.items()});env.pop('ROS_IP',None)
    plan=LidarDemoPlan(Path(__file__).with_name('known_target_lidar.json'))
    make_world(folder,plan.target,env,fixed_grasp=True,lidar_geometry=plan.geometry_profile,fastlio=True)
    scene=ET.parse(folder/'scene.world');model=scene.find("world/model[@name='yhang550']")
    model.find('pose').text='0 0 .45 0 0 0'
    bridge=model.find("plugin[@name='mavlink_interface']")
    for el in list(bridge):
        if el.tag in ('enableArmGraspFeedback','send_odometry','visionSubTopic'):bridge.remove(el)
    if any(e.get('filename')=='libgazebo_vision_plugin.so' for e in model.findall('plugin')):raise RuntimeError('Ground-truth localization source present')
    scene.write(folder/'scene.world',encoding='unicode',xml_declaration=True)
    mavros=Path(sp.check_output(['rospack','find','mavros'],env=env,text=True).strip())/'launch'
    cfg=yaml.safe_load((mavros/'px4_config.yaml').read_text());cfg['time']['timesync_mode']='PASSTHROUGH';cfg['conn']['timesync_rate']=0.
    save(folder/'mavros_config.json',cfg);launch=ET.parse(mavros/'px4.launch')
    launch.find("include/arg[@name='config_yaml']").set('value',str(folder/'mavros_config.json'));launch.write(folder/'mavros.launch')
    save(folder/'manifest.json',dict(stage='LIO_TARGET_OBSERVATION_NO_ARM_MOTION' if args.observe_target else 'LIO_LOCAL_FLIGHT_NO_ARM_MOTION',ground_truth_control_inputs=False,
        gazebo_vision_plugin=False,lio_alignment_source='FIRST_LIO_BODY_POSE',ekf_parameters=PARAMS,
        target_observation=args.observe_target,external_velocity_fused=False,planned_local_goal_m=[.3,0,.9],sim_speed_factor=speed))
    os.environ.update(ROS_MASTER_URI=env['ROS_MASTER_URI'],ROS_HOSTNAME='localhost');os.environ.pop('ROS_IP',None)
    procs=[];logs=[];subs=[];pub=None;latest={};result=dict(success=False);begin=end=None;observation=None
    def start(name,cmd,cwd=None):
        log=(folder/(name+'.log')).open('w');logs.append(log)
        p=sp.Popen(cmd,env=env,cwd=cwd,stdout=log,stderr=sp.STDOUT,start_new_session=True);procs.append(p);return p
    started=time.monotonic()
    def health():
        if any(p.poll() is not None for p in procs):raise RuntimeError('Own process exited')
        if time.monotonic()-started>200/speed:raise RuntimeError('Wall watchdog')
    try:
        core=start('roscore',['roscore']);deadline=time.monotonic()+20
        while not rosgraph.is_master_online():
            if time.monotonic()>deadline:raise RuntimeError('ROS unavailable')
            time.sleep(.1)
        sp.run(['rosparam','set','/use_sim_time','true'],env=env,check=True)
        rospy.init_node('lio_hover_probe',disable_signals=True)
        for topic,typ,key in [('/mavros/state',State,'state'),('/mavros/local_position/odom',Odometry,'odom'),
            ('/lidar_slam/odom',Odometry,'lio'),('/grasp_demo/flight_state',String,'manager')]:
            subs.append(rospy.Subscriber(topic,typ,lambda m,k=key:latest.__setitem__(k,m),queue_size=50,tcp_nodelay=True))
        start('audit',['rosbag','record','--lz4','-O',str(folder/'audit.bag'),'/grasp_audit/truth','/lidar_slam/odom',
            '/mavros/local_position/odom','/mavros/state','/fastlio_sim/alignment','/grasp_demo/known_setpoint','__name:=lio_hover_audit_recorder'])
        start('gazebo',['gzserver','--verbose','--seed','123','-s','libgazebo_ros_api_plugin.so',str(folder/'scene.world')])
        if args.gui:start('gui',['gzclient'])
        px4=start('px4',[str(BUILD/'bin/px4'),'-d',str(BUILD/'etc')],folder/'rootfs')
        start('mavros',['roslaunch',str(folder/'mavros.launch'),'fcu_url:=udp://:14540@127.0.0.1:14580'])
        while rospy.Time.now().to_sec()<5:health();time.sleep(.05)
        start('fastlio',['roslaunch','super_px4_bridge','fastlio_sim.launch','alignment_source:=lio_origin','publish_to_fcu:=true'])
        native=NativeClient(BUILD,env,transport='unix',expected_pid=px4.pid)
        while True:
            health();now=rospy.Time.now().to_sec();odom=latest.get('odom');lio=latest.get('lio')
            if now>40:raise RuntimeError('LIO/EKF startup timeout')
            if odom and lio and now-lio.header.stamp.to_sec()<.15 and now-odom.header.stamp.to_sec()<.15:
                flags=native.status('estimator_status_flags')
                if all(flags.get(k) for k in ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw')) and not any(flags.get(k) for k in ('cs_ev_vel','cs_gnss_pos','cs_gnss_vel','cs_baro_hgt','cs_mag_hdg','cs_mag_3d')):
                    raw=native.status('vehicle_visual_odometry')
                    external={k:raw[k] for k in ('timestamp','timestamp_sample','pose_frame','position','q')}
                    save(folder/'ekf_preflight.json',dict(flags=flags,external_pose=external,external_velocity_fused=False));break
            time.sleep(.1)
        graph=rosgraph.Master('/lio_hover_graph').getSystemState();save(folder/'graph_before_flight.json',dict(publishers=graph[0],subscribers=graph[1]))
        if any(node!='/lio_hover_audit_recorder' for topic,nodes in graph[1] if topic=='/grasp_audit/truth' for node in nodes):
            raise RuntimeError('Truth entered a control node')
        if [nodes for topic,nodes in graph[0] if topic=='/mavros/vision_pose/pose_cov']!=[['/fastlio_sim_adapter']]:raise RuntimeError('Unexpected FCU pose publisher')
        start('manager',['rosrun','super_px4_bridge','super_flight_manager_node','_known_goal_mode:=true','_auto_arm:=true',
            '_takeoff_height:=.9','_hover_stable_time:=2.0','__name:=lio_hover_manager'])
        pub=rospy.Publisher('/grasp_demo/known_setpoint',KnownFlightSetpoint,queue_size=1)
        route=None;origin=None;ready=None;held=None;last=-math.inf;goal=np.array([.3,0,.9]);begin=now
        with (folder/'telemetry.jsonl').open('w') as log:
            while True:
                health();now=rospy.Time.now().to_sec();odom=latest.get('odom');lio=latest.get('lio');state=latest.get('state');manager=latest.get('manager')
                if now>110:raise RuntimeError('Simulation-time flight deadline')
                if route is None and now-begin>30 and manager and manager.data!='HOVER_READY':
                    raise RuntimeError('Takeoff did not reach stable hover within 30 simulation seconds')
                if not (odom and lio and state and manager):time.sleep(.01);continue
                if not -.02<=now-lio.header.stamp.to_sec()<.2:raise RuntimeError('LIO input stale')
                if not -.02<=now-odom.header.stamp.to_sec()<.2:raise RuntimeError('EKF output stale')
                p=odom.pose.pose;q=p.orientation;R=rotation([q.w,q.x,q.y,q.z]);xyz=np.array([p.position.x,p.position.y,p.position.z])
                if not np.isfinite(xyz).all() or abs(xyz[0])>1.2 or abs(xyz[1])>.8 or not -.2<xyz[2]<1.5 or R[2,2]<math.cos(math.radians(15)):raise RuntimeError('Estimated flight envelope exceeded')
                v=odom.twist.twist.linear;speed_now=np.linalg.norm([v.x,v.y,v.z])
                active=state.connected and state.armed and state.mode=='OFFBOARD' and -.02<=now-state.header.stamp.to_sec()<1.5
                if manager.data in ('HOVER_READY','TRACK_KNOWN') and not active:raise RuntimeError('Active OFFBOARD state lost')
                if manager.data=='HOVER_READY' and active and route is None:
                    if ready is None:ready=now
                    elif now-ready>=5:route=now;origin=xyz.copy();print('LIO hover accepted; 0.3m local movement starts',flush=True)
                if route is not None and now-last>=.02:
                    wp,wv=quintic_segment(origin,goal,now-route,6.)
                    cmd=KnownFlightSetpoint();cmd.header.stamp=rospy.Time.now();cmd.header.frame_id='map';cmd.task_id=1
                    cmd.setpoint.header=cmd.header;cmd.setpoint.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
                    cmd.setpoint.type_mask=PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW_RATE
                    cmd.setpoint.position.x,cmd.setpoint.position.y,cmd.setpoint.position.z=map(float,wp)
                    cmd.setpoint.velocity.x,cmd.setpoint.velocity.y,cmd.setpoint.velocity.z=map(float,wv);cmd.setpoint.yaw=0.;pub.publish(cmd);last=now
                error=float(np.linalg.norm(xyz-goal))
                good=active and route is not None and now-route>=6 and error<.05 and speed_now<.08
                if not good:held=None
                elif held is None:held=now
                row=dict(sim_s=now,manager=manager.data,estimated_body=xyz.tolist(),goal_error_m=error,lio_age_s=now-lio.header.stamp.to_sec(),
                    observation_window=observation.last_window_check if observation else None)
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
                if observation is not None:
                    v=lio.twist.twist.linear
                    estimate=observation.poll(good and np.linalg.norm([v.x,v.y,v.z])<.02)
                    if estimate is not None:
                        save(folder/'perceived_target_map.json',dict(frame=estimate['frame'],map_from_object=estimate['transform'].tolist(),
                            source_observation=estimate['report']['source_observation'],estimate_path=str(estimate['path']/'estimate.json')))
                        result['target_observation_completed']=True;end=now;break
                if manager.data=='FAILSAFE_HOVER':raise RuntimeError('Manager input lease failed')
                time.sleep(.005)
    except (Exception,KeyboardInterrupt) as e:result['error']=str(e) or type(e).__name__
    finally:
        if observation is not None:
            try:observation.close()
            except Exception as e:result['observation_cleanup_error']=str(e)
        if pub:pub.unregister()
        for sub in subs:sub.unregister()
        for p in reversed(procs):stop(p)
        for log in logs:log.close()
    if result.get('control_flight_completed') and (not args.observe_target or result.get('target_observation_completed')) and end is not None:
        try:
            result['offline_score']=score_recording(folder,begin,end,goal)
            result['success']=result['offline_score']['passed'] and (not args.observe_target or result['offline_score'].get('target',{}).get('passed',False))
        except Exception as e:result['offline_score_error']=str(e)
    result.update(ground_truth_control_inputs=False,arm_motion_requested=False,
        own_processes_stopped=all(p.poll() is not None for p in procs),wall_elapsed_s=time.monotonic()-started)
    if not result['own_processes_stopped'] or result.get('observation_cleanup_error'):result['success']=False
    save(folder/'result.json',result);print(json.dumps(result,indent=2),flush=True)
    return 0 if result['success'] else 2

if __name__=='__main__':raise SystemExit(main())
