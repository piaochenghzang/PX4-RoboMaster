"""Minimal known-shape LiDAR demo adapters; native arm/flight controllers stay unchanged."""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from fixed_demo_plan import FixedDemoPlan, PAD_NAMES, quaternion
from known_target import KnownTarget, inverse

PERCEPTION=Path('/home/pcz/super_ws/src/super_px4_bridge/scripts/perception')
sys.path.insert(0,str(PERCEPTION))
from check_installation import BRIDGE, load_profile, transform, sensor_transform, box_gap
from run_static_scan import box, pose_text

PROFILE=BRIDGE/'config/perception/mid360s_s1.yaml'


class LidarDemoPlan(FixedDemoPlan):
    def __init__(self,target_file,geometry=None):
        super().__init__(target_file)
        self.geometry_profile=geometry or load_profile(PROFILE)
        # Preserve the observed orientation; never replace it with the offline IK orientation.
        config=json.loads(Path(target_file).read_text())
        measured=KnownTarget(dict(self.config,object_quaternion_world_wxyz=config['object_quaternion_world_wxyz']))
        angle=math.acos(float(np.clip((np.trace(self.target.object[:3,:3].T@measured.object[:3,:3])-1)/2,-1,1)))
        if angle>math.radians(5):raise ValueError('Observed grasp orientation incompatible with existing position IK')
        self.config['object_quaternion_world_wxyz']=config['object_quaternion_world_wxyz']
        self.target=KnownTarget(self.config)
        self.arm=self.target.hover@self.target.mount
        self.parts=[(p['name'],transform(p['xyz_rpy_deg']),np.array(p['size_m'])/2)
                    for p in self.geometry_profile['target']['parts']]
        self.support=self.target.object@transform(self.geometry_profile['target']['object_from_support_xyz_rpy_deg'])
        self.support_half=np.array(self.geometry_profile['target']['support_size_m'])/2
        # Seat the fixed finger before the existing Align dwell closes the moving jaw.
        # The 2.3mm correction stays inside the original 3mm close-entry budget.
        self.close_reference=np.array([.0022,-.000218,-.104])

    def geometry(self,q,jaw,unfold=False):
        rows=[]
        obstacles=[(name,self.target.object@T,h) for name,T,h in self.parts]
        obstacles.append(('support',self.support,self.support_half))
        for name,T,h in self.boxes(q,jaw):
            rows.append(dict(collision=name,obstacle='ground/asphalt',distance=float(T[2,3]-abs(T[2,:3])@h-.05),required=.016))
            for part,P,ph in obstacles:
                if not unfold and name in PAD_NAMES and part=='handle':continue
                rows.append(dict(collision=name,obstacle=part,distance=box_gap(T,h,P,ph),
                    required=.003 if not unfold and name in PAD_NAMES and part=='support' else .016))
        return sorted(rows,key=lambda row:row['distance']-row['required'])

    def preview(self):
        r=super().preview()
        r.update(target_profile='mid360s_body_key_handle',contact_exception='two pads versus designated handle ONLY',
                 support_size_m=(self.support_half*2).tolist(),observed_orientation_preserved=True,gripper_p=1.5,gripper_d=.001)
        return r


def add_payload(world,target,geometry):
    """One dynamic rigid body, analytic positive inertia and explicit COM; no attachment."""
    cfg=geometry['target'];m=ET.SubElement(world,'model',name='test_cube')
    ET.SubElement(m,'static').text='false';ET.SubElement(m,'pose').text=pose_text(target.object)
    link=ET.SubElement(m,'link',name='link')
    sizes=np.array([p['size_m'] for p in cfg['parts']]);centers=np.array([p['xyz_rpy_deg'][:3] for p in cfg['parts']])
    mass=cfg['candidate_total_mass_kg'];weights=np.prod(sizes,axis=1);weights=mass*weights/sum(weights)
    com=np.sum(weights[:,None]*centers,axis=0)/mass;I=np.zeros((3,3))
    for part,w,center,size in zip(cfg['parts'],weights,centers,sizes):
        R=transform(part['xyz_rpy_deg'])[:3,:3];d=center-com
        local=np.diag(w/12*np.array([size[1]**2+size[2]**2,size[0]**2+size[2]**2,size[0]**2+size[1]**2]))
        I+=R@local@R.T+w*((d@d)*np.eye(3)-np.outer(d,d))
    if np.min(np.linalg.eigvalsh(I))<=0:raise ValueError('Invalid payload inertia')
    inertial=ET.SubElement(link,'inertial');ET.SubElement(inertial,'mass').text=str(mass)
    ET.SubElement(inertial,'pose').text=' '.join(map(str,[*com,0,0,0]));tensor=ET.SubElement(inertial,'inertia')
    for name,i,j in [('ixx',0,0),('iyy',1,1),('izz',2,2),('ixy',0,1),('ixz',0,2),('iyz',1,2)]:
        ET.SubElement(tensor,name).text=str(I[i,j])
    for part in cfg['parts']:
        box(link,'collision' if part['name']=='handle' else part['name'],transform(part['xyz_rpy_deg']),part['size_m'],(.8,.35,.1,1))
        col=link.findall('collision')[-1]
        surface=ET.SubElement(col,'surface');friction=ET.SubElement(ET.SubElement(surface,'friction'),'ode')
        for name in ('mu','mu2'):ET.SubElement(friction,name).text='1.2' if part['name']=='handle' else '.4'
        contact=ET.SubElement(ET.SubElement(surface,'contact'),'ode')
        ET.SubElement(contact,'min_depth').text='.00005' if part['name']=='handle' else '.0005'
        ET.SubElement(contact,'max_vel').text='.01' if part['name']=='handle' else '.05'
        if part['name']=='handle':
            # Rubber-like simulation candidate, not measured material data.
            # Avoid a rigid overconstraint between both fingers and the support.
            ET.SubElement(contact,'kp').text='500';ET.SubElement(contact,'kd').text='10'
    support=ET.SubElement(world,'model',name='cube_support');ET.SubElement(support,'static').text='true'
    ET.SubElement(support,'pose').text=pose_text(target.object@transform(cfg['object_from_support_xyz_rpy_deg']))
    box(ET.SubElement(support,'link',name='link'),'collision',np.eye(4),cfg['support_size_m'],(.3,.3,.3,1))
    return dict(mass_kg=mass,com_object_m=com.tolist(),inertia_com_kg_m2=I.tolist(),body_key_friction=.4,handle_friction=1.2,
        handle_contact=dict(kp_n_m=500,kd_n_s_m=10,min_depth_m=.00005,max_correction_velocity_m_s=.01,hardware_calibrated=False))


def adapt_world(world,vehicle,target,geometry):
    for name in ('test_cube','cube_support'):world.remove(world.find("model[@name='%s']"%name))
    inertia=add_payload(world,target,geometry)
    observer=world.find("plugin[@name='coordinate_truth']")
    ET.SubElement(observer,'targetProfile').text='mid360s_body_key_handle'
    # Housing and plate are a real payload, rigidly mounted to the body.
    l=geometry['lidar'];mount=ET.SubElement(vehicle,'link',name='mid360s_mount')
    B_F=transform(l['body_from_base_xyz_rpy_deg']);ET.SubElement(mount,'pose').text=pose_text(B_F)
    plate_mass=float(np.prod(l['plate_size_m'])*l['plate_density_kg_m3']);total=l['mass_kg']+plate_mass
    cg=(l['mass_kg']*l['housing_size_m'][2]/2-plate_mass*l['plate_size_m'][2]/2)/total
    inertial=ET.SubElement(mount,'inertial');ET.SubElement(inertial,'mass').text=str(total)
    ET.SubElement(inertial,'pose').text='0 0 '+str(cg)+' 0 0 0'
    inertia_element=ET.SubElement(inertial,'inertia');diag=np.zeros(3)
    for mass,size,z in [(l['mass_kg'],l['housing_size_m'],l['housing_size_m'][2]/2),(plate_mass,l['plate_size_m'],-l['plate_size_m'][2]/2)]:
        x,y,h=size;diag+=mass*np.array([(y*y+h*h)/12+(z-cg)**2,(x*x+h*h)/12+(z-cg)**2,(x*x+y*y)/12])
    for name,value in zip(('ixx','iyy','izz','ixy','ixz','iyz'),[*diag,0,0,0]):ET.SubElement(inertia_element,name).text=str(value)
    box(mount,'plate',transform([0,0,-l['plate_size_m'][2]/2,0,0,0]),l['plate_size_m'],(.15,.15,.15,1))
    # Keep the lower housing below the optical ray origin.
    box(mount,'housing',transform([0,0,.01975,0,0,0]),[.065,.065,.0395],(.15,.15,.15,1))
    joint=ET.SubElement(vehicle,'joint',name='mid360s_fixed',type='fixed')
    ET.SubElement(joint,'parent').text='yhang550_base::base_link';ET.SubElement(joint,'child').text='mid360s_mount'
    cfg=yaml.safe_load((BRIDGE/'config/perception/mid360s_static.yaml').read_text())
    plugin=ET.SubElement(world,'plugin',name='demo_lidar',filename='libso101_mid360s_static.so')
    params=dict(trace_file=BRIDGE/cfg['trace_file'],point_rate=200000,frame_rate=10,imu_rate=200,
        min_range=l['min_range_m'],max_range=l['max_range_m'],low_deg=l['vertical_fov_deg'][0],high_deg=l['vertical_fov_deg'][1],
        noise_sigma=0,seed=19,points_topic=cfg['topics']['points'],imu_topic=cfg['topics']['imu'],truth_topic=cfg['topics']['truth'],
        demoMovingSensor='true',vehicle_name='yhang550',sensor_link='yhang550_base::base_link',sensor_pose=pose_text(sensor_transform(geometry)))
    for name,value in params.items():ET.SubElement(plugin,name).text=str(value)
    return dict(payload=inertia,lidar_mount_mass_kg=total,ideal_range_noise_m=0,moving_imu_is_gravity_cue_not_lio=True)


class TargetObservation:
    """One stable hover snapshot paired to explicitly simulated body poses, never target truth."""
    def __init__(self,geometry,folder):
        import threading
        from collections import deque
        import rospy
        from sensor_msgs.msg import PointCloud2,Imu
        from nav_msgs.msg import Odometry
        from super_px4_bridge.msg import CoordinateTruth
        self.geometry=geometry;self.folder=folder;self.lock=threading.RLock()
        self.clouds=deque(maxlen=40);self.imus=deque(maxlen=1000);self.body=deque(maxlen=1000)
        self.thread=None;self.result=None;self.error=None;self.attempts=0
        self.subscribers=[rospy.Subscriber(topic,typ,lambda m,k=key:self.receive(k,m),queue_size=1000,tcp_nodelay=True)
            for topic,typ,key in [('/mid360s_sim/points',PointCloud2,'clouds'),('/mid360s_sim/imu',Imu,'imus')]]
        self.subscribers.append(rospy.Subscriber('/grasp_audit/truth',CoordinateTruth,self.body_pose,queue_size=1000,tcp_nodelay=True))

    def body_pose(self,m):
        # Explicit simulation localization only. Object pose, labels and joints
        # are not read or copied into the perception input.
        from nav_msgs.msg import Odometry
        if m.header.frame_id!='gazebo_world':return
        try:i=list(m.link_names).index('yhang550_base::base_link')
        except ValueError:return
        odom=Odometry();odom.header=m.header;odom.child_frame_id='base_link'
        odom.pose.pose=m.link_poses[i]
        with self.lock:self.body.append(odom)

    def receive(self,key,value):
        with self.lock:getattr(self,key).append(value)

    def poll(self,ready):
        import threading
        if self.thread and self.thread.is_alive():return None
        if self.error:raise RuntimeError(self.error)
        if self.result:return self.result
        if not ready:return None
        with self.lock:
            if len(self.clouds)<15 or len(self.body)<100:return None
            clouds=list(self.clouds)[-12:-2];odom=list(self.body);imus=list(self.imus)
        self.attempts+=1
        if self.attempts>4:raise RuntimeError('No reliable LiDAR snapshot after four attempts; arm remains folded')
        self.thread=threading.Thread(target=self.fit,args=(clouds,odom,imus),daemon=True);self.thread.start()
        return None

    def fit(self,clouds,odom,imus):
        try:
            import rosbag
            from scan_metrics import cloud_array,xyz,point_times
            from static_target import estimate,shape_from_geometry
            from grasp_audit.geometry import interpolate_pose
            poses=[]
            for m in odom:
                p=m.pose.pose
                if m.header.frame_id!='gazebo_world' or m.child_frame_id!='base_link':raise ValueError('Unexpected simulated body frames')
                poses.append((m.header.stamp.to_sec(),[p.position.x,p.position.y,p.position.z,
                    p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]))
            poses=sorted(dict(poses).items())
            stamps=[m.header.stamp.to_nsec() for m in clouds]
            if len(stamps)!=10 or np.any(np.diff(stamps)!=100000000):raise ValueError('Incomplete LiDAR window')
            begin=stamps[0];end=stamps[-1]+100000000
            # Pair to source BODY poses, never to receipt time or object truth.
            ref,_=interpolate_pose(poses,end*1e-9,.05);B_L=sensor_transform(self.geometry);M_L=ref@B_L
            arrays=[];max_motion=0.;max_angle=0.
            for cloud in clouds:
                if cloud.header.frame_id!='mid360s_lidar':raise ValueError('Unexpected cloud frame')
                a=cloud_array(cloud);t=point_times(cloud,a,100000000);raw=xyz(a)
                # The generator uses one pose per 1ms physics step. Deskew to
                # a single L frame in 5ms source-time bins from the simulated body pose stream.
                corrected=np.empty_like(raw)
                for key in np.unique(t//5000000):
                    mask=t//5000000==key;stamp=float(np.mean(t[mask]))*1e-9
                    B,_=interpolate_pose(poses,stamp,.05);delta=inverse(M_L)@B@B_L
                    corrected[mask]=raw[mask]@delta[:3,:3].T+delta[:3,3]
                    max_motion=max(max_motion,float(np.linalg.norm(B[:3,3]-ref[:3,3])))
                    max_angle=max(max_angle,math.acos(float(np.clip((np.trace(B[:3,:3].T@ref[:3,:3])-1)/2,-1,1))))
                arrays.append(corrected)
            if max_motion>.01 or max_angle>math.radians(1):return
            selected=[m for m in imus if begin<=m.header.stamp.to_nsec()<=end]
            if len(selected)<190:raise ValueError('Incomplete gravity cue window')
            ups=[]
            for m in selected:
                a=m.linear_acceleration;B,_=interpolate_pose(poses,m.header.stamp.to_sec(),.05)
                ups.append((inverse(M_L)@B@B_L)[:3,:3]@np.array([a.x,a.y,a.z]))
            up=np.median(ups,axis=0);up/=np.linalg.norm(up)
            cfg=yaml.safe_load((BRIDGE/'config/perception/static_target.yaml').read_text())
            report,_,_=estimate(np.concatenate(arrays),shape_from_geometry(self.geometry),cfg,up)
            report['source_observation']=dict(observation_start_ns=begin,observation_end_ns=end,restamped=False,
                frame='mid360s_lidar',body_world_at_reference=ref.tolist(),lidar_world_at_reference=M_L.tolist(),
                localization='SIM_BODY_WORLD_POSE_NOT_TARGET_TRUTH',deskew_bin_s=.005,max_body_translation_m=max_motion,max_body_rotation_rad=max_angle,
                moving_imu='stable-window specific-force direction; dynamics selected by scene profile')
            out=self.folder/('perception_attempt_'+str(self.attempts));out.mkdir()
            (out/'estimate.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
            with rosbag.Bag(str(out/'observation.bag'),'w',compression='lz4') as bag:
                for topic,items in [('/mid360s_sim/points',clouds),('/mid360s_sim/imu',selected),('/grasp_demo/sim_body_pose_snapshot',odom)]:
                    for m in items:bag.write(topic,m,m.header.stamp)
            if not report['valid']:return
            best=report['best'];L_O=np.eye(4);L_O[:3,:3]=best['R'];L_O[:3,3]=best['position_lidar_m']
            M_O=M_L@L_O
            self.result=dict(transform=M_O,report=report,path=out)
        except Exception as exc:self.error='LiDAR observation failed: '+str(exc)

    def close(self):
        for sub in self.subscribers:sub.unregister()
        if self.thread:self.thread.join(timeout=15)
        if self.thread and self.thread.is_alive():raise RuntimeError('Perception worker did not stop')
