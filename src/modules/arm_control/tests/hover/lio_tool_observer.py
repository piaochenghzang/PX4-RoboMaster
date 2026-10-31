"""Read-only LIO + encoder FK. No truth input, motion clients or permissions."""
from collections import deque
import math
from pathlib import Path
import sys
import threading
import numpy as np

sys.path.insert(0,str(Path('/home/pcz/super_ws/src/super_px4_bridge/scripts')))
from grasp_audit.geometry import finite,fk_gripper,interpolate_pose,inverse,pose_transform,rpy_transform


def ordered_joints(names,positions,expected_names):
    names=list(names);expected=list(expected_names)
    if len(names)!=6 or len(expected)!=6 or len(set(names))!=6 or len(set(expected))!=6 or set(names)!=set(expected):
        raise ValueError('JOINT_NAMES')
    q=finite(positions,(6,))
    return q[[names.index(n) for n in expected]].copy()


def relative_tool_target(config,body_map,q,target_map,reference_gripper):
    """T_parent_child maps child coordinates into parent coordinates."""
    body_map=finite(body_map,(4,4));target_map=finite(target_map,(4,4))
    inverse(body_map);inverse(target_map) # Validate proper rigid transforms.
    arm=body_map@rpy_transform(config['mount_pose'])
    gripper=arm@fk_gripper(config,q)
    tcp=gripper@rpy_transform(config['tool_pose'])
    G_reference=np.eye(4);G_reference[:3,3]=finite(reference_gripper,(3,))
    reference=gripper@G_reference
    relative=inverse(reference)@target_map
    return dict(map_from_arm=arm,map_from_gripper=gripper,map_from_tcp=tcp,
        map_from_grasp_reference=reference,arm_from_target=inverse(arm)@target_map,
        tcp_from_target=inverse(tcp)@target_map,grasp_reference_from_target=relative,
        distance_m=float(np.linalg.norm(relative[:3,3])))


class LioToolObserver:
    """Sample at measured joint time; LIO must bracket it without extrapolation."""
    def __init__(self,config,reference_gripper):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        self.config=config;self.reference=finite(reference_gripper,(3,)).copy()
        self.lock=threading.RLock();self.body=deque(maxlen=400);self.joints=deque(maxlen=1000)
        self.error=None;self.target=None;self.target_stamp=None;self.last_stamp=0.
        self.subscribers=[rospy.Subscriber('/lidar_slam/odom',Odometry,self.on_body,queue_size=100,tcp_nodelay=True),
            rospy.Subscriber('/so101/joint_states',JointState,self.on_joints,queue_size=100,tcp_nodelay=True)]

    def on_body(self,m):
        with self.lock:
            try:
                if m.header.frame_id!='map' or m.child_frame_id!='base_link':raise ValueError('LIO_FRAME')
                t=m.header.stamp.to_sec();p=m.pose.pose
                v=[p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]
                pose_transform(v)
                if not math.isfinite(t) or t<=0 or (self.body and t<=self.body[-1][0]):raise ValueError('LIO_SOURCE_TIME')
                self.body.append((t,v))
            except ValueError as e:self.error=str(e)

    def on_joints(self,m):
        with self.lock:
            try:
                q=ordered_joints(m.name,m.position,self.config['joint_names'])
                velocity=ordered_joints(m.name,m.velocity,self.config['joint_names'])
                t=m.header.stamp.to_sec()
                if not math.isfinite(t) or t<=0 or (self.joints and t<=self.joints[-1][0]):raise ValueError('JOINT_SOURCE_TIME')
                self.joints.append((t,q,velocity))
            except ValueError as e:self.error=str(e)

    def lock_target(self,estimate):
        with self.lock:
            if self.target is not None:raise ValueError('TARGET_ALREADY_LOCKED')
            source=estimate['source_observation']
            if estimate['frame']!='map' or source['reference_frame']!='map' or source['localization']!='FAST_LIO_BODY_MAP_POSE':
                raise ValueError('TARGET_FRAME_OR_SOURCE')
            target=finite(estimate['map_from_object'],(4,4)).copy();inverse(target)
            stamp=float(source['observation_end_ns'])*1e-9
            if not math.isfinite(stamp) or stamp<=0:raise ValueError('TARGET_SOURCE_TIME')
            target.flags.writeable=False;self.target=target;self.target_stamp=stamp

    def sample(self,now):
        with self.lock:
            if self.error:raise ValueError(self.error)
            if not math.isfinite(now) or now<=0:raise ValueError('CURRENT_TIME')
            if self.target is None or len(self.body)<2 or not self.joints:return None
            # Use an actual encoder measurement with a before/after LIO pair.
            candidates=[r for r in self.joints if max(self.body[0][0],self.target_stamp,self.last_stamp)<r[0]<=min(self.body[-1][0],now)]
            if not candidates:return None
            t,q,velocity=candidates[-1]
            if not 0<=now-t<=.2:raise ValueError('PAIRED_STATE_STALE_OR_FUTURE')
            before=max(r[0] for r in self.body if r[0]<=t)
            after=min(r[0] for r in self.body if r[0]>=t)
            if after-before>.125+1e-9:raise ValueError('LIO_PAIR_GAP')
            body,skew=interpolate_pose(list(self.body),t,.125)
            result=relative_tool_target(self.config,body,q,self.target,self.reference)
            result={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in result.items()}
            result.update(frame='map',source='LIO_MOUNT_ENCODER_FK',stamp_s=t,source_age_s=now-t,
                lio_before_stamp_s=before,lio_after_stamp_s=after,lio_skew_s=skew,
                joint_names=list(self.config['joint_names']),joint_position_rad=q.tolist(),joint_velocity_rad_s=velocity.tolist(),
                joint_time_basis='ROS_SIM_CLOCK_APPROXIMATE_PHYSICS_SAMPLE',map_from_body=body.tolist(),
                map_from_target=self.target.tolist(),target_observation_stamp_s=self.target_stamp,
                target_policy='STATIC_TARGET_LOCKED_AT_OBSERVATION',motion_permission=False,close_permission=False)
            self.last_stamp=t
            return result

    def close(self):
        for sub in self.subscribers:sub.unregister()
