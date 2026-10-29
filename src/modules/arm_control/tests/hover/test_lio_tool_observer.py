#!/usr/bin/env python3
"""Read-only frame-chain invariants and measured source-time admission."""
import copy
import math
from pathlib import Path
import sys
import types
import unittest
from unittest import mock
import numpy as np
import lio_tool_observer as tool

NAMES=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']


def configuration():
    return dict(joint_names=NAMES.copy(),mount_pose=[1,2,3,0,0,math.pi/2],
        joint_origins=[[.5,0,0,0,0,0]]+[[0]*6 for _ in range(4)],
        tool_pose=[.2,0,0,0,math.pi,0])


def inputs():
    body=np.eye(4);body[:3,3]=[10,-3,4]
    target=np.eye(4);target[:3,3]=[11.2,-.2,7.5]
    q=np.array([math.pi/2,0,0,0,0,.3])
    return body,q,target,np.array([.1,0,.05])


class JointNameTests(unittest.TestCase):
    def test_shuffled_feedback_is_reordered_by_joint_identity(self):
        names=NAMES[::-1];positions=np.arange(6,dtype=float)
        actual=tool.ordered_joints(names,positions,NAMES)
        np.testing.assert_array_equal(actual,[5,4,3,2,1,0])
        positions[:]=20
        np.testing.assert_array_equal(actual,[5,4,3,2,1,0])

    def test_missing_duplicate_or_unknown_joint_names_are_rejected(self):
        for names in [NAMES[:-1],NAMES+[NAMES[0]],NAMES[:-1]+[NAMES[0]],NAMES[:-1]+['unknown']]:
            with self.subTest(names=names):
                with self.assertRaises(ValueError):tool.ordered_joints(names,np.zeros(len(names)),NAMES)
        with self.assertRaises(ValueError):tool.ordered_joints(NAMES,np.zeros(6),NAMES+[NAMES[0]])

    def test_invalid_joint_array_or_nonfinite_measurement_is_rejected(self):
        for positions in [[0]*5,[0]*7,[0,0,float('nan'),0,0,0],[0,0,0,0,0,float('inf')]]:
            with self.subTest(positions=positions):
                with self.assertRaises(ValueError):tool.ordered_joints(NAMES,positions,NAMES)


class FrameChainTests(unittest.TestCase):
    def test_quarter_turn_mount_and_joint_have_hand_calculated_positions(self):
        body,q,target,reference=inputs()
        out=tool.relative_tool_target(configuration(),body,q,target,reference)
        # Mount yaw=90deg, joint yaw=90deg: the first .5m link origin
        # lies along map +Y, while the gripper's +X points along map -X.
        np.testing.assert_allclose(np.array(out['map_from_arm'])[:3,3],[11,-1,7],atol=1e-12)
        np.testing.assert_allclose(np.array(out['map_from_gripper'])[:3,3],[11,-.5,7],atol=1e-12)
        np.testing.assert_allclose(np.array(out['map_from_tcp'])[:3,3],[10.8,-.5,7],atol=1e-12)
        np.testing.assert_allclose(np.array(out['map_from_grasp_reference'])[:3,3],[10.9,-.5,7.05],atol=1e-12)
        np.testing.assert_allclose(np.array(out['arm_from_target'])[:3,3],[.8,-.2,.5],atol=1e-12)
        np.testing.assert_allclose(np.array(out['tcp_from_target'])[:3,3],[.4,-.3,-.5],atol=1e-12)
        np.testing.assert_allclose(np.array(out['grasp_reference_from_target'])[:3,3],[-.3,-.3,.45],atol=1e-12)
        self.assertAlmostEqual(out['distance_m'],math.sqrt(.3**2+.3**2+.45**2),places=12)

    def test_common_map_translation_and_rotation_preserve_relative_geometry(self):
        body,q,target,reference=inputs();cfg=configuration()
        before=tool.relative_tool_target(cfg,body,q,target,reference)
        change=np.array([[0.,0.,1.,-2.],[0.,1.,0.,4.],[-1.,0.,0.,1.],[0.,0.,0.,1.]])
        after=tool.relative_tool_target(cfg,change@body,q,change@target,reference)
        for key in ['arm_from_target','tcp_from_target','grasp_reference_from_target']:
            np.testing.assert_allclose(after[key],before[key],atol=1e-12,err_msg=key)
        for key in ['map_from_arm','map_from_gripper','map_from_tcp','map_from_grasp_reference']:
            np.testing.assert_allclose(after[key],change@np.array(before[key]),atol=1e-12,err_msg=key)
        self.assertAlmostEqual(after['distance_m'],before['distance_m'],places=12)

    def test_gripper_opening_does_not_move_the_gripper_link_or_tcp(self):
        body,q,target,reference=inputs();cfg=configuration()
        before=tool.relative_tool_target(cfg,body,q,target,reference)
        q[5]=.04;after=tool.relative_tool_target(cfg,body,q,target,reference)
        for key in before:np.testing.assert_allclose(after[key],before[key],atol=1e-12,err_msg=key)

    def test_grasp_reference_is_distinct_from_native_tcp(self):
        body,q,target,reference=inputs();target[:3,3]=[10.9,-.5,7.05]
        out=tool.relative_tool_target(configuration(),body,q,target,reference)
        self.assertAlmostEqual(out['distance_m'],0.,places=12)
        self.assertGreater(np.linalg.norm(np.array(out['tcp_from_target'])[:3,3]),.1)
        np.testing.assert_allclose(np.array(out['map_from_grasp_reference'])[:3,:3],np.diag([-1,-1,1]),atol=1e-12)
        np.testing.assert_allclose(np.array(out['map_from_tcp'])[:3,:3],np.diag([1,-1,-1]),atol=1e-12)

    def test_inputs_are_not_modified(self):
        cfg=configuration();args=inputs();saved=copy.deepcopy((cfg,args))
        tool.relative_tool_target(cfg,*args)
        self.assertEqual(cfg,saved[0])
        for actual,expected in zip(args,saved[1]):np.testing.assert_array_equal(actual,expected)

    def test_nonrigid_or_nonfinite_input_frames_are_rejected(self):
        bad=[]
        for index,value in [((0,0),2.),((0,1),.1),((2,2),-1.),((3,0),.1),((0,3),float('nan'))]:
            T=np.eye(4);T[index]=value;bad.append(T)
        bad.append(np.eye(3))
        for T in bad:
            for which in ('body','target'):
                with self.subTest(which=which,T=T):
                    body,q,target,reference=inputs()
                    if which=='body':body=T
                    else:target=T
                    with self.assertRaises(ValueError):tool.relative_tool_target(configuration(),body,q,target,reference)

    def test_invalid_joint_or_grasp_reference_inputs_are_rejected(self):
        body,q,target,reference=inputs()
        for bad in [q[:5],np.r_[q[:5],float('nan')]]:
            with self.assertRaises(ValueError):tool.relative_tool_target(configuration(),body,bad,target,reference)
        for bad in [[0,0],[0,0,float('inf')]]:
            with self.assertRaises(ValueError):tool.relative_tool_target(configuration(),body,q,target,bad)


def message_pose(stamp,x=0.):
    return types.SimpleNamespace(header=types.SimpleNamespace(frame_id='map',stamp=types.SimpleNamespace(to_sec=lambda:stamp)),
        child_frame_id='base_link',pose=types.SimpleNamespace(pose=types.SimpleNamespace(
            position=types.SimpleNamespace(x=x,y=0.,z=0.),orientation=types.SimpleNamespace(w=1.,x=0.,y=0.,z=0.))))


def target_input():
    T=np.eye(4);T[0,3]=1.
    return dict(frame='map',map_from_object=T.tolist(),source_observation=dict(
        reference_frame='map',localization='FAST_LIO_BODY_MAP_POSE',observation_end_ns=900000000))


class ReadOnlyObserverTests(unittest.TestCase):
    def setUp(self):
        rospy=types.ModuleType('rospy');rospy.Subscriber=mock.Mock()
        modules={'rospy':rospy}
        for package,name in [('nav_msgs','Odometry'),('sensor_msgs','JointState')]:
            parent=types.ModuleType(package);messages=types.ModuleType(package+'.msg')
            setattr(messages,name,type(name,(),{}));parent.msg=messages
            modules[package]=parent;modules[package+'.msg']=messages
        self.ros=rospy;self.patcher=mock.patch.dict(sys.modules,modules);self.patcher.start()
        self.addCleanup(self.patcher.stop)
        cfg=dict(joint_names=NAMES.copy(),mount_pose=[0]*6,joint_origins=[[0]*6 for _ in range(5)],tool_pose=[0]*6)
        self.node=tool.LioToolObserver(cfg,[0,0,0]);self.addCleanup(self.node.close)

    def joints(self,stamp):
        return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_sec=lambda:stamp)),
            name=NAMES.copy(),position=[0]*6,velocity=[0]*6)

    def test_only_lio_and_measured_joints_are_subscribed(self):
        topics=[call.args[0] for call in self.ros.Subscriber.call_args_list]
        self.assertCountEqual(topics,['/lidar_slam/odom','/so101/joint_states'])
        self.assertNotIn('/grasp_audit/truth',topics)
        self.assertFalse(hasattr(self.ros,'Publisher'))

    def test_target_lock_copies_input_and_rejects_replacement(self):
        value=target_input();self.node.lock_target(value)
        value['map_from_object'][0][3]=5.
        self.assertAlmostEqual(self.node.target[0,3],1.)
        self.assertFalse(self.node.target.flags.writeable)
        with self.assertRaises(ValueError):self.node.lock_target(target_input())

    def test_paired_sample_uses_encoder_time_and_never_grants_motion(self):
        self.node.lock_target(target_input())
        self.node.on_body(message_pose(1.,0.));self.node.on_body(message_pose(1.1,.1))
        self.node.on_joints(self.joints(1.05))
        row=self.node.sample(1.15)
        self.assertAlmostEqual(row['stamp_s'],1.05)
        np.testing.assert_allclose(np.array(row['grasp_reference_from_target'])[:3,3],[.95,0,0],atol=1e-12)
        self.assertFalse(row['motion_permission']);self.assertFalse(row['close_permission'])
        self.assertIsNone(self.node.sample(1.16))  # A sample cannot be restamped/reused.

    def test_missing_body_endpoint_cannot_produce_an_extrapolated_sample(self):
        self.node.lock_target(target_input())
        self.node.on_body(message_pose(1.));self.node.on_body(message_pose(1.1))
        self.node.on_joints(self.joints(1.15))
        self.assertIsNone(self.node.sample(1.2))

    def test_dropped_lio_frame_and_stale_joint_pair_are_rejected(self):
        self.node.lock_target(target_input())
        self.node.on_body(message_pose(1.));self.node.on_body(message_pose(1.2))
        self.node.on_joints(self.joints(1.1))
        with self.assertRaisesRegex(ValueError,'LIO_PAIR_GAP'):self.node.sample(1.25)
        # Fresh observer data, but an aged source time must still be rejected.
        self.node.body.clear();self.node.joints.clear()
        self.node.on_body(message_pose(1.));self.node.on_body(message_pose(1.1))
        self.node.on_joints(self.joints(1.05))
        with self.assertRaisesRegex(ValueError,'STALE_OR_FUTURE'):self.node.sample(1.3)


if __name__=='__main__':unittest.main()
