#!/usr/bin/env python3
"""Source-time pairing and localization-source isolation; no simulator or commands."""
import sys
import types
import unittest
from pathlib import Path
from unittest import mock
import numpy as np
import lidar_grasp_demo as observation


def sample(stamp):
    # A valid moving pose makes it possible to detect accidental value restamping.
    return (stamp,[stamp*.01,0.,.9,1.,0.,0.,0.])


class PoseWindowTests(unittest.TestCase):
    def poses(self):
        return [sample(round(t*.1,10)) for t in range(9,22)]

    def window(self,poses,begin=1.025,end=1.975):
        return observation.paired_pose_window(poses,begin,end,.125)

    def test_ten_hz_source_poses_bracket_the_entire_window(self):
        poses=self.poses()
        paired=self.window(poses)
        np.testing.assert_allclose([row[0] for row in paired],np.arange(10,21)*.1,atol=1e-12)
        for stamp,pose in paired:self.assertAlmostEqual(pose[0],stamp*.01,places=12)

    def test_out_of_order_arrival_is_sorted_without_changing_source_times(self):
        poses=self.poses()[::-1]
        paired=self.window(poses)
        self.assertEqual([row[0] for row in paired],sorted(row[0] for row in paired))
        np.testing.assert_allclose([row[0] for row in paired],np.arange(10,21)*.1,atol=1e-12)
        self.assertGreater(poses[0][0],poses[-1][0])

    def test_missing_either_endpoint_rejects_extrapolation(self):
        poses=self.poses()
        for begin,end in [(.85,1.975),(1.025,2.15)]:
            with self.subTest(begin=begin,end=end):
                with self.assertRaises(ValueError):self.window(poses,begin,end)
        with self.assertRaises(ValueError):self.window([])

    def test_missing_one_ten_hz_sample_rejects_interpolation_across_point_two_seconds(self):
        poses=[row for row in self.poses() if row[0]!=1.5]
        # Every requested time still has a before/after pair. Reject because
        # the pair spans a dropped LIO frame, rather than extrapolating it.
        with self.assertRaises(ValueError):self.window(poses)

    def test_gaps_outside_observation_window_do_not_block_valid_pairing(self):
        paired=self.window([sample(.1),*self.poses(),sample(5.)])
        np.testing.assert_allclose([row[0] for row in paired],np.arange(10,21)*.1,atol=1e-12)

    def test_duplicate_source_time_is_rejected(self):
        poses=self.poses();poses.append(sample(1.5))
        with self.assertRaises(ValueError):self.window(poses)

    def test_nonfinite_stamp_or_pose_is_rejected(self):
        for field in ('stamp','position','quaternion'):
            for value in (float('nan'),float('inf')):
                with self.subTest(field=field,value=value):
                    poses=self.poses();stamp,pose=sample(1.5)
                    if field=='stamp':stamp=value
                    elif field=='position':pose[0]=value
                    else:pose[3]=value
                    poses[6]=(stamp,pose)
                    with self.assertRaises(ValueError):self.window(poses)


class LocalizationSourceTests(unittest.TestCase):
    def ros_modules(self):
        rospy=types.ModuleType('rospy');rospy.Subscriber=mock.Mock()
        modules={'rospy':rospy}
        for package,names in [('sensor_msgs',['PointCloud2','Imu']),('nav_msgs',['Odometry']),
                              ('super_px4_bridge',['CoordinateTruth'])]:
            parent=types.ModuleType(package);messages=types.ModuleType(package+'.msg')
            for name in names:setattr(messages,name,type(name,(),{}))
            parent.msg=messages;modules[package]=parent;modules[package+'.msg']=messages
        return rospy,modules

    def test_lio_mode_subscribes_only_sensor_data_and_lio_pose(self):
        rospy,modules=self.ros_modules()
        with mock.patch.dict(sys.modules,modules):
            node=observation.TargetObservation({},Path('/tmp/unused_lio_observation_test'),body_source='lio')
            topics=[call.args[0] for call in rospy.Subscriber.call_args_list]
            self.assertCountEqual(topics,['/mid360s_sim/points','/mid360s_sim/imu','/lidar_slam/odom'])
            self.assertNotIn('/grasp_audit/truth',topics)
            node.close()

    def test_lio_poll_waits_for_a_new_nonoverlapping_window_without_spending_fit_budget(self):
        rospy,modules=self.ros_modules()
        def message(tenth):
            stamp=types.SimpleNamespace(to_sec=lambda:tenth/10.)
            return types.SimpleNamespace(header=types.SimpleNamespace(stamp=stamp))
        worker=mock.Mock();worker.is_alive.return_value=False
        with mock.patch.dict(sys.modules,modules),mock.patch('threading.Thread',return_value=worker) as threads:
            node=observation.TargetObservation({},Path('/tmp/unused_lio_observation_test'),body_source='lio')
            # Initially available scans cover 1.0..2.4; poll selects a full
            # older window and has LIO source poses bracketing its endpoints.
            node.clouds.extend(message(i) for i in range(10,25))
            node.body.extend(message(i) for i in range(9,61))
            node.poll(True)
            self.assertEqual(threads.call_count,1)
            self.assertEqual(node.attempts,0)
            node.poll(True)  # Identical data may not consume another retry.
            self.assertEqual(threads.call_count,1)
            node.clouds.extend(message(i) for i in range(25,33))
            node.poll(True)  # New scans still produce an overlapping window.
            self.assertEqual(threads.call_count,1)
            self.assertEqual(node.attempts,0)
            node.clouds.extend(message(i) for i in range(33,36))
            node.poll(True)  # A distinct complete window may be checked.
            self.assertEqual(threads.call_count,2)
            self.assertEqual(node.attempts,0)
            first=threads.call_args_list[0].kwargs['args'][0]
            second=threads.call_args_list[1].kwargs['args'][0]
            self.assertGreaterEqual(second[0].header.stamp.to_sec(),first[-1].header.stamp.to_sec()+.1)
            node.close()

    def test_default_mode_keeps_existing_simulated_body_pose_subscription(self):
        rospy,modules=self.ros_modules()
        with mock.patch.dict(sys.modules,modules):
            node=observation.TargetObservation({},Path('/tmp/unused_lio_observation_test'))
            topics=[call.args[0] for call in rospy.Subscriber.call_args_list]
            self.assertIn('/grasp_audit/truth',topics)
            self.assertNotIn('/lidar_slam/odom',topics)
            node.close()

    def test_misspelled_source_cannot_silently_fall_back_to_truth(self):
        rospy,modules=self.ros_modules()
        with mock.patch.dict(sys.modules,modules):
            with self.assertRaises(ValueError):
                observation.TargetObservation({},Path('/tmp/unused_lio_observation_test'),body_source='lio_typo')
        rospy.Subscriber.assert_not_called()


if __name__=='__main__':unittest.main()
