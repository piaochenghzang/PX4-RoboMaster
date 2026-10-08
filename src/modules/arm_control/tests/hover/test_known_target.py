#!/usr/bin/env python3
"""Known input/frame regression only; no motion or control transport."""
import copy
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from known_target import KnownTarget, pose, inverse, mount_transform, quintic_segment, local_motion


class KnownTargetTests(unittest.TestCase):
    def setUp(self):
        self.config = json.loads(Path(__file__).with_name('known_target.json').read_text())

    def test_hover_is_offset_from_object_not_object_center(self):
        target = KnownTarget(self.config); report = target.preview()
        np.testing.assert_allclose(target.hover[:3,3], [.416721237, .005129812, 1.5], atol=2e-7)
        self.assertGreater(np.linalg.norm(target.hover[:3,3]-target.object[:3,3]), .1)
        np.testing.assert_allclose(target.nominal_object_arm[:3,3], target.reference, atol=1e-12)
        for key in ('close_permitted','flight_ready','mavros_goal_generated','ik_reachability_tested',
                    'flight_commands_sent','joint_commands_sent'):
            self.assertFalse(report[key])

    def test_yaw_changes_offset_with_full_rotation(self):
        config = copy.deepcopy(self.config); config['body_hover_yaw_world_rad'] = math.pi/2
        target = KnownTarget(config)
        np.testing.assert_allclose(target.hover[:3,3], [.594870188,-.183278763,1.5], atol=2e-7)
        np.testing.assert_allclose(target.nominal_object_arm[:3,3], target.reference, atol=1e-12)

    def test_actual_arm_pose_used_not_assumed_mount(self):
        target = KnownTarget(self.config)
        body = [.2,-.3,.9,math.cos(.4),0,0,math.sin(.4)]
        arm = [.22,-.31,.84,math.cos(.3),math.sin(.3),0,0]
        snapshot = dict(frame='gazebo_world',sample_time_s=12.5,source_instance='recorded',
            body_pose_world=body,arm_pose_world=arm)
        report = target.preview(snapshot)
        expected = inverse(pose(arm[:3],arm[3:])) @ target.object
        np.testing.assert_allclose(report['known_target_pose_in_recorded_arm'], expected, atol=1e-12)
        self.assertEqual(report['recorded_sample_time_s'],12.5)
        self.assertFalse(report['snapshot_runtime_freshness_checked'])
        self.assertFalse(report['known_target_actually_observed'])

    def test_random_full_rigid_snapshot_roundtrip(self):
        target = KnownTarget(self.config); rng = np.random.default_rng(101)
        for _ in range(30):
            q = rng.normal(size=4); q /= np.linalg.norm(q); p = rng.uniform(-2,2,3)
            snap = dict(frame='gazebo_world',sample_time_s=10,source_instance='synthetic_math_only',
                body_pose_world=[*p,*q],arm_pose_world=[*p,*q])
            report = target.preview(snap); arm = pose(p,q)
            np.testing.assert_allclose(arm @ np.asarray(report['known_target_pose_in_recorded_arm']),target.object,atol=1e-12)

    def test_mount_agrees_with_production_sdf(self):
        root = Path(__file__).resolve().parents[5]
        sdf = root/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/yhang550/yhang550.sdf'
        model = ET.parse(sdf).find('model')
        include = next(e for e in model.findall('include') if e.findtext('name')=='so101')
        nominal = [float(x) for x in include.findtext('pose').split()]
        np.testing.assert_allclose(mount_transform(nominal),KnownTarget(self.config).mount,atol=1e-12)

    def test_world_map_ambiguity_rejected(self):
        for frame in ('world','map','local_enu',''):
            config = copy.deepcopy(self.config); config['frame'] = frame
            with self.assertRaises(ValueError): KnownTarget(config)

    def test_nonfinite_wrong_shape_quaternion_rejected(self):
        changes = [('object_position_world_m',[0,0,float('nan')]),
            ('object_position_world_m',[0,0]),('object_quaternion_world_wxyz',[0,0,0,0]),
            ('object_quaternion_world_wxyz',[2,0,0,0]),('arm_mount_xyz_rpy',[0]*5),
            ('nominal_object_center_arm_m',[0,float('inf'),.1]),
            ('body_hover_yaw_world_rad',float('nan')),('target_name','')]
        for key,value in changes:
            config = copy.deepcopy(self.config); config[key] = value
            with self.assertRaises((ValueError,TypeError)): KnownTarget(config)

    def test_bad_snapshot_identity_frame_time_rejected(self):
        target = KnownTarget(self.config)
        snapshot = dict(frame='gazebo_world',sample_time_s=1,source_instance='recorded',
            body_pose_world=[0,0,1,1,0,0,0],arm_pose_world=[0,0,.95,1,0,0,0])
        for key,value in [('frame','map'),('source_instance',''),('sample_time_s',0),
                          ('sample_time_s',float('nan')),('arm_pose_world',[0]*7)]:
            changed = dict(snapshot); changed[key] = value
            with self.assertRaises(ValueError): target.preview(changed)

    def test_preview_imports_no_control_transport(self):
        import ast
        tree = ast.parse(Path(__file__).with_name('known_target.py').read_text())
        imported = [n.name for node in ast.walk(tree) if isinstance(node,ast.Import) for n in node.names]
        imported += [node.module for node in ast.walk(tree) if isinstance(node,ast.ImportFrom)]
        self.assertFalse(set(imported)&{'rospy','subprocess','socket','mavros','pymavlink'})

    def test_smooth_trajectory_endpoints_and_zero_velocity(self):
        for t,expected in [(-1,[0,0,1]),(0,[0,0,1]),(6,[.4,0,1.5]),(7,[.4,0,1.5])]:
            p,v=quintic_segment([0,0,1],[.4,0,1.5],t,6)
            np.testing.assert_allclose(p,expected,atol=1e-12)
            np.testing.assert_allclose(v,[0,0,0],atol=1e-12)
        p,v=quintic_segment([0,0,1],[.4,0,1.5],3,6)
        np.testing.assert_allclose(p,[.2,0,1.25]); self.assertGreater(v[0],0)

    def test_live_local_conversion_preserves_body_relative_goal_and_velocity(self):
        body_world=[1,2,.5,1,0,0,0]
        body_local=[0,0,0,math.sqrt(.5),0,0,math.sqrt(.5)]
        p,v,yaw=local_motion([1.4,2,1.5],[.1,0,0],0,body_world,body_local,
                             [.02,0,0],[.05,0,0])
        np.testing.assert_allclose(p,[0,.4,1],atol=1e-12)
        np.testing.assert_allclose(v,[0,.13,0],atol=1e-12)
        self.assertAlmostEqual(yaw,math.pi/2)


if __name__ == '__main__':
    unittest.main()
