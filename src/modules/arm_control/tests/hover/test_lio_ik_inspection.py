"""Coordinate conversion and local 16 mm geometry inspection; no simulator."""
import copy
import math
import unittest
import numpy as np
from lio_ik_inspection import tcp_goal_from_observation,local_geometry_check


def rigid(position=(0.,0.,0.),yaw=0.):
    c,s=math.cos(yaw),math.sin(yaw)
    T=np.eye(4);T[:3,:3]=[[c,-s,0],[s,c,0],[0,0,1]];T[:3,3]=position
    return T


class ToolGoalTests(unittest.TestCase):
    def fixture(self):
        row=dict(frame='map',map_from_target=rigid([2.,3.,4.],math.pi/2),
            map_from_arm=rigid([1.,1.,1.],math.pi/2))
        config=dict(tool_pose=[.4,0.,.2,0.,math.pi,0.])
        return row,config,np.array([.1,.2,.3])

    def test_hand_computed_rotated_reference_and_tool_offset(self):
        row,cfg,reference=self.fixture();result=tcp_goal_from_observation(row,cfg,reference)
        # A quarter turn maps the reference (.1,.2,.3) to (-.2,.1,.3).
        # Thus gripper translation=(2.2,2.9,3.7), then its tool offset adds
        # (0,.4,.2). The quarter-turned arm expresses the TCP as (2.3,-1.2,2.9).
        np.testing.assert_allclose(np.array(result['map_from_desired_gripper'])[:3,3],[2.2,2.9,3.7],atol=1e-12)
        np.testing.assert_allclose(np.array(result['map_from_desired_tcp'])[:3,3],[2.2,3.3,3.9],atol=1e-12)
        np.testing.assert_allclose(np.array(result['arm_from_desired_tcp'])[:3,3],[2.3,-1.2,2.9],atol=1e-12)
        self.assertLess(result['reference_roundtrip_error_m'],1e-12)

    def test_object_center_is_not_implicitly_used_as_native_tcp(self):
        row,cfg,reference=self.fixture();result=tcp_goal_from_observation(row,cfg,reference)
        desired=np.array(result['map_from_desired_tcp'])
        self.assertGreater(np.linalg.norm(desired[:3,3]-row['map_from_target'][:3,3]),.3)
        # Native TCP rotation includes its configured half turn, rather than
        # silently adopting the object/gripper orientation.
        np.testing.assert_allclose(desired[:3,:3],[[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]],atol=1e-12)

    def test_common_map_change_preserves_arm_input_and_source_data(self):
        row,cfg,reference=self.fixture();before=copy.deepcopy(row)
        original=tcp_goal_from_observation(row,cfg,reference)
        H=rigid([-5.,2.,.7],-.73)
        shifted=dict(frame='map',map_from_target=H@row['map_from_target'],map_from_arm=H@row['map_from_arm'])
        transformed=tcp_goal_from_observation(shifted,cfg,reference)
        np.testing.assert_allclose(transformed['arm_from_desired_tcp'],original['arm_from_desired_tcp'],atol=1e-12)
        for key in ('map_from_target','map_from_arm'):np.testing.assert_array_equal(row[key],before[key])

    def test_wrong_frame_invalid_rigid_matrices_and_reference_are_rejected(self):
        row,cfg,reference=self.fixture()
        wrong=dict(row,frame='gazebo_world')
        with self.assertRaises(ValueError):tcp_goal_from_observation(wrong,cfg,reference)
        invalid=[]
        for key in ('map_from_target','map_from_arm'):
            for value in (np.eye(3),np.diag([1.,1.,-1.,1.]),np.diag([2.,1.,1.,1.])):
                invalid.append(dict(row,**{key:value}))
            T=rigid();T[0,3]=float('nan');invalid.append(dict(row,**{key:T}))
        for bad in invalid:
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):tcp_goal_from_observation(bad,cfg,reference)
        for bad in ([.1,.2],[.1,float('nan'),.3],[.1,.2,float('inf')]):
            with self.assertRaises(ValueError):tcp_goal_from_observation(row,cfg,bad)


class LocalGeometryTests(unittest.TestCase):
    def geometry(self):
        return dict(target=dict(parts=[dict(name='body',size_m=[.1,.1,.1],xyz_rpy_deg=[0.,0.,0.,0.,0.,0.])],
            support_size_m=[.1,.1,.1],object_from_support_xyz_rpy_deg=[0.,0.,-.2,0.,0.,0.]))

    def check(self,position):
        return local_geometry_check([('test_arm_box',rigid(position),np.array([.05]*3))],rigid(),self.geometry())

    def test_far_geometry_passes_without_granting_motion_or_close(self):
        result=self.check([1.,0.,0.])
        self.assertTrue(result['local_clearance_valid'])
        self.assertAlmostEqual(result['min_lower_bound_m'],.9,places=12)
        self.assertIsNotNone(result['minimum_pair'])
        self.assertFalse(result['motion_permitted']);self.assertFalse(result['closing_permitted'])

    def test_concentric_boxes_are_rejected(self):
        result=self.check([0.,0.,0.])
        self.assertFalse(result['local_clearance_valid'])
        self.assertEqual(result['min_lower_bound_m'],0.)
        self.assertFalse(result['motion_permitted']);self.assertFalse(result['closing_permitted'])

    def test_sixteen_mm_threshold_is_not_relaxed_for_a_finger(self):
        geometry=self.geometry();object_map=rigid()
        for gap,expected in [(.0159,False),(.0161,True)]:
            for name in ('gripper_fixed_finger_collision','moving_jaw_collision','wrist_collision'):
                with self.subTest(gap=gap,name=name):
                    boxes=[(name,rigid([.1+gap,0.,0.]),np.array([.05]*3))]
                    result=local_geometry_check(boxes,object_map,geometry)
                    self.assertEqual(result['local_clearance_valid'],expected)
                    self.assertAlmostEqual(result['min_lower_bound_m'],gap,places=12)
                    self.assertFalse(result['motion_permitted']);self.assertFalse(result['closing_permitted'])

    def test_support_is_checked_even_when_the_object_body_is_far(self):
        # Body is 0.2 m above the coincident arm/support boxes.
        result=self.check([0.,0.,-.2])
        self.assertFalse(result['local_clearance_valid'])
        self.assertEqual(result['min_lower_bound_m'],0.)

    def test_common_map_change_preserves_clearance(self):
        geometry=self.geometry();O=rigid();B=rigid([.1161,0.,0.])
        original=local_geometry_check([('arm_box',B,np.array([.05]*3))],O,geometry)
        H=rigid([3.,-2.,.5],.47)
        transformed=local_geometry_check([('arm_box',H@B,np.array([.05]*3))],H@O,geometry)
        self.assertEqual(transformed['local_clearance_valid'],original['local_clearance_valid'])
        self.assertAlmostEqual(transformed['min_lower_bound_m'],original['min_lower_bound_m'],places=12)


if __name__=='__main__':unittest.main()
