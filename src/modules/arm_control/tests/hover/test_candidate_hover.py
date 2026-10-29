#!/usr/bin/env python3
"""Pure candidate hover geometry: one level body pose, two distinct TCP goals."""
import copy
import math
import unittest
import numpy as np
from candidate_hover import derive_hover


def configuration():
    return dict(mount_pose=[.2,-.1,.3,0,0,math.pi/2],
        joint_origins=[[.4,0,.1,0,0,0]]+[[0.]*6 for _ in range(4)],
        tool_pose=[-.03,.04,-.08,0,math.pi,0],reference_gripper=[.1,-.02,-.05])


def target():
    T=np.diag([-1.,-1.,1.,1.]);T[:3,3]=[2.,3.,1.5]
    return T


class CandidateHoverTests(unittest.TestCase):
    def test_nonzero_mount_and_quarter_turn_have_hand_calculated_body_pose(self):
        out=derive_hover(target(),configuration(),np.zeros(6))
        body=np.array(out['map_from_body_hover']);arm=np.array(out['map_from_arm'])
        np.testing.assert_allclose(body[:3,:3],[[0,-1,0],[1,0,0],[0,0,1]],atol=1e-12)
        np.testing.assert_allclose(body[:3,3],[2.4,2.78,1.15],atol=1e-12)
        np.testing.assert_allclose(arm[:3,3],[2.5,2.98,1.45],atol=1e-12)
        np.testing.assert_allclose(arm[:3,:3],np.diag([-1,-1,1]),atol=1e-12)
        np.testing.assert_allclose(np.array(out['final_goal']['arm_from_desired_tcp'])[:3,3],[.37,.04,.02],atol=1e-12)
        self.assertAlmostEqual(out['yaw_rad'],math.pi/2,places=12)
        self.assertLess(out['template_orientation_error_deg'],1e-5)

    def test_known_template_grasp_reference_lands_on_observed_target(self):
        cfg=configuration();T=target();out=derive_hover(T,cfg,np.zeros(6))
        # For this fixture B_G=[Rz(90),(.2,.3,.4)], obtained by hand
        # from the mount and the first .4m link, with all joint angles zero.
        B_G=np.array([[0.,-1.,0.,.2],[1.,0.,0.,.3],[0.,0.,1.,.4],[0.,0.,0.,1.]])
        M_G=np.array(out['map_from_body_hover'])@B_G
        np.testing.assert_allclose((M_G@np.r_[cfg['reference_gripper'],1])[:3],T[:3,3],atol=1e-12)
        np.testing.assert_allclose(M_G,out['final_goal']['map_from_desired_gripper'],atol=1e-12)

    def test_preapproach_changes_tcp_goal_without_moving_candidate_body_again(self):
        cfg=configuration();T=target();out=derive_hover(T,cfg,np.zeros(6),pre_distance=.04)
        final=np.array(out['final_goal']['map_from_desired_tcp'])
        pre=np.array(out['pre_goal']['map_from_desired_tcp'])
        np.testing.assert_allclose(pre[:3,3]-final[:3,3],T[:3,:3]@np.array([0.,0.,.04]),atol=1e-12)
        np.testing.assert_allclose(pre[:3,:3],final[:3,:3],atol=1e-12)
        arm=np.array(out['map_from_arm'])
        np.testing.assert_allclose(arm@np.array(out['final_goal']['arm_from_desired_tcp']),final,atol=1e-12)
        np.testing.assert_allclose(arm@np.array(out['pre_goal']['arm_from_desired_tcp']),pre,atol=1e-12)
        # This independent hand calculation detects adding the standoff to the
        # body pose, which would displace the actual final target as well.
        np.testing.assert_allclose(np.array(out['map_from_body_hover'])[:3,3],[2.4,2.78,1.15],atol=1e-12)
        self.assertEqual(out['pre_distance_m'],.04)

    def test_tilted_object_keeps_gravity_aligned_body_and_reports_direction_mismatch(self):
        T=np.eye(4);angle=math.radians(12);c,s=math.cos(angle),math.sin(angle)
        T[:3,:3]=[[c,0,s],[0,1,0],[-s,0,c]];T[:3,3]=[2.,3.,1.5]
        out=derive_hover(T,configuration(),np.zeros(6))
        body=np.array(out['map_from_body_hover'])
        np.testing.assert_allclose(body[:3,2],[0,0,1],atol=1e-12)
        np.testing.assert_allclose(body[2,:3],[0,0,1],atol=1e-12)
        self.assertGreater(out['template_orientation_error_deg'],5.)
        self.assertAlmostEqual(out['template_orientation_error_deg'],12.,places=9)

    def test_horizontal_map_yaw_and_translation_transform_candidate_equivariantly(self):
        T=target();cfg=configuration();q=np.zeros(6)
        angle=.6;c,s=math.cos(angle),math.sin(angle)
        change=np.array([[c,-s,0.,4.],[s,c,0.,-2.],[0.,0.,1.,.7],[0.,0.,0.,1.]])
        first=derive_hover(T,cfg,q);second=derive_hover(change@T,cfg,q)
        for key in ['map_from_body_hover','map_from_arm','map_from_target']:
            np.testing.assert_allclose(second[key],change@np.array(first[key]),atol=1e-12,err_msg=key)
        for key in ['final_goal','pre_goal']:
            np.testing.assert_allclose(second[key]['arm_from_desired_tcp'],first[key]['arm_from_desired_tcp'],atol=1e-12)
        self.assertAlmostEqual(second['template_orientation_error_deg'],first['template_orientation_error_deg'],places=5)

    def test_inputs_are_unchanged_and_target_output_does_not_alias_input(self):
        T=target();cfg=configuration();q=np.zeros(6)
        saved=copy.deepcopy((T,cfg,q));out=derive_hover(T,cfg,q)
        np.testing.assert_array_equal(T,saved[0]);self.assertEqual(cfg,saved[1]);np.testing.assert_array_equal(q,saved[2])
        before=np.array(out['map_from_target']).copy();T[0,3]+=9
        np.testing.assert_array_equal(out['map_from_target'],before)

    def test_invalid_target_transform_reference_and_joint_shape_are_rejected(self):
        invalid=[]
        for index,value in [((0,0),2.),((0,1),.1),((2,2),-1.),((3,1),.1),((0,3),float('nan'))]:
            T=target();T[index]=value;invalid.append(T)
        invalid.append(np.eye(3))
        for T in invalid:
            with self.subTest(transform=T),self.assertRaises(ValueError):derive_hover(T,configuration(),np.zeros(6))
        for reference in [[.1,0],[.1,0,float('inf')]]:
            cfg=configuration();cfg['reference_gripper']=reference
            with self.subTest(reference=reference),self.assertRaises(ValueError):derive_hover(target(),cfg,np.zeros(6))
        for q in [np.zeros(5),np.array([0,0,0,float('nan'),0,0])]:
            with self.subTest(q=q),self.assertRaises(ValueError):derive_hover(target(),configuration(),q)

    def test_invalid_preapproach_distance_is_rejected(self):
        for distance in [-.01,0.,float('nan'),float('inf')]:
            with self.subTest(distance=distance),self.assertRaises(ValueError):
                derive_hover(target(),configuration(),np.zeros(6),pre_distance=distance)


if __name__=='__main__':unittest.main()
