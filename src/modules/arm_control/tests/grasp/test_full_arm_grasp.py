#!/usr/bin/env python3
"""Pure checks for the full-arm experiment; no simulation or command output."""
import unittest
import json
from unittest.mock import patch
from pathlib import Path
from tempfile import TemporaryDirectory
import xml.etree.ElementTree as ET
import numpy as np
from run_full_arm_grasp import box_bounds,build_world,contact_summary,cube_local,grasp_reference,rotation,retention_checks,set_pad_patch


class FullArmChecks(unittest.TestCase):
    def test_contacts_are_weighted_by_received_physics_frames(self):
        values=[
            dict(contact_frames=1,bilateral_fraction=0,cube_support_fraction=1),
            dict(contact_frames=99,bilateral_fraction=1,cube_support_fraction=0),
        ]
        summary=contact_summary(values)
        self.assertAlmostEqual(summary['bilateral_fraction'],.99)
        self.assertAlmostEqual(summary['cube_support_fraction'],.01)

    def test_empty_contact_window_cannot_verify_contact(self):
        self.assertEqual(contact_summary([])['bilateral_fraction'],0)

    def test_small_cube_reference_reaches_fixed_finger(self):
        self.assertAlmostEqual(grasp_reference(20)[0],.0025)
        self.assertAlmostEqual(grasp_reference(16)[0]-.008,-.008)

    def test_relative_object_position_handles_world_rotation(self):
        quat=[np.sqrt(.5),0,0,np.sqrt(.5)]
        position=np.array([.4,-.2,1.])
        local=np.array([.003,.001,-.088])
        world=position+rotation(quat)@local
        row=dict(zip(['gx','gy','gz','gw','gqx','gqy','gqz','cx','cy','cz'],
                     [*position,*quat,*world]))
        np.testing.assert_allclose(cube_local(row),local,atol=1e-12)

    def test_invalid_quaternion_is_not_a_valid_pose(self):
        for q in [[0,0,0,0],[float('nan'),0,0,1]]:
            with self.assertRaises(RuntimeError):
                rotation(q)

    def test_body_envelope_uses_rotation_and_local_offset(self):
        R=rotation([np.sqrt(.5),0,0,np.sqrt(.5)])
        low,high=box_bounds([1,2,3],R,[.1,0,0],[.01,.02,.03])
        np.testing.assert_allclose(low,[.98,2.09,2.97],atol=1e-12)
        np.testing.assert_allclose(high,[1.02,2.11,3.03],atol=1e-12)

    def test_demo_records_slip_without_requiring_precision(self):
        contact=dict(bilateral_fraction=.99,cube_support_fraction=0,
                     fixed_normal_N=.10,moving_normal_N=.10)
        self.assertEqual(retention_checks(contact,.008,.009,.027),(True,False))
        self.assertEqual(retention_checks(contact,.002,.003,.027),(True,True))

    def test_supported_or_unheld_object_cannot_pass_demo(self):
        contact=dict(bilateral_fraction=.99,cube_support_fraction=.1,
                     fixed_normal_N=.10,moving_normal_N=.10)
        self.assertEqual(retention_checks(contact,0,0,.03),(False,False))

        contact.update(cube_support_fraction=0,moving_normal_N=0)
        self.assertEqual(retention_checks(contact,0,0,.03),(False,False))
        contact.update(moving_normal_N=.10,bilateral_fraction=.2)
        self.assertEqual(retention_checks(contact,0,0,.03),(False,False))
        contact.update(bilateral_fraction=1)
        self.assertEqual(retention_checks(contact,0,0,-.001),(False,False))
        contact.update(moving_normal_N=float('nan'))
        self.assertEqual(retention_checks(contact,0,0,.03),(False,False))

    def test_numpy_retention_results_can_be_saved_as_json(self):
        contact=dict(bilateral_fraction=.99,cube_support_fraction=0,
                     fixed_normal_N=.10,moving_normal_N=.10)
        retained,strict=retention_checks(contact,np.float64(.002),np.float64(.003),np.float64(.027))
        self.assertIs(type(retained),bool)
        self.assertIs(type(strict),bool)
        self.assertEqual(json.loads(json.dumps(dict(success=retained,strict=strict))),
                         dict(success=True,strict=True))

    def test_bench_resize_updates_inertia_and_keeps_support_below_cube(self):
        # XML generation only; no simulator is launched or physical outcome mocked.
        calibration=dict(cube_world_position=[.18,0,1.],
                         gripper_world_quaternion=[1.,0,0,0])
        expanded='<sdf version="1.7"><model name="yhang550"/></sdf>'
        with TemporaryDirectory() as name:
            with patch('run_full_arm_grasp.sp.check_output',return_value=expanded):
                build_world(Path(name),{},calibration,cube_mm=16)
            tree=ET.parse(Path(name)/'scene.world')
            cube=tree.find("world/model[@name='test_cube']")
            box=cube.find('link/collision/geometry/box/size')
            self.assertEqual([float(x) for x in box.text.split()],[.016]*3)
            self.assertAlmostEqual(float(cube.find('link/inertial/mass').text),.01)
            self.assertAlmostEqual(float(cube.find('link/inertial/inertia/ixx').text),.01*.016**2/6)
            support=tree.find("world/model[@name='cube_support']/pose")
            self.assertAlmostEqual(float(support.text.split()[2]),.990)

    def test_p_override_does_not_raise_torque_limit(self):
        expanded=('<sdf version="1.7"><model name="yhang550">'
                  '<plugin name="gripper_controller" filename="libso101_joint_position_controller.so">'
                  '<p>0.03</p><cmdMax>0.01</cmdMax></plugin></model></sdf>')
        with TemporaryDirectory() as name:
            with patch('run_full_arm_grasp.sp.check_output',return_value=expanded):
                build_world(Path(name),{},None,gripper_p=.045)
            controller=ET.parse(Path(name)/'scene.world').find('world/model/plugin')
            self.assertAlmostEqual(float(controller.find('p').text),.045)
            self.assertAlmostEqual(float(controller.find('cmdMax').text),.01)

    def test_finite_pad_patch_preserves_linear_friction(self):
        pad=ET.fromstring('<collision><surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface></collision>')
        set_pad_patch(pad,4)
        friction=pad.find('surface/friction')
        self.assertAlmostEqual(float(friction.find('ode/mu').text),1.2)
        self.assertAlmostEqual(float(friction.find('ode/mu2').text),1.2)
        self.assertAlmostEqual(float(friction.find('torsional/coefficient').text),.8)
        self.assertAlmostEqual(float(friction.find('torsional/patch_radius').text),.004)
        self.assertEqual(friction.find('torsional/use_patch_radius').text,'true')
        set_pad_patch(pad,3)
        self.assertEqual(len(friction.findall('torsional')),1)

    def test_solver_override_is_only_in_generated_world(self):
        expanded='<sdf version="1.7"><model name="yhang550"/></sdf>'
        with TemporaryDirectory() as name:
            with patch('run_full_arm_grasp.sp.check_output',return_value=expanded):
                build_world(Path(name),{},None,solver='world')
            solver=ET.parse(Path(name)/'scene.world').find('world/physics/ode/solver/type')
            self.assertEqual(solver.text,'world')


if __name__=='__main__':
    unittest.main()
