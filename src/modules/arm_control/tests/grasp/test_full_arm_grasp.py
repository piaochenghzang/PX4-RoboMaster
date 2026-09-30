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
from mavlink_fault_proxy import FrameStream
from auto_grasp_case import exit_retention_checks


class FullArmChecks(unittest.TestCase):
    def test_fault_bridge_preserves_fragmented_frames_and_signatures(self):
        # Synthetic framing only, not synthetic contact evidence.
        frames=[bytes([0xfe,2,0,1,200,90])+b'ab'+b'CC',
                bytes([0xfd,3,0,0,1,1,200])+int(42002).to_bytes(3,'little')+b'xyz'+b'CC',
                bytes([0xfd,1,1,0,2,1,200])+int(42001).to_bytes(3,'little')+b'j'+b'CC'+b'S'*13]
        stream=FrameStream();received=[]
        for byte in b''.join(frames): received.extend(stream.feed(bytes([byte])))
        self.assertEqual(received,list(zip([90,42002,42001],frames)))
        self.assertFalse(stream.buffer)

    def test_fault_bridge_refuses_unframed_data(self):
        with self.assertRaises(RuntimeError): FrameStream().feed(b'not MAVLink')

    def test_grasp_feedback_is_explicitly_opt_in(self):
        expanded=('<sdf version="1.7"><model name="yhang550"><plugin name="mavlink_interface" filename="bridge.so"/>'
                  '<plugin name="gripper_controller" filename="servo.so"/>'
                  '<plugin name="wrist_controller" filename="servo.so"/></model></sdf>')
        with TemporaryDirectory() as name:
            for enabled in [False,True]:
                with patch('run_full_arm_grasp.sp.check_output',return_value=expanded):
                    build_world(Path(name),{},None,grasp_feedback=enabled)
                bridge=ET.parse(Path(name)/'scene.world').find('world/model/plugin')
                self.assertEqual(bridge.find('enableArmGraspFeedback') is not None,enabled)
                model=ET.parse(Path(name)/'scene.world').find('world/model')
                jaw=model.find("plugin[@name='gripper_controller']/commandTimeoutPolicy")
                self.assertEqual(jaw.text if jaw is not None else None,'last_target' if enabled else None)
                self.assertIsNone(model.find("plugin[@name='wrist_controller']/commandTimeoutPolicy"))

    def test_fault_exit_requires_force_and_no_unloading_or_drop(self):
        # Only verifies the checker, not physical grasp evidence.
        initial=dict(sim_s=0,cx=0,cy=0,cz=1,gx=0,gy=0,gz=1,gw=1,gqx=0,gqy=0,gqz=0)
        row=dict(initial,sim_s=30,contact_frames=100,bilateral_fraction=1,
                 cube_support_fraction=0,cube_other_fraction=0,support_arm_fraction=0,
                 fixed_normal_N=.15,moving_normal_N=.15,jaw_angle=.06,jaw_speed=0,jaw_torque_Nm=-.0095)
        self.assertTrue(exit_retention_checks([row],initial,cube_local,contact_summary)['retained'])
        for change in [dict(moving_normal_N=0),dict(cz=.05),dict(cx=.05),
                       dict(cube_support_fraction=.1),dict(jaw_torque_Nm=.02),dict(jaw_angle=float('nan')),
                       dict(moving_normal_N=float('nan'))]:
            self.assertFalse(exit_retention_checks([dict(row,**change)],initial,cube_local,contact_summary)['retained'])
        brief_loss=[dict(row,sim_s=i*.1,bilateral_fraction=0 if i<7 else 1) for i in range(100)]
        self.assertFalse(exit_retention_checks(brief_loss,initial,cube_local,contact_summary)['retained'])

    def test_contacts_are_weighted_by_received_physics_frames(self):
        values=[
            dict(contact_frames=1,bilateral_fraction=0,cube_support_fraction=1),
            dict(contact_frames=99,bilateral_fraction=1,cube_support_fraction=0),
        ]
        summary=contact_summary(values)
        self.assertAlmostEqual(summary['bilateral_fraction'],.99)
        self.assertAlmostEqual(summary['cube_support_fraction'],.01)

    def test_gui_camera_does_not_change_physics_or_add_actuation(self):
        expanded='<sdf version="1.7"><model name="yhang550"/></sdf>'
        with TemporaryDirectory() as name:
            with patch('run_full_arm_grasp.sp.check_output',return_value=expanded):
                build_world(Path(name),{},None,gui=True)
            world=ET.parse(Path(name)/'scene.world').find('world')
            self.assertEqual(world.find('gui/camera/pose').text,'1.4 -1.5 1.5 0 0.193 2.322')
            self.assertEqual(world.find('physics/max_step_size').text,'0.001')
            self.assertIsNotNone(world.find("model[@name='yhang550']/joint[@name='bench_anchor']"))

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
