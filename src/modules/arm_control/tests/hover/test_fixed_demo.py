#!/usr/bin/env python3
"""Offline fixture geometry checks; not permission or physical grasp proof."""
import unittest
import math
import xml.etree.ElementTree as ET
from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import patch, Mock
from threading import RLock
from pathlib import Path
import numpy as np
from fixed_demo_plan import FixedDemoPlan, SUPPORT_HALF, SUPPORT_OFFSET_X, START, UNFOLDED, fk, configure_arm_hold, ARM_HOLD
from fixed_grasp_probe import pair_feedback, FixedGraspProbe, ObservationCursor, StabilityWindow, context_geometry,world_progress,lateral_center,may_trim,insertion_goal,compensation_speed,precision_motion_ready


class FixedDemoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan=FixedDemoPlan(Path(__file__).with_name('known_target_fixed.json'))

    def test_nominal_route_and_jaw_stroke(self):
        report=self.plan.preview()
        self.assertTrue(report['sampled_geometry_feasible'])
        self.assertGreater(report['minimum_sampled_slack_m'],.001)
        self.assertGreater(report['full_jaw_sweep_worst_pair']['distance'],.007)
        self.assertGreater(report['nominal_fixed_pad_axial_overlap_m'],.008)
        self.assertFalse(report['motion_permitted'])
        self.assertFalse(report['grasp_tested'])
        self.assertFalse(report['continuous_or_flight_proof'])
        self.assertGreater(report['unfold_worst_pair']['distance'],.05)
        self.assertGreater(report['open_route_min_pad_target_gap_m'],.002)
        self.assertEqual(len(self.plan.waypoints_world),2)

    def test_known_target_and_world_up_lift(self):
        np.testing.assert_allclose(self.plan.target.object[:3,3],[.6,0,1.5])
        np.testing.assert_allclose(self.plan.lift_world-self.plan.grasp_world,[0,0,.03])
        np.testing.assert_allclose(fk(self.plan.expected_q),[.26,.015,.36],atol=.0002)
        self.assertGreater(self.plan.target.hover[2,3],1.)
        self.assertLess(self.plan.target.hover[2,3],2.)
        goal=(np.linalg.inv(self.plan.arm)@np.r_[self.plan.lift_world,1])[:3]
        np.testing.assert_allclose(fk(self.plan.lift_samples[-1]),goal,atol=.0002)

    def test_reachable_grasp_but_unreachable_lift_rejected(self):
        with patch('fixed_demo_plan.ROUTE',[[.22,.015,.28]]):
            with self.assertRaisesRegex(ValueError,'lift IK rejected before launch'):
                FixedDemoPlan(Path(__file__).with_name('known_target_fixed.json'))

    def test_unfold_pad_sweep_cannot_use_contact_exception(self):
        with patch('fixed_demo_plan.ROUTE',[[.22,.015,.31]]):
            plan=FixedDemoPlan(Path(__file__).with_name('known_target_fixed.json'))
            self.assertFalse(plan.preview()['sampled_geometry_feasible'])
        t=NS(fixed_grasp_geometry_valid=True,fixed_grasp_forbidden_contact=False,
            fixed_grasp_clearance=.015,fixed_grasp_current_clearance=.024,approach_geometry_valid=True,
            arm_external_contact=True,arm_obstacle_clearance=.01)
        self.assertEqual(context_geometry(t,False),(False,.01))
        self.assertEqual(context_geometry(t,True),(True,.024))
        self.assertEqual(context_geometry(t,True,True),(True,.015))
        # An unsafe unrequested closure does not forbid safe OPEN approach.
        # Native close_entry_ready independently requires the full stroke.
        self.assertLess(t.fixed_grasp_clearance,.016)
        t.fixed_grasp_forbidden_contact=True
        self.assertFalse(context_geometry(t,True)[0])

    def test_unfold_recovers_inward(self):
        np.testing.assert_allclose(self.plan.samples[0],START)
        np.testing.assert_allclose(self.plan.samples[130],UNFOLDED)
        positions=[fk(q) for q in self.plan.samples[:131]]
        for previous,current in zip(positions,positions[1:]):
            if previous[2]<.02:self.assertGreaterEqual(current[2],previous[2]-1e-6)
            self.assertLessEqual(current[2],.4)
        for a,b in zip(self.plan.samples,self.plan.samples[1:]):
            self.assertLessEqual(float(max(abs(b-a))),.020001)

    def test_gravity_projection_and_friction(self):
        R=self.plan.target.object[:3,:3]
        gravity=R.T@np.array([0.,0.,-1.])
        projection=gravity[:2]*(.01/abs(gravity[2]))
        for closing_shift in (0.,-.003):
            footprint=projection+np.array([closing_shift-SUPPORT_OFFSET_X,0.])
            self.assertTrue(np.all(abs(footprint)<np.minimum(SUPPORT_HALF[:2],.01)))
        self.assertLess(np.linalg.norm(gravity[:2])/abs(gravity[2]),1.2)

    def test_integral_copy_keeps_pd_and_torque_caps(self):
        model=ET.parse(Path(__file__).resolve().parents[5]/
            'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/so101/so101.sdf').find('model')
        before=ET.tostring(model)
        copy=ET.fromstring(before);configure_arm_hold(copy)
        for name,cfg in ARM_HOLD.items():
            old=model.find("plugin[@name='%s_controller']"%name)
            new=copy.find("plugin[@name='%s_controller']"%name)
            for field in ('p','d','cmdMax','initialPosition'):
                self.assertEqual(old.findtext(field),new.findtext(field))
            self.assertGreater(float(new.findtext('i')),0)
            self.assertLess(float(new.findtext('iMax')),float(new.findtext('cmdMax')))
        self.assertEqual(ET.tostring(model),before)
        copy.find("plugin[@name='wrist_flex_controller']/cmdMax").text='.3'
        with self.assertRaisesRegex(ValueError,'torque cap'):configure_arm_hold(copy)

    def test_pair_specific_rules(self):
        rows=self.plan.geometry(self.plan.expected_q,.3)
        pads={'gripper_fixed_finger_collision','moving_jaw_collision'}
        self.assertFalse(any(r['collision'] in pads and r['obstacle']=='target' for r in rows))
        for r in rows:
            expected=.003 if r['collision'] in pads and r['obstacle']=='support' else .016
            self.assertEqual(r['required'],expected)

    def test_world_progress_is_slow_without_catchup_jump(self):
        a=np.array([1.,2.,3.]);goal=a+[0,0,.03]
        np.testing.assert_allclose(world_progress(a,goal,.1),a+[0,0,.001])
        np.testing.assert_allclose(world_progress(a,goal,10.),a+[0,0,.001])
        np.testing.assert_allclose(world_progress(a,goal,.1,False),a)
        np.testing.assert_allclose(world_progress(goal,goal,.1),goal)
        np.testing.assert_allclose(world_progress(a,goal,.1,True,.002),a+[0,0,.0002])
        np.testing.assert_allclose(world_progress(a,goal,10.,True,.002),a+[0,0,.0002])
        with self.assertRaises(ValueError):world_progress(a,goal,-.1)
        with self.assertRaises(ValueError):world_progress(a,goal,.1,True,.02)
        with self.assertRaises(ValueError):world_progress(a,[0,0,np.nan],.1)

    def test_duplicate_truth_cannot_consume_pending_request_ack(self):
        cursor=ObservationCursor()
        self.assertIsNone(cursor.take((1,1000),None))
        self.assertIsNone(cursor.source)
        self.assertTrue(cursor.take((1,1000),(1,'waiting')))
        # Status poller finishes START while the truth poller is on the SAME
        # frame. It must remain pending, not disappear behind its serial.
        self.assertIsNone(cursor.take((1,1000),(2,'accepted')))
        self.assertEqual(cursor.serial,1)
        self.assertTrue(cursor.take((2,21000),(2,'accepted')))
        self.assertEqual(cursor.serial,2)
        self.assertFalse(cursor.take((3,41000),(2,'accepted')))
        self.assertIsNone(cursor.take((3,41000),(3,'closing')))
        self.assertTrue(cursor.take((4,61000),(3,'closing')))

    def test_unfold_stability_dwell_requires_continuous_source_time(self):
        window=StabilityWindow()
        for i in range(50):self.assertFalse(window.update(1.+i*.02,True))
        self.assertTrue(window.update(2.,True))
        self.assertFalse(window.update(2.02,False))
        self.assertFalse(window.update(2.04,True))
        self.assertFalse(window.update(3.04,True)) # No backdating across a source gap.
        for i in range(50):self.assertFalse(window.update(3.06+i*.02,True))
        self.assertTrue(window.update(4.061,True))
        with self.assertRaises(ValueError):window.update(4.,True)
        with self.assertRaises(ValueError):window.update(float('nan'),True)

    def test_shared_status_preserves_mode_and_rejects_other_owners(self):
        for mode,locked,bench,rejected in ((0,False,False,False),(1,True,False,False),
                (0,True,False,True),(2,False,False,True),(1,True,True,True)):
            with self.subTest(mode=mode,locked=locked,bench=bench):
                probe=FixedGraspProbe.__new__(FixedGraspProbe)
                probe.stop_event=Mock();probe.stop_event.wait.side_effect=[False,True]
                probe.sources=NS(read=lambda:dict(now=1.))
                values=dict(arm_hover_status={},arm_approach_status=dict(mode=mode,ownership_locked=locked),
                    arm_control_status={},arm_grasp_status=dict(ownership_locked=bench))
                probe.status=Mock(side_effect=lambda topic:values[topic])
                probe.status_lock=RLock();probe.status_snapshot=None;probe.error=None
                probe.read_status()
                self.assertEqual(probe.error is not None,rejected)
                self.assertEqual(probe.stop_event.set.called,rejected)
                if not rejected:self.assertEqual(probe.status_snapshot[2]['mode'],mode)
                self.assertNotIn('arm_fixed_grasp_status',[c.args[0] for c in probe.status.call_args_list])

    def test_lateral_center_does_not_advance_insertion(self):
        R=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
        start=np.zeros(3);result=lateral_center(start,R,.002)
        np.testing.assert_allclose(result,[0,-.0001,0])
        self.assertAlmostEqual(float((result-start)@R[:,2]),0.)
        np.testing.assert_allclose(lateral_center(start,R,.0052),start)
        limited=lateral_center(R[:,0]*.0049,R,.02)
        self.assertLessEqual(np.linalg.norm(limited),.005000001)
        with self.assertRaises(ValueError):lateral_center(start,R,np.nan)

    def test_endpoint_trim_freezes_on_hover_pause_and_close(self):
        self.assertTrue(may_trim(1,False,False,True))
        self.assertTrue(may_trim(2,False,False,True))
        for phase in range(10):
            self.assertFalse(may_trim(phase,True,False,True))
            self.assertFalse(may_trim(phase,False,True,True))
            self.assertFalse(may_trim(phase,False,False,False))
        for phase in (0,3,4,5,6,7,8,9):
            self.assertFalse(may_trim(phase,False,False,True))

    def test_precision_admission_counts_body_rotation_and_keeps_motion_headroom(self):
        self.assertAlmostEqual(compensation_speed([0,0,0],[.01,0,0],[0,0,.1],[.3,0,0]),math.sqrt(.001))
        self.assertTrue(precision_motion_ready(.020,.003,.003,.08,0.))
        self.assertTrue(precision_motion_ready(.020,.003,.003,.08,.005))
        self.assertFalse(precision_motion_ready(.010,.001,.001,.01,.0051))
        for values in ((.021,.001,.001,.01),(.01,.0031,.001,.01),
                       (.01,.001,.0031,.01),(.01,.001,.001,.081),
                       (float('nan'),0,0,0),(-.001,0,0,0)):
            self.assertFalse(precision_motion_ready(*values,body_error=0.))
        # A hold freezes only task insertion; the world-fixed TCP remains
        # transformed into each live arm frame by the coordinator.
        start=np.array([1.,2.,3.]);goal=start+[0,0,.02]
        ready=precision_motion_ready(.021,0,0,0,0.)
        np.testing.assert_allclose(world_progress(start,goal,.1,ready,.002),start)
        with self.assertRaises(ValueError):compensation_speed([0,0],[0,0,0],[0,0,0],[0,0,0])

    def test_open_corridor_prevents_forward_side_collision(self):
        current=np.zeros(3);desired=np.array([.002,0.,-.03]);R=np.eye(3)
        result,gap,ready=insertion_goal(current,desired,R,[.005,0,-.12],R,.3,0)
        self.assertTrue(ready);self.assertAlmostEqual(gap,.003);np.testing.assert_allclose(result,desired)
        result,gap,ready=insertion_goal(current,desired,R,[.001,0,-.12],R,.3,0)
        self.assertFalse(ready);self.assertLess(gap,0);np.testing.assert_allclose(result,[.002,0,0])
        result,_,ready=insertion_goal(current,desired,R,[.005,0,-.12],R,.3,math.radians(6))
        self.assertFalse(ready);self.assertAlmostEqual(result[2],0)
        with self.assertRaises(ValueError):insertion_goal(current,desired,R,[.005,0,-.12],R,.1,0)
        # Rotated cube projection, not only center X, must clear the pad plane.
        angle=math.pi/4;rot=np.array([[math.cos(angle),0,math.sin(angle)],[0,1,0],[-math.sin(angle),0,math.cos(angle)]])
        _,gap,ready=insertion_goal(current,desired,R,[.005,0,-.12],rot,.3,0)
        self.assertFalse(ready);self.assertLess(gap,0)

    def test_source_time_pairing_rotating_gripper(self):
        def message(stamp,angle,epoch=1):
            def p(x,q):return NS(position=NS(x=x[0],y=x[1],z=x[2]),
                orientation=NS(w=q[0],x=q[1],y=q[2],z=q[3]))
            return NS(source_instance='test',world_epoch=epoch,vehicle_id=2,target_id=3,
                header=NS(stamp=NS(to_sec=lambda:stamp)),link_names=['so101::gripper_link'],
                link_poses=[p([0,0,0],[math.cos(angle/2),0,0,math.sin(angle/2)])],
                target_pose=p([1,0,0],[1,0,0,0]))
        history=[message(1.,0),message(1.02,.04)]
        local,skew=pair_feedback(history,1.01,('test',1,2,3))
        np.testing.assert_allclose(local,[math.cos(.02),-math.sin(.02),0],atol=1e-9)
        self.assertAlmostEqual(skew,.01)
        for stamp in (.99,1.03):
            with self.assertRaises(ValueError):pair_feedback(history,stamp,('test',1,2,3))
        with self.assertRaises(ValueError):pair_feedback(history,1.01,('test',2,2,3))
        with self.assertRaises(ValueError):pair_feedback(history+history,1.01,('test',1,2,3))
        probe=object.__new__(FixedGraspProbe)
        probe.sources=NS(truth_samples=lambda:history)
        probe.feedback=dict(source_sequence=1,capabilities=15,sample_time_us=1010000,
            object_center_gripper=local.tolist())
        probe.checked_feedback=None;probe.last_verified_stamp=-math.inf
        probe.pending_feedback=deque(maxlen=12);probe.seen_feedback=None
        row=dict(truth=history[-1],now=1.02)
        probe.verify_feedback_frame(row);self.assertTrue(probe.frame_verified)
        self.assertLess(probe.frame_error,1e-9)
        # Pending right bracket cannot renew the old checked evidence.
        probe.feedback=dict(probe.feedback,source_sequence=2,sample_time_us=1040000)
        row['now']=1.13;probe.verify_feedback_frame(row);self.assertTrue(probe.frame_verified)
        row['now']=1.161;probe.verify_feedback_frame(row);self.assertFalse(probe.frame_verified)
        probe.feedback=dict(probe.feedback,sample_time_us=1010000,
            source_sequence=3,object_center_gripper=(local+np.array([.004,0,0])).tolist())
        with self.assertRaisesRegex(RuntimeError,'frame mismatch'):probe.verify_feedback_frame(row)

        # A newer unbracketed sample must not replace/drop the older pending one.
        probe.pending_feedback=deque(maxlen=12);probe.seen_feedback=None
        history[:]=[message(1.,0),message(1.02,.04)]
        probe.feedback=dict(source_sequence=10,capabilities=15,sample_time_us=1040000,
            object_center_gripper=[math.cos(.08),-math.sin(.08),0])
        row['now']=1.04;probe.verify_feedback_frame(row)
        history.append(message(1.06,.12))
        probe.feedback=dict(probe.feedback,source_sequence=11,sample_time_us=1070000)
        row['now']=1.07;probe.verify_feedback_frame(row)
        self.assertEqual(probe.checked_feedback,10)
        self.assertTrue(probe.frame_verified)
        self.assertEqual(len(probe.pending_feedback),1)


if __name__=='__main__':unittest.main()
