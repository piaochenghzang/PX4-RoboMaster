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
from fixed_demo_plan import FixedDemoPlan, SUPPORT_HALF, SUPPORT_OFFSET_X, START, UNFOLDED, fk, configure_arm_hold, ARM_HOLD, GRIPPER_P, OPEN_REFERENCE
from fixed_grasp_probe import pair_feedback, FixedGraspProbe, ObservationCursor, StabilityWindow, context_geometry,world_progress,lateral_center,may_trim,insertion_goal,compensation_speed,precision_motion_ready,unfold_motion_ready,waypoint_dwell_seconds,centered_world_target
from run_known_target_flight import DEMO_OBSTACLES, obstacle_clearance, line_deviation


class FixedDemoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan=FixedDemoPlan(Path(__file__).with_name('known_target_fixed.json'))

    def test_supported_closure_stays_world_fixed_until_verified_lift(self):
        from fixed_grasp_probe import grasp_body_hold
        for phase in (5,6,7):self.assertTrue(grasp_body_hold(phase))
        for phase in (0,1,2,3,4,8,9):self.assertFalse(grasp_body_hold(phase))
        # Contact retries keep the supported TCP in exactly the same frame.
        self.assertEqual([grasp_body_hold(p) for p in (3,4,3,4)], [False]*4)

    def test_close_goal_removes_live_entry_residual_in_world_frame(self):
        R=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        tool=np.array([5.,0.,1.5]);local=OPEN_REFERENCE+[.001,.002,-.001]
        goal=centered_world_target(tool,R,local)
        # Translating the gripper to the goal must leave the cube at the
        # intended reference, including when its axes differ from world.
        np.testing.assert_allclose(local-R.T@(goal-tool),OPEN_REFERENCE,atol=1e-12)
        self.assertLess(float(np.linalg.norm(goal-tool)),.003)
        np.testing.assert_allclose(centered_world_target(tool,R,OPEN_REFERENCE),tool)

    def test_nominal_route_and_jaw_stroke(self):
        report=self.plan.preview()
        self.assertTrue(report['sampled_geometry_feasible'])
        self.assertGreater(report['minimum_sampled_slack_m'],.001)
        self.assertGreater(report['full_jaw_sweep_worst_pair']['distance'],.007)
        self.assertGreaterEqual(report['nominal_fixed_pad_axial_overlap_m'],.007-1e-9)
        self.assertFalse(report['motion_permitted'])
        self.assertFalse(report['grasp_tested'])
        self.assertFalse(report['continuous_or_flight_proof'])
        self.assertGreater(report['unfold_worst_pair']['distance'],.05)
        self.assertGreater(report['open_route_min_pad_target_gap_m'],.002)
        self.assertEqual(report['gripper_p'],GRIPPER_P)
        self.assertEqual(report['gripper_torque_limit_nm'],.01)
        self.assertEqual(len(self.plan.waypoints_world),3)
        self.assertAlmostEqual(float(np.linalg.norm(self.plan.waypoints_world[-1]-self.plan.waypoints_world[-2])),.004,places=8)

    def test_known_target_and_world_up_lift(self):
        np.testing.assert_allclose(self.plan.target.object[:3,3],[.6,0,1.5])
        np.testing.assert_allclose(self.plan.lift_world-self.plan.grasp_world,[0,0,.04])
        np.testing.assert_allclose(fk(self.plan.expected_q),[.16,.015,.39],atol=.0002)
        self.assertGreater(self.plan.target.hover[2,3],1.)
        self.assertLess(self.plan.target.hover[2,3],2.)
        goal=(np.linalg.inv(self.plan.arm)@np.r_[self.plan.lift_world,1])[:3]
        np.testing.assert_allclose(fk(self.plan.lift_samples[-1]),goal,atol=.0002)

    def test_obstacle_demo_translates_the_proven_fixture_only(self):
        obstacle_plan=FixedDemoPlan(Path(__file__).with_name('known_target_obstacle.json'))
        np.testing.assert_allclose(obstacle_plan.target.object[:3,3],[5.,0.,1.5])
        np.testing.assert_allclose(obstacle_plan.target.object[:3,:3],self.plan.target.object[:3,:3],atol=1e-8)
        np.testing.assert_allclose(obstacle_plan.expected_q,self.plan.expected_q,atol=1e-8)
        np.testing.assert_allclose(obstacle_plan.target.hover[:3,3]-self.plan.target.hover[:3,3],[4.4,0.,0.])
        for shifted,original in zip(obstacle_plan.waypoints_world,self.plan.waypoints_world):
            np.testing.assert_allclose(shifted-original,[4.4,0.,0.],atol=1e-8)
        self.assertTrue(obstacle_plan.preview()['sampled_geometry_feasible'])

    def test_obstacle_course_blocks_the_direct_path_and_leaves_target_clear(self):
        start=np.array([0.,0.,1.5]);goal=np.array([5.,0.,1.5])
        wall=next(row for row in DEMO_OBSTACLES if row['name']=='demo_obstacle_wall')
        self.assertLess(obstacle_clearance([1.5,0.,1.5],wall),0.)
        self.assertGreater(min(obstacle_clearance(goal,row) for row in DEMO_OBSTACLES),.9)
        self.assertAlmostEqual(line_deviation([2.5,0.,1.5],start,goal),0.)
        self.assertGreater(line_deviation([2.5,1.,1.5],start,goal),.9)

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

    def test_fine_pad_target_contact_does_not_disable_other_collision_checks(self):
        t=NS(fixed_grasp_geometry_valid=True,fixed_grasp_forbidden_contact=False,
            fixed_grasp_clearance=.022,fixed_grasp_current_clearance=.025,
            approach_geometry_valid=False,arm_external_contact=True,
            arm_obstacle_clearance=.001,fixed_grasp_bilateral_fraction=1.)
        # Fine profile already classifies the exact permitted pair in the truth
        # plugin. Unfold cannot use that exception; other fine contacts reject.
        self.assertFalse(context_geometry(t,False)[0])
        self.assertEqual(context_geometry(t,True),(True,.025))
        self.assertEqual(context_geometry(t,True,True),(True,.022))
        t.fixed_grasp_forbidden_contact=True
        self.assertFalse(context_geometry(t,True)[0])
        self.assertFalse(context_geometry(t,True,True)[0])

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

    def test_tracking_copy_keeps_d_and_torque_caps(self):
        model=ET.parse(Path(__file__).resolve().parents[5]/
            'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/so101/so101.sdf').find('model')
        before=ET.tostring(model)
        copy=ET.fromstring(before);configure_arm_hold(copy)
        for name,cfg in ARM_HOLD.items():
            old=model.find("plugin[@name='%s_controller']"%name)
            new=copy.find("plugin[@name='%s_controller']"%name)
            for field in ('d','cmdMax','initialPosition'):
                self.assertEqual(old.findtext(field),new.findtext(field))
            self.assertAlmostEqual(float(new.findtext('p')),2*float(old.findtext('p')))
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
        np.testing.assert_allclose(world_progress(a,goal,.1),a+[0,0,.002])
        np.testing.assert_allclose(world_progress(a,goal,10.),a+[0,0,.002])
        np.testing.assert_allclose(world_progress(a,goal,.1,False),a)
        np.testing.assert_allclose(world_progress(goal,goal,.1),goal)
        np.testing.assert_allclose(world_progress(a,goal,.1,True,.002),a+[0,0,.0002])
        np.testing.assert_allclose(world_progress(a,goal,10.,True,.002),a+[0,0,.0002])
        with self.assertRaises(ValueError):world_progress(a,goal,-.1)
        np.testing.assert_allclose(world_progress(a,goal,.1,True,.02),a+[0,0,.002])
        with self.assertRaises(ValueError):world_progress(a,goal,.1,True,.02001)
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

    def test_grasp_frame_audit_starts_at_matching_terminal_hold(self):
        for streaming,matches,terminal,checked in ((False,False,True,False),
                (False,True,False,False),(False,True,True,True),(True,False,False,True)):
            with self.subTest(streaming=streaming,matches=matches,terminal=terminal):
                probe=FixedGraspProbe.__new__(FixedGraspProbe)
                probe.stop_event=Mock();probe.stop_event.wait.side_effect=[False,True]
                probe.streaming=streaming;probe.error=None;probe.log=Mock()
                truth=NS(header=NS(seq=1,stamp=NS(to_nsec=lambda:1000000000)))
                probe.sources=NS(read=lambda **kwargs:dict(now=1.,known_status=NS(terminal_hold=terminal),truth=truth))
                probe.plan=NS(matches=lambda status:matches)
                probe.status_lock=RLock();probe.feedback_lock=RLock();probe.status_snapshot=(1,{},{},{})
                probe.verify_feedback_frame=Mock(side_effect=RuntimeError('frame audit still enforced'))
                probe.run()
                self.assertEqual(probe.verify_feedback_frame.called,checked)
                self.assertEqual(probe.error,'frame audit still enforced' if checked else None)

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

    def test_unfold_handover_reserves_compensation_capacity(self):
        self.assertTrue(unfold_motion_ready(.020,.003,.05,.005))
        for values in [(.02001,0,0,0),(0,.00301,0,0),(0,0,.05001,0),
                (0,0,0,.00501),(float('nan'),0,0,0),(-.001,0,0,0)]:
            self.assertFalse(unfold_motion_ready(*values))
        demand=compensation_speed([0,0,0],[.008,0,0],[0,0,.04],[.3,0,0])
        self.assertTrue(unfold_motion_ready(demand,.001,.01,.004))
        self.assertLessEqual(demand,.020) # Original native 30mm/s leaves >=10mm/s.

    def test_open_passages_have_no_extra_dwell_but_final_gate_remains(self):
        self.assertEqual(waypoint_dwell_seconds(1,3),0.)
        self.assertEqual(waypoint_dwell_seconds(0,3),0.)
        self.assertEqual(waypoint_dwell_seconds(0,2),0.)
        self.assertEqual(waypoint_dwell_seconds(2,3),1.)
        self.assertEqual(waypoint_dwell_seconds(1,2),1.)

    def test_shared_status_preserves_mode_and_rejects_other_owners(self):
        for mode,locked,bench,rejected in ((0,False,False,False),(1,True,False,False),
                (0,True,False,True),(2,False,False,True),(1,True,True,True)):
            with self.subTest(mode=mode,locked=locked,bench=bench):
                probe=FixedGraspProbe.__new__(FixedGraspProbe)
                probe.stop_event=Mock();probe.stop_event.wait.side_effect=[False,True]
                probe.sources=NS(read=lambda **kwargs:dict(now=1.))
                values=dict(arm_hover_status={},arm_approach_status=dict(timestamp=1000000,mode=mode,ownership_locked=locked),
                    arm_control_status={},arm_grasp_status=dict(ownership_locked=bench))
                probe.status=Mock(side_effect=lambda topic:values[topic])
                probe.status_lock=RLock();probe.status_snapshot=None;probe.error=None
                probe.read_status()
                self.assertEqual(probe.error is not None,rejected)
                self.assertEqual(probe.stop_event.set.called,rejected)
                if not rejected:self.assertEqual(probe.status_snapshot[2]['mode'],mode)
                self.assertNotIn('arm_fixed_grasp_status',[c.args[0] for c in probe.status.call_args_list])

    def test_stopped_native_scheduler_aborts_instead_of_waiting_forever(self):
        probe=FixedGraspProbe.__new__(FixedGraspProbe)
        probe.stop_event=Mock();probe.stop_event.wait.side_effect=[False,True]
        probe.sources=NS(read=lambda **kwargs:dict(now=2.))
        probe.status=Mock(return_value=dict(timestamp=1000000))
        probe.status_lock=RLock();probe.status_snapshot=None;probe.error=None
        probe.read_status()
        self.assertIn('scheduled status stopped',probe.error)
        probe.stop_event.set.assert_called_once()
        self.assertIsNone(probe.status_snapshot)

    def test_lateral_center_does_not_advance_insertion(self):
        R=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
        start=np.zeros(3);result=lateral_center(start,R,.002)
        np.testing.assert_allclose(result,[0,-.0001,0])
        self.assertAlmostEqual(float((result-start)@R[:,2]),0.)
        np.testing.assert_allclose(lateral_center(start,R,float(OPEN_REFERENCE[0]+.0002)),start)
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
        self.assertTrue(precision_motion_ready(.010,.001,.001,.01,.010,body_limit=.010))
        self.assertFalse(precision_motion_ready(.010,.001,.001,.01,.0101,body_limit=.010))
        self.assertFalse(precision_motion_ready(.010,.001,.001,.01,0.,body_limit=.0101))
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
        # A new truth bracket alone must verify the pending sample: do not
        # wait for another (possibly slow) native feedback poll to return.
        row['now']=1.06;probe.verify_feedback_frame(row)
        self.assertEqual(probe.checked_feedback,10)
        probe.feedback=dict(probe.feedback,source_sequence=11,sample_time_us=1070000)
        row['now']=1.07;probe.verify_feedback_frame(row)
        self.assertEqual(probe.checked_feedback,10)
        self.assertTrue(probe.frame_verified)
        self.assertEqual(len(probe.pending_feedback),1)


class LidarPayloadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from lidar_grasp_demo import LidarDemoPlan
        cls.plan=LidarDemoPlan(Path(__file__).with_name('known_target_lidar.json'))

    def test_new_payload_inertia_and_grasp_origin_are_distinct(self):
        from lidar_grasp_demo import add_payload,transform
        world=ET.Element('world');r=add_payload(world,self.plan.target,self.plan.geometry_profile)
        I=np.array(r['inertia_com_kg_m2']);e=np.linalg.eigvalsh(I)
        self.assertTrue(np.all(e>0));self.assertLess(e[-1],sum(e[:2]))
        self.assertAlmostEqual(r['mass_kg'],.01)
        self.assertGreater(np.linalg.norm(r['com_object_m']),.05)
        model=world.find("model[@name='test_cube']")
        self.assertEqual(model.findtext('static'),'false')
        parts={c.get('name'):c for c in model.findall('link/collision')}
        self.assertEqual(set(parts),{'collision','body','key'})
        self.assertEqual(parts['collision'].findtext('geometry/box/size'),'0.02 0.02 0.07')
        self.assertEqual(parts['collision'].findtext('surface/contact/ode/kp'),'500')
        self.assertIsNone(parts['body'].find('surface/contact/ode/kp'))
        support=world.find("model[@name='cube_support']")
        expected=self.plan.target.object@transform(self.plan.geometry_profile['target']['object_from_support_xyz_rpy_deg'])
        np.testing.assert_allclose(np.fromstring(support.findtext('pose'),sep=' ')[:3],expected[:3,3])

    def test_handle_close_seats_fixed_finger_inside_original_entry_budget(self):
        from fixed_grasp_probe import centered_world_target
        local=OPEN_REFERENCE.copy();tool=np.zeros(3);R=np.eye(3)
        goal=centered_world_target(tool,R,local,self.plan.close_reference)
        np.testing.assert_allclose(local-goal,self.plan.close_reference)
        self.assertLess(np.linalg.norm(goal),.003)
        self.assertAlmostEqual(self.plan.close_reference[0]-.01+.008,.0002)

    def test_all_parts_guarded_and_only_handle_exempt_for_pads(self):
        from fixed_demo_plan import PAD_NAMES
        plan=self.plan
        for unfold in (True,False):
            rows=plan.geometry(plan.expected_q,.3,unfold)
            for pad in PAD_NAMES:
                obstacles={r['obstacle'] for r in rows if r['collision']==pad}
                self.assertTrue({'body','key','support','ground/asphalt'}<=obstacles)
                self.assertEqual('handle' in obstacles,unfold)
                support=next(r for r in rows if r['collision']==pad and r['obstacle']=='support')
                self.assertAlmostEqual(support['required'],.016 if unfold else .003)
        self.assertGreater(min(r['distance']-r['required'] for r in plan.geometry(plan.expected_q,.04)),.020)

    def test_estimated_orientation_is_preserved_and_wrong_orientation_rejected(self):
        import json,tempfile
        from lidar_grasp_demo import LidarDemoPlan,quaternion
        from known_target import mount_transform
        config=json.loads(Path(__file__).with_name('known_target_lidar.json').read_text())
        R=self.plan.target.object[:3,:3]@mount_transform([0,0,0,0,0,.01])[:3,:3]
        config['object_quaternion_world_wxyz']=quaternion(R).tolist()
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'estimate.json';path.write_text(json.dumps(config))
            plan=LidarDemoPlan(path);np.testing.assert_allclose(plan.target.object[:3,:3],R,atol=1e-12)
            config['object_quaternion_world_wxyz']=[1,0,0,0];path.write_text(json.dumps(config))
            with self.assertRaisesRegex(ValueError,'incompatible'):LidarDemoPlan(path)



class PillarCourseTests(unittest.TestCase):
    def test_wide_posts_and_alternating_clear_openings(self):
        from pillar_course import obstacles,GATE_X,LANE_Y,POST_SIZE,room
        items=obstacles();self.assertEqual(len(items),6)
        self.assertGreaterEqual(min(POST_SIZE[:2]),1.2)
        self.assertEqual(len(room()),5)
        for x,y in zip(GATE_X,LANE_Y):
            same=[o for o in items if o['center'][0]==x]
            self.assertEqual(len(same),2)
            self.assertGreater(min(obstacle_clearance([x,y,1.25],o) for o in items),.30)
        for o in (*items,*room()):self.assertTrue(o['name'].startswith('demo_obstacle_'))

    def test_gate_audit_rejects_bypass_and_overflight(self):
        from pillar_course import gate_crossings,GATE_X,LANE_Y
        rows=[];t=0.
        for x,y in zip(GATE_X,LANE_Y):
            rows.extend([dict(sim_s=t,body_world=[x-.1,y,1.25]),dict(sim_s=t+.4,body_world=[x+.1,y,1.25])]);t+=2
        self.assertTrue(gate_crossings(rows)['passed'])
        self.assertFalse(gate_crossings(rows[:2])['passed'])
        self.assertFalse(gate_crossings([dict(r,body_world=[r['body_world'][0],0.,1.25]) for r in rows])['passed'])
        self.assertFalse(gate_crossings([dict(r,body_world=[*r['body_world'][:2],3.5]) for r in rows])['passed'])

    def test_search_goals_end_before_target_grasp(self):
        from pillar_course import observation_goals
        from lidar_grasp_demo import LidarDemoPlan
        plan=LidarDemoPlan(Path(__file__).with_name('known_target_pillars.json'))
        goals=observation_goals(plan.target,1.25)
        self.assertEqual(len(goals),4)
        self.assertGreater(goals[-1][0],8.8)
        self.assertAlmostEqual(plan.target.object[0,3]-goals[-1][0],1.5)
        self.assertTrue(plan.preview()['sampled_geometry_feasible'])

class CarryTests(unittest.TestCase):
    def test_smooth_bounded_carry(self):
        from carry_demo import CarryRoute
        route=CarryRoute([1.,0.,1.9],[1.,.5,2.02],2.)
        speeds=[]
        for t in np.linspace(2.,2.+route.duration,1001):
            p,v=route.sample(t);speeds.append(np.linalg.norm(v))
            self.assertGreaterEqual(p[2],1.9-1e-10)
        self.assertLessEqual(max(speeds),.030001)
        np.testing.assert_allclose(route.sample(2.)[1],[0,0,0])
        np.testing.assert_allclose(route.sample(2.+route.duration)[0],route.goal)
        np.testing.assert_allclose(route.sample(2.+route.duration)[1],[0,0,0],atol=1e-10)
        self.assertFalse(route.finished(2.));self.assertTrue(route.finished(2.+route.duration+.001))
        with self.assertRaises(ValueError):route.sample(1.)

    def test_reject_invalid_carry_endpoints(self):
        from carry_demo import CarryRoute
        for goal in ([0,0,1.], [0,0,2.01], [0,1.,2.2], [float('nan'),0,2.2], [0,0,3.]):
            with self.assertRaises(ValueError):CarryRoute([0,0,2.],goal,0.)

    def test_carry_requires_actual_transport_and_contact(self):
        from carry_demo import carry_evidence
        rows=[dict(sim_s=i*.1,contact_frames=100,bilateral=1.,support=0.,object_local=[0,0,0],
            object_world=[i*.003,0,2.],phase=7,geometry_valid=True,forbidden_contact=False) for i in range(101)]
        self.assertTrue(carry_evidence(rows,[0,0,0])['validated'])
        self.assertFalse(carry_evidence([], [0,0,0])['validated'])
        for key,bad in [('support',1.),('bilateral',0.),('geometry_valid',False),('phase',8),
                        ('forbidden_contact',True),('object_local',[.05,0,0])]:
            changed=[dict(r,**{key:bad}) for r in rows]
            self.assertFalse(carry_evidence(changed,[0,0,0])['validated'])
        self.assertFalse(carry_evidence([dict(r,object_world=[0,0,2.]) for r in rows],[0,0,0])['validated'])
        self.assertFalse(carry_evidence(rows[:20],[0,0,0])['validated'])

if __name__=='__main__':unittest.main()
