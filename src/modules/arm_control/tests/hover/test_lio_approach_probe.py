"""LIO context frame/binding/time contracts and independent no-contact geometry."""
import copy
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock,patch
import numpy as np
from lio_approach_probe import context_values,approach_geometry,alignment_bias,cancellation_ack,transit_ready,LioApproachProbe
from lio_tool_observer import rpy_transform
from fixed_demo_plan import FixedDemoPlan


IDS=[1,11,22,33]


def stamp(seconds):
    return SimpleNamespace(to_sec=lambda:seconds,to_nsec=lambda:int(round(seconds*1e9)))


def lease(now=12.36,*,ids=IDS,frame='map',header_age=.02,command_age=.06,
          terminal=True,valid=True):
    return SimpleNamespace(task_id=ids[0],plan_id=ids[1],target_id=ids[2],scene_id=ids[3],
        header=SimpleNamespace(frame_id=frame,stamp=stamp(now-header_age)),
        command_stamp=stamp(now-command_age),terminal_hold=terminal,lease_valid=valid)


def sample():
    body=rpy_transform([1.,2.,3.,0.,0.,math.pi/2])
    arm=body@rpy_transform([.1,0.,-.05,0.,0.,0.])
    grip=rpy_transform([1.2,2.1,3.,0.,0.,math.pi/2])
    reference=grip.copy();reference[:3,3]=[1.25,2.1,3.]
    target=grip@rpy_transform([0.,0.,0.,.2,0.,0.]);target[:3,3]=[1.3,2.1,3.]
    return dict(frame='map',stamp_s=12.345678,map_from_body=body,map_from_arm=arm,
        map_from_gripper=grip,map_from_grasp_reference=reference,map_from_target=target,
        linear_velocity_map_m_s=[.03,.04,0.],angular_velocity_map_rad_s=[0.,.02,0.],
        tcp_velocity_map_m_s=[.006,.008,0.],reference_velocity_map_m_s=[.009,.012,0.])


def context(row=None,gate=None):
    row=sample() if row is None else row
    candidate=np.array(row['map_from_body']).copy();candidate[0,3]+=.01
    goal=np.eye(4);goal[:3,3]=[1.3,2.2,3.15]
    return context_values(row,goal,candidate,lease() if gate is None else gate,IDS,
        dict(valid=True,clearance_m=.05),12.36)


class LioContextTests(unittest.TestCase):
    def test_cli_layout_and_hand_calculated_arm_coordinates(self):
        fields=context()
        self.assertEqual(len(fields),22)
        self.assertEqual([fields[i] for i in (0,12,13,14)],IDS)
        self.assertEqual(fields[1:5],[12345678,12345678,1,1])
        # Arm origin is (1,2.1,2.95), yaw90deg: inverse applies
        # (goal-origin)=(.3,.1,.2) -> (.1,-.3,.2).
        np.testing.assert_allclose(fields[18:21],[.1,-.3,.2],atol=1e-12)
        np.testing.assert_allclose(fields[5:12],[.05,.02,0.,.01,.05,.015,.2],atol=1e-12)
        self.assertEqual(fields[15],12300000)
        self.assertEqual(fields[16:18],[1,1])
        self.assertEqual(fields[21],.05)

    def test_seated_grasp_alignment_uses_one_fixed_reference_not_the_transit_waypoint(self):
        row=sample();saved=copy.deepcopy(row)
        reference=np.array(row['map_from_target'])[:3,3]+[.0023,0.,0.]
        row['map_from_grasp_reference'][:3,3]=reference+[.002,0.,0.]
        outputs=[]
        for transit in ([1.,2.,3.],[1.3,2.2,3.15]):
            goal=np.eye(4);goal[:3,3]=transit
            fields=context_values(row,goal,row['map_from_body'],lease(),IDS,
                dict(valid=True,clearance_m=.05),12.36,alignment_target_map=reference)
            self.assertAlmostEqual(fields[9],.002)
            outputs.append(fields)
        self.assertEqual(outputs[0][5:12],outputs[1][5:12])
        ordinary=context_values(row,np.eye(4),row['map_from_body'],lease(),IDS,
            dict(valid=True,clearance_m=.05),12.36)
        self.assertAlmostEqual(ordinary[9],.0043)
        np.testing.assert_array_equal(row['map_from_target'],saved['map_from_target'])
        self.assertEqual(outputs[0][10:12],ordinary[10:12])

    def test_relative_speed_uses_the_larger_reference_or_tcp_speed(self):
        row=sample();row['tcp_velocity_map_m_s']=[0.,0.,.025]
        self.assertAlmostEqual(context(row)[10],.025)
        row['reference_velocity_map_m_s']=[.03,0.,.04]
        self.assertAlmostEqual(context(row)[10],.05)

    def test_body_relative_transit_subtracts_goal_velocity_but_retains_flight_speed(self):
        row=sample();row['tcp_velocity_map_m_s']=[.03,.04,0.];row['reference_velocity_map_m_s']=[.03,.04,0.]
        fields=context_values(row,np.eye(4),row['map_from_body'],lease(),IDS,
            dict(valid=True,clearance_m=.05),12.36,goal_velocity_map=[.03,.04,0.])
        self.assertAlmostEqual(fields[10],0.)
        self.assertAlmostEqual(fields[5],.05)

    def test_measurement_timestamp_is_preserved_without_receipt_restamping(self):
        row=sample();row['stamp_s']=12.310123
        fields=context(row)
        self.assertEqual(fields[1],12310123)
        self.assertEqual(fields[2],12310123)
        self.assertNotEqual(fields[2],12360000)

    def test_wrong_binding_lease_state_and_time_are_refused(self):
        invalid=[lease(ids=[1,12,22,33]),lease(ids=[2,11,22,33]),
            lease(ids=[1,11,23,33]),lease(ids=[1,11,22,34]),
            lease(terminal=False),lease(valid=False),lease(header_age=.10001),
            lease(command_age=.30001),lease(header_age=-.02001),lease(command_age=-.02001)]
        for value in invalid:
            with self.subTest(lease=value):self.assertEqual(context(gate=value)[16],0)
        row=sample();body=row['map_from_body'];goal=np.eye(4)
        fields=context_values(row,goal,body,None,IDS,dict(valid=True,clearance_m=.05),12.36)
        self.assertEqual(fields[15:17],[0,0])

    def test_small_callback_clock_skew_uses_the_existing_native_tolerance(self):
        self.assertEqual(context(gate=lease(header_age=-.001))[16],1)
        self.assertEqual(context(gate=lease(command_age=-.001))[16],1)
        self.assertEqual(context(gate=lease(header_age=-.001,command_age=-.001))[16],1)

    def test_wrong_status_frame_does_not_authorize_a_matching_lease(self):
        self.assertEqual(context(gate=lease(frame='world'))[16],0)

    def test_geometry_refusal_remains_separate_from_a_valid_flight_lease(self):
        row=sample();body=row['map_from_body'];goal=np.eye(4)
        fields=context_values(row,goal,body,lease(),IDS,dict(valid=False,clearance_m=.01),12.36)
        self.assertEqual(fields[16:18],[1,0]);self.assertEqual(fields[21],.01)

    def test_wrong_state_frame_is_rejected_before_native_io(self):
        row=sample();row['frame']='gazebo_world'
        with self.assertRaises(ValueError):context(row)

    def test_horizontal_map_origin_change_preserves_the_context_geometry(self):
        row=sample();candidate=row['map_from_body'].copy();candidate[0,3]+=.01
        goal=np.eye(4);goal[:3,3]=[1.3,2.2,3.15]
        before=context_values(row,goal,candidate,lease(),IDS,dict(valid=True,clearance_m=.05),12.36)
        change=rpy_transform([5.,-4.,.7,0.,0.,.8]);moved=copy.deepcopy(row)
        for key in ('map_from_body','map_from_arm','map_from_gripper','map_from_grasp_reference','map_from_target'):
            moved[key]=change@row[key]
        for key in ('linear_velocity_map_m_s','angular_velocity_map_rad_s',
                    'tcp_velocity_map_m_s','reference_velocity_map_m_s'):
            moved[key]=(change[:3,:3]@row[key]).tolist()
        after=context_values(moved,change@goal,change@candidate,lease(),IDS,dict(valid=True,clearance_m=.05),12.36)
        np.testing.assert_allclose(after,before,atol=1e-12)


class LioCancellationTests(unittest.TestCase):
    def test_ack_requires_approach_mode_task_binding_and_a_new_accepted_request(self):
        state=dict(zip(('task_id','plan_id','target_id','scene_id'),IDS),
            mode=0,phase=5,close_authorized=False,request_result=1,last_request_id=101)
        arm=dict(joint_target=[0.,0.,0.,0.,0.,.3])
        self.assertTrue(cancellation_ack(state,arm,IDS,.3,100))
        for key,value in [('mode',1),('phase',4),('close_authorized',True),
                ('request_result',3),('last_request_id',100),('last_request_id',99),
                *[(key,999) for key in ('task_id','plan_id','target_id','scene_id')]]:
            with self.subTest(key=key,value=value):
                self.assertFalse(cancellation_ack(dict(state,**{key:value}),arm,IDS,.3,100))
        self.assertFalse(cancellation_ack(state,dict(joint_target=[0.]*6),IDS,.3,100))
        self.assertFalse(cancellation_ack(state,dict(joint_target=[0.]*5+[float('nan')]),IDS,.3,100))


class RejectedStartCleanupTests(unittest.TestCase):
    def probe(self,folder,*,changed=False,request=101):
        p=LioApproachProbe.__new__(LioApproachProbe)
        p.stop_event=Mock();p.thread=Mock();p.thread.is_alive.return_value=False
        p.folder=Path(folder);p.grasp=True;p.jaw=.3;p.ids=IDS;p.motion_requested=True;p.complete=False
        p.error=None;p.events=[];p.last_record=None;p.closing_requested=False
        p.native=dict(last_request_id=100)
        p.baseline=dict(joint_target=[-.07,-1.175,1.45,1.3,-1.57,.3]);p.arm=copy.deepcopy(p.baseline)
        arm=copy.deepcopy(p.baseline)
        if changed:arm['joint_target'][1]+=.01
        state=dict(mode=1,phase=0,ownership_locked=False,request_result=3,last_request_id=request,jaw_target=.3)
        p.client=Mock();p.client.status.side_effect=lambda topic:state if topic=='arm_approach_status' else arm
        return p

    def test_rejected_unowned_start_has_a_confirmed_no_motion_cancel_result(self):
        with TemporaryDirectory() as folder:
            p=self.probe(folder);result=p.finish()
            self.assertFalse(result['completed'])
            self.assertTrue(result['cancellation']['acknowledged'])
            self.assertTrue(result['cancellation']['motion_not_acquired'])
            self.assertTrue(result['cancellation']['jaw_unchanged'])
            p.client.client.assert_called_once_with('arm_control','fixed_grasp_cancel',*IDS)

    def test_an_old_response_or_changed_joint_targets_cannot_claim_no_motion(self):
        for changed,request in ((True,101),(False,100)):
            with self.subTest(changed=changed,request=request),TemporaryDirectory() as folder:
                p=self.probe(folder,changed=changed,request=request)
                with patch('lio_approach_probe.time.monotonic',side_effect=[0.,.1,3.]),patch('lio_approach_probe.time.sleep'):
                    with self.assertRaisesRegex(RuntimeError,'not acknowledged'):p.finish()


class LioPrecisionTests(unittest.TestCase):
    def geometry(self,name,*,support_x=.4,points=None,extra=None):
        row=dict(map_from_body=np.eye(4),map_from_arm=np.eye(4),
            map_from_target=np.eye(4),joint_position_rad=np.zeros(6),
            prediction_translation_m=0.,prediction_rotation_rad=0.)
        parts=[dict(name='handle',size_m=[.02]*3,xyz_rpy_deg=[0.]*6)]
        if extra is not None:parts.append(extra)
        geometry=dict(target=dict(parts=parts,support_size_m=[.02]*3,
            object_from_support_xyz_rpy_deg=[support_x,0.,0.,0.,0.,0.]))
        static=[('vehicle',rpy_transform([0.,.5,0.,0.,0.,0.]),np.array([.01]*3))]
        cloud=np.array([[0.,0.,0.],[0.,.4,0.]]) if points is None else points
        with patch.object(FixedDemoPlan,'boxes',return_value=[(name,np.eye(4),np.array([.01]*3))]):
            return (approach_geometry(row,{},geometry,static,cloud),
                approach_geometry(row,{},geometry,static,cloud,precision=True))

    def test_only_designated_pads_may_contact_handle(self):
        for name in ('gripper_fixed_finger_collision','moving_jaw_collision'):
            ordinary,fine=self.geometry(name)
            self.assertFalse(ordinary['valid']);self.assertTrue(fine['valid'])
            self.assertEqual(fine['contact_exemptions'],[[name,'handle']])
        self.assertFalse(self.geometry('gripper_body_collision')[1]['valid'])

    def test_body_and_unclassified_return_still_refuse_contact(self):
        extra=dict(name='body',size_m=[.02]*3,xyz_rpy_deg=[0.]*6)
        self.assertFalse(self.geometry('gripper_fixed_finger_collision',extra=extra)[1]['valid'])
        self.assertFalse(self.geometry('moving_jaw_collision',points=np.array([[.022,0.,0.]]))[1]['valid'])

    def test_support_exception_reserves_three_mm_plus_localization(self):
        # Two half-widths total20mm; x28mm leaves8mm physical separation.
        _,fine=self.geometry('moving_jaw_collision',support_x=.028,points=np.array([[0.,.4,0.]]))
        self.assertTrue(fine['valid']);self.assertAlmostEqual(fine['clearance_m'],.017)
        _,fine=self.geometry('moving_jaw_collision',support_x=.026,points=np.array([[0.,.4,0.]]))
        self.assertFalse(fine['valid'])

    def test_full_stroke_reserves_motion_between_samples(self):
        from lio_approach_probe import jaw_sweep_geometry
        row=dict(joint_position_rad=np.zeros(6),map_from_arm=np.eye(4),map_from_gripper=np.eye(4))
        with patch('lio_approach_probe.approach_geometry',return_value=dict(external_clearance_m=.03,arm_body_lower_bound_m=.04)) as measure:
            result=jaw_sweep_geometry(row, {}, {}, [], [],precision=True)
        self.assertEqual(measure.call_count,1)
        moving=[b for b in measure.call_args.kwargs['boxes_override'] if b[0]=='moving_jaw_collision']
        self.assertEqual(len(moving),53)
        self.assertAlmostEqual(result['external_clearance_m'],.03-.11*.005)
        self.assertAlmostEqual(result['arm_body_lower_bound_m'],.04-.11*.005)

    def test_batched_stroke_matches_complete_production_scene_scalar_evaluations(self):
        from lio_approach_probe import jaw_sweep_geometry
        geometry=dict(target=dict(parts=[dict(name='handle',size_m=[.02]*3,xyz_rpy_deg=[0.]*6)],
            support_size_m=[.02]*3,object_from_support_xyz_rpy_deg=[.2,0.,0.,0.,0.,0.]))
        for q in ([0.,-.8,1.2,-1.,-.5,.3],[-.07,-1.175,1.45,1.3,-1.57,.3]):
            row=dict(joint_position_rad=q,map_from_arm=np.eye(4),map_from_body=np.eye(4),
                map_from_target=rpy_transform([.8,0.,.5,0.,0.,0.]),
                prediction_translation_m=.001,prediction_rotation_rad=.002)
            static=[('vehicle',rpy_transform([.1,.2,.1,0.,0.,0.]),np.array([.01]*3))]
            points=np.array([[.5,.4,0.],[.8,0.,.5]])
            for precision in (False,True):
                with self.subTest(q=q,precision=precision):
                    batch=jaw_sweep_geometry(row,{},geometry,static,points,precision=precision)
                    scalar=[approach_geometry(row,{},geometry,static,points,precision=precision,jaw=v) for v in np.linspace(.04,.3,53)]
                    for key in ('external_clearance_m','arm_body_lower_bound_m'):
                        self.assertAlmostEqual(batch[key],min(x[key] for x in scalar)-.11*.005,places=12)

    def test_contact_grasp_cancel_requires_fixed_mode_and_keeps_jaw_bias(self):
        state=dict(zip(('task_id','plan_id','target_id','scene_id'),IDS),
            mode=1,phase=9,close_authorized=False,request_result=1,last_request_id=101)
        arm=dict(joint_target=[0.]*5+[.04])
        self.assertTrue(cancellation_ack(state,arm,IDS,.04,100,grasp=True))
        self.assertFalse(cancellation_ack(dict(state,mode=0),arm,IDS,.04,100,grasp=True))
        self.assertFalse(cancellation_ack(dict(state,phase=7),arm,IDS,.04,100,grasp=True))


class LioFinishTests(unittest.TestCase):
    def test_motion_fault_keeps_cancel_evidence_instead_of_reporting_cleanup_failure(self):
        with TemporaryDirectory() as directory:
            p=LioApproachProbe.__new__(LioApproachProbe)
            p.stop_event=Mock();p.thread=Mock();p.thread.is_alive.return_value=False
            p.motion_requested=True;p.complete=False;p.error='geometry refused';p.ids=IDS;p.jaw=.3
            p.native=dict(last_request_id=100);p.events=[];p.last_record=None;p.folder=Path(directory)
            p.client=Mock();state=dict(zip(('task_id','plan_id','target_id','scene_id'),IDS),
                mode=0,phase=5,close_authorized=False,request_result=1,last_request_id=101)
            p.client.status.side_effect=[state,dict(joint_target=[0.]*5+[.3])]
            result=p.finish()
            self.assertFalse(result['completed']);self.assertEqual(result['error'],'geometry refused')
            self.assertTrue(result['cancellation']['acknowledged'])
            p.client.client.assert_called_once_with('arm_control','hover_cancel',*IDS)


class LioAlignmentBiasTests(unittest.TestCase):
    def test_tracking_error_uses_the_compensated_filter_reference(self):
        from lio_approach_probe import filter_tracking_errors
        position,speed=filter_tracking_errors([1.07,2.,3.],[.06,0.,0.],[1.,2.,3.],[.06,0.,0.],[.05,0.,0.])
        self.assertAlmostEqual(position,.01)
        self.assertAlmostEqual(speed,.01)

    def test_velocity_bias_compensates_estimated_velocity_offset_with_limits(self):
        from lio_approach_probe import alignment_velocity
        previous=np.zeros(3)
        out=alignment_velocity(previous,[.03,-.04,0.],[0.,0.,0.],.1)
        np.testing.assert_allclose(out,[.012,-.016,0.],atol=1e-12)
        for _ in range(20):out=alignment_velocity(out,[.3,-.4,0.],np.zeros(3),.1)
        np.testing.assert_allclose(out,[.03,-.04,0.],atol=1e-12)
        late=alignment_velocity(previous,[.3,-.4,0.],np.zeros(3),1.)
        self.assertAlmostEqual(np.linalg.norm(late),.020)
        np.testing.assert_array_equal(previous,np.zeros(3))
        for elapsed in (0.,-.1,float('nan')):
            with self.assertRaises(ValueError):alignment_velocity(previous,[0.,0.,0.],[0.,0.,0.],elapsed)

    def test_filter_minus_lio_sign_and_twenty_mm_per_second_rate(self):
        bias,velocity=alignment_bias(np.zeros(3),[1.,2.,3.],[1.03,1.96,3.],.1)
        np.testing.assert_allclose(bias,[-.0012,.0016,0.],atol=1e-12)
        np.testing.assert_allclose(velocity,[-.012,.016,0.],atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(velocity),.02)

    def test_offset_norm_is_capped_and_long_dt_does_not_jump_to_goal(self):
        bias=np.zeros(3)
        for _ in range(60):
            previous=bias.copy();bias,velocity=alignment_bias(bias,[1.,2.,2.],np.zeros(3),.1)
            self.assertLessEqual(np.linalg.norm(bias),.06000000001)
            self.assertLessEqual(np.linalg.norm(bias-previous),.002000000001)
            self.assertLessEqual(np.linalg.norm(velocity),.02000000001)
        np.testing.assert_allclose(bias,[.02,.04,.04],atol=1e-12)
        delayed,velocity=alignment_bias(np.zeros(3),[1.,0.,0.],np.zeros(3),1.)
        np.testing.assert_allclose(delayed,[.002,0.,0.],atol=1e-12)
        np.testing.assert_allclose(velocity,[.002,0.,0.],atol=1e-12)

    def test_invalid_time_and_pose_inputs_are_refused_without_mutation(self):
        for elapsed in (0.,-.01,float('nan'),float('inf')):
            with self.subTest(elapsed=elapsed),self.assertRaises(ValueError):
                alignment_bias(np.zeros(3),[.01,0.,0.],np.zeros(3),elapsed)
        for previous,ekf,lio in [([0.,0.],[0.,0.,0.],[0.,0.,0.]),
                ([0.,0.,0.],[float('nan'),0.,0.],[0.,0.,0.]),
                ([0.,0.,0.],[0.,0.,0.],[0.,float('inf'),0.])]:
            with self.subTest(previous=previous,ekf=ekf,lio=lio),self.assertRaises(ValueError):
                alignment_bias(previous,ekf,lio,.1)
        previous=np.array([.001,0.,0.]);ekf=np.array([.03,.02,0.]);lio=np.array([.01,0.,0.])
        saved=[value.copy() for value in (previous,ekf,lio)]
        result,_=alignment_bias(previous,ekf,lio,.1);result[:]=99.
        for actual,expected in zip((previous,ekf,lio),saved):np.testing.assert_array_equal(actual,expected)


class LioApproachGeometryTests(unittest.TestCase):
    def setUp(self):
        self.row=dict(map_from_body=np.eye(4),map_from_arm=np.eye(4),
            map_from_target=rpy_transform([.2,0.,0.,0.,0.,0.]),joint_position_rad=np.zeros(6),
            prediction_translation_m=0.,prediction_rotation_rad=0.)
        self.geometry=dict(target=dict(parts=[dict(name='body',size_m=[.02]*3,xyz_rpy_deg=[0.]*6)],
            support_size_m=[.02]*3,object_from_support_xyz_rpy_deg=[.2,0.,0.,0.,0.,0.]))
        self.static=[('body',rpy_transform([0.,.5,0.,0.,0.,0.]),np.array([.01]*3))]
        self.points=np.array([[0.,.4,0.],[1.,1.,1.]])
        self.patch=patch.object(FixedDemoPlan,'boxes',return_value=[('arm',np.eye(4),np.array([.01]*3))])
        self.patch.start();self.addCleanup(self.patch.stop)

    def evaluate(self):return approach_geometry(self.row,{},self.geometry,self.static,self.points)

    def test_hand_calculated_target_gap_reserves_localization(self):
        out=self.evaluate()
        self.assertTrue(out['valid'])
        self.assertAlmostEqual(out['target_support_lower_bound_m'],.18)
        self.assertAlmostEqual(out['observed_return_gap_m'],.39)
        self.assertAlmostEqual(out['arm_body_lower_bound_m'],.48)
        self.assertAlmostEqual(out['clearance_m'],.176)
        self.assertFalse(out['unknown_space_certified']);self.assertEqual(out['contact_exemptions'],[])

    def test_target_point_returns_or_vehicle_can_each_refuse_approach(self):
        self.row['map_from_target'][0,3]=.03
        self.assertFalse(self.evaluate()['valid'])
        self.row['map_from_target'][0,3]=.2;self.points=np.array([[.025,0.,0.]])
        self.assertFalse(self.evaluate()['valid'])
        self.points=np.array([[0.,.4,0.]])
        self.static=[('body',rpy_transform([0.,.03,0.,0.,0.,0.]),np.array([.01]*3))]
        self.assertFalse(self.evaluate()['valid'])

    def test_prediction_translation_and_rotation_are_reserved_from_clearance(self):
        first=self.evaluate();self.row['prediction_translation_m']=.01;self.row['prediction_rotation_rad']=.02
        second=self.evaluate()
        self.assertAlmostEqual(first['clearance_m']-second['clearance_m'],.01+.51*.02)
        self.assertAlmostEqual(second['prediction_localization_margin_m'],.004+.01+.51*.02)

    def test_empty_body_geometry_is_rejected_without_exporting_infinity(self):
        self.static=[]
        with self.assertRaises(ValueError):self.evaluate()

    def test_inputs_are_not_modified(self):
        saved=copy.deepcopy((self.row,self.geometry,self.static,self.points));self.evaluate()
        for key in self.row:
            np.testing.assert_array_equal(self.row[key],saved[0][key])
        self.assertEqual(self.geometry,saved[1]);np.testing.assert_array_equal(self.points,saved[3])
        np.testing.assert_array_equal(self.static[0][1],saved[2][0][1])
        np.testing.assert_array_equal(self.static[0][2],saved[2][0][2])


class TransitReadyTests(unittest.TestCase):
    def test_only_far_waypoint_can_use_instantaneous_readiness(self):
        self.assertTrue(transit_ready(1,1,.00244,.00733,True))
        for index,phase,error,speed,hover in [(0,1,.002,.007,True),(2,1,.002,.007,True),
                (1,3,.002,.007,True),(1,4,.002,.007,True),(1,1,.0051,.007,True),
                (1,1,.002,.0101,True),(1,1,.002,.007,False),(1,1,float('nan'),.007,True)]:
            self.assertFalse(transit_ready(index,phase,error,speed,hover))
        self.assertTrue(transit_ready(2,2,.003,.005,True))


class SmoothGoalTests(unittest.TestCase):
    def probe(self):
        p=LioApproachProbe.__new__(LioApproachProbe);p.index=1
        p.goals=[np.eye(4) for _ in range(3)];p.goals[1][0,3]=.05
        p.segment=(10.,np.zeros(3),1.875*.05/.010)
        return p

    def test_transition_is_bounded_and_keeps_exact_static_endpoint(self):
        p=self.probe();saved=p.goals[1].copy();duration=p.segment[2]
        for t in np.linspace(10,10+duration,101):
            goal,velocity,done=p.goal_at({},t)
            self.assertLessEqual(np.linalg.norm(velocity),.010000001)
            self.assertGreaterEqual(goal[0,3],0);self.assertLessEqual(goal[0,3],.050000001)
        start,v,done=p.goal_at({},10);np.testing.assert_allclose(start[:3,3],0);self.assertFalse(done)
        goal,v,done=p.goal_at({},10+duration+1)
        np.testing.assert_array_equal(goal,saved);np.testing.assert_allclose(v,0);self.assertTrue(done)
        np.testing.assert_array_equal(p.goals[1],saved)

    def test_invalid_transition_clock_is_refused(self):
        p=self.probe()
        for stamp in (9.9,float('nan')):
            with self.assertRaisesRegex(ValueError,'SEGMENT_TIME'):p.goal_at({},stamp)


class LioApproachCoverageRegressionTests(unittest.TestCase):
    def create(self,points,folder,client):
        candidate=dict(map_from_target=np.eye(4),map_from_body_hover=np.eye(4),map_from_arm=np.eye(4),
            pre_goal=dict(map_from_desired_tcp=np.eye(4)))
        static=[('vehicle::base',np.eye(4),np.ones(3)*.01)]
        with patch('lio_route_clearance.body_collision_boxes',return_value=static):
            return LioApproachProbe(client,SimpleNamespace(),{}, {},candidate,np.zeros(5),folder,
                {},Path('/unused/scene.world'),points)

    def test_sparse_nonempty_local_returns_do_not_invent_a_fifty_point_threshold(self):
        # Global map and target-fit acceptance belong to the caller. The local
        # registered subset may legitimately contain just 49 voxel representatives.
        for count in (1,49):
            with self.subTest(count=count),TemporaryDirectory() as directory:
                client=Mock();client.status.return_value=dict(joint_target=[-.07,-1.175,1.45,1.3,-1.57,.3])
                points=np.column_stack([np.linspace(.1,.5,count),np.zeros(count),np.zeros(count)])
                probe=self.create(points,Path(directory)/'probe',client)
                self.assertEqual(len(probe.points),count)
                self.assertEqual(len(probe.goals),3)
                np.testing.assert_allclose(probe.goals[1][:3,3]-probe.goals[2][:3,3],[0.,0.,.015])
                client.status.assert_called_once_with('arm_control_status')
                client.client.assert_not_called()
                self.assertFalse(probe.thread.is_alive())
                self.assertFalse(probe.motion_requested)

    def test_missing_global_or_local_returns_still_refuse_before_native_io(self):
        for points in (np.empty((0,3)),np.array([[2.,0.,0.],[0.,3.,0.]])):
            with self.subTest(points=points.tolist()),TemporaryDirectory() as directory:
                client=Mock()
                with self.assertRaises(ValueError):self.create(points,Path(directory)/'probe',client)
                client.status.assert_not_called();client.client.assert_not_called()


if __name__=='__main__':unittest.main()
