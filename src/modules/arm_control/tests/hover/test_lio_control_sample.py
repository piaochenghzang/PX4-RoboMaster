"""Control prediction provenance, refusal bounds and measured FK velocity."""
from collections import deque
import copy
import json
import math
from threading import RLock
from types import SimpleNamespace
import unittest
import numpy as np
from lio_control_sample import predict_body,midpoint_body_odometry,sample_control,tool_velocity,flight_control_sample
from lio_tool_observer import LioToolObserver,fk_gripper,rpy_transform


def pose_values(x=0.,yaw=0.,roll=0.):
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    return [x,0.,0.,cy*cr,cy*sr,sy*sr,sy*cr]


def configuration():
    return dict(joint_names=['q'+str(j) for j in range(6)],mount_pose=[0.]*6,
        joint_origins=[[0.]*6 for _ in range(5)],tool_pose=[.2,0.,0.,0.,math.pi,0.])


def observer():
    target=np.eye(4);target[0,3]=1.
    return SimpleNamespace(lock=RLock(),error=None,config=configuration(),reference=np.array([.1,0.,0.]),
        body=deque([(1.,pose_values()),(1.1,pose_values(.01))]),
        joints=deque([(1.18,np.array([0.,0.,0.,0.,0.,.3]),np.zeros(6))]),
        target=target,target_stamp=.9,last_stamp=1.05)


class MidpointOdometryTests(unittest.TestCase):
    def test_pose_velocity_and_timestamp_refer_to_the_same_true_source_midpoint(self):
        samples=[(1.,pose_values()),(1.1,pose_values(.01,.02))];saved=copy.deepcopy(samples)
        result=midpoint_body_odometry(samples)
        self.assertAlmostEqual(result['stamp_s'],1.05)
        np.testing.assert_allclose(result['body'],rpy_transform([.005,0.,0.,0.,0.,.01]),atol=1e-12)
        np.testing.assert_allclose(result['linear_v_map'],[.1,0.,0.],atol=1e-12)
        np.testing.assert_allclose(result['linear_v_body'],[.1*math.cos(.01),-.1*math.sin(.01),0.],atol=1e-12)
        np.testing.assert_allclose(result['angular_v_body'],[0.,0.,.2],atol=1e-12)
        self.assertEqual(result['prediction_s'],0.)
        self.assertEqual(samples,saved)

    def test_nontrivial_body_rotation_changes_twist_basis_not_its_reference_point(self):
        result=midpoint_body_odometry([(1.,pose_values(roll=.4)),(1.1,pose_values(.01,.02,.4))])
        R=rpy_transform([0.,0.,0.,.4,0.,.01])[:3,:3]
        np.testing.assert_allclose(result['linear_v_body'],R.T@[.1,0.,0.],atol=1e-12)
        np.testing.assert_allclose(result['angular_v_body'],R.T@[0.,0.,.2],atol=1e-12)

    def test_missing_duplicate_reversed_or_dropped_pairs_are_refused(self):
        for samples in ([],[(1.,pose_values())],[(1.,pose_values())]*3,
                [(1.,pose_values()),(1.,pose_values())],
                [(1.1,pose_values()),(1.,pose_values())],
                [(1.,pose_values()),(1.2,pose_values())]):
            with self.subTest(samples=samples),self.assertRaises(ValueError):midpoint_body_odometry(samples)

    def test_takeoff_transport_does_not_relax_hover_control_derivative_limits(self):
        samples=[(1.,pose_values()),(1.1,pose_values(.04,.06))]
        result=midpoint_body_odometry(samples)
        np.testing.assert_allclose(result['linear_v_map'],[.4,0.,0.],atol=1e-12)
        with self.assertRaisesRegex(ValueError,'DERIVATIVE_LIMIT'):predict_body(samples,1.1)
        with self.assertRaisesRegex(ValueError,'DERIVATIVE_LIMIT'):
            midpoint_body_odometry([(1.,pose_values()),(1.1,pose_values(.081))])


class PredictionTests(unittest.TestCase):
    def test_constant_world_translation_and_yaw_have_hand_calculated_prediction(self):
        samples=[(1.,pose_values()),(1.1,pose_values(.01,.02))]
        result=predict_body(samples,1.2)
        np.testing.assert_allclose(result['body'][:3,3],[.02,0.,0.],atol=1e-12)
        np.testing.assert_allclose(result['body'][:3,:3],rpy_transform([0.,0.,0.,0.,0.,.04])[:3,:3],atol=1e-12)
        np.testing.assert_allclose(result['linear_v_map'],[.1,0.,0.],atol=1e-12)
        np.testing.assert_allclose(result['angular_v_map'],[0.,0.,.2],atol=1e-12)
        self.assertEqual(result['last_lio_stamp_s'],1.1)
        self.assertAlmostEqual(result['prediction_s'],.1)
        self.assertEqual(result['mode'],'BOUNDED_CONSTANT_TWIST_PREDICTION')

    def test_world_angular_rate_is_applied_on_left_of_nontrivial_body_rotation(self):
        result=predict_body([(1.,pose_values(roll=.4)),(1.1,pose_values(yaw=.02,roll=.4))],1.2)
        expected=rpy_transform([0.,0.,0.,.4,0.,.04])
        np.testing.assert_allclose(result['body'],expected,atol=1e-12)
        np.testing.assert_allclose(result['angular_v_map'],[0.,0.,.2],atol=1e-12)

    def test_bracket_interpolates_without_claiming_a_prediction(self):
        result=predict_body([(1.,pose_values()),(1.1,pose_values(.01,.02))],1.05)
        np.testing.assert_allclose(result['body'],rpy_transform([.005,0.,0.,0.,0.,.01]),atol=1e-12)
        self.assertEqual(result['prediction_s'],0.)
        self.assertEqual(result['mode'],'SOURCE_BRACKET_INTERPOLATION')

    def test_exact_sample_uses_measurement_and_retains_velocity_from_source_pair(self):
        result=predict_body([(1.,pose_values()),(1.1,pose_values(.01))],1.1)
        self.assertEqual(result['mode'],'SOURCE_MEASURED')
        self.assertEqual(result['prediction_s'],0.)
        np.testing.assert_allclose(result['linear_v_map'],[.1,0.,0.],atol=1e-12)

    def test_prediction_cannot_exceed_120ms_or_custom_smaller_limit(self):
        samples=[(1.,pose_values()),(1.1,pose_values(.01))]
        self.assertAlmostEqual(predict_body(samples,1.22)['prediction_s'],.12)
        with self.assertRaisesRegex(ValueError,'PREDICTION_LIMIT'):predict_body(samples,1.221)
        with self.assertRaisesRegex(ValueError,'PREDICTION_LIMIT'):predict_body(samples,1.18,.05)
        for limit in (-.01,.121,float('nan')):
            with self.subTest(limit=limit),self.assertRaises(ValueError):predict_body(samples,1.11,limit)
        for kwargs in (dict(max_linear_speed=.81),dict(max_angular_speed=1.01),
                dict(max_linear_speed=0.),dict(max_angular_speed=float('nan'))):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):predict_body(samples,1.11,**kwargs)

    def test_duplicate_reverse_invalid_or_missing_source_times_are_rejected(self):
        for stamps in ([1.,1.],[1.1,1.],[0.,1.],[float('nan'),1.1]):
            with self.subTest(stamps=stamps),self.assertRaises(ValueError):
                predict_body([(t,pose_values()) for t in stamps],1.1)
        with self.assertRaisesRegex(ValueError,'TWO_POSES'):predict_body([(1.,pose_values())],1.01)
        with self.assertRaisesRegex(ValueError,'NOT_BRACKETED'):
            predict_body([(1.,pose_values()),(1.1,pose_values())],.99)
        for stamp in (float('nan'),0.,float('inf')):
            with self.assertRaises(ValueError):predict_body([(1.,pose_values()),(1.1,pose_values())],stamp)

    def test_dropped_lio_frame_and_extreme_derivative_are_rejected(self):
        with self.assertRaisesRegex(ValueError,'PAIR_GAP'):
            predict_body([(1.,pose_values()),(1.2,pose_values(.01))],1.21)
        for last in (pose_values(.031),pose_values(yaw=.051)):
            with self.subTest(last=last),self.assertRaisesRegex(ValueError,'DERIVATIVE_LIMIT'):
                predict_body([(1.,pose_values()),(1.1,last)],1.12)

    def test_prediction_does_not_mutate_source_samples(self):
        samples=[(1.,pose_values()),(1.1,pose_values(.01))];saved=copy.deepcopy(samples)
        result=predict_body(samples,1.2);result['body'][:]=9.
        self.assertEqual(samples,saved)
        bad=pose_values();bad[-1]=float('nan')
        with self.assertRaises(ValueError):predict_body([(1.,pose_values()),(1.1,bad)],1.2)



class FilterHistoryTests(unittest.TestCase):
    def test_filter_history_returns_the_actual_latest_past_message(self):
        from types import SimpleNamespace
        from lio_control_sample import past_filter_message
        def frame(t):return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda:t)))
        a,b,c=frame(1.18),frame(1.20),frame(1.22)
        history=deque([a,b,c]);saved=list(history)
        self.assertIs(past_filter_message(history,1.201),b)
        self.assertIs(past_filter_message(history,1.221),c)
        self.assertEqual(list(history),saved)
        self.assertIsNone(past_filter_message([c],1.201))
        # Expired samples are not relabelled or dropped to hide the caller's
        # existing 200ms selected-source freshness rejection.
        self.assertIs(past_filter_message([a],1.5),a)
        self.assertEqual(a.header.stamp.to_sec(),1.18)
        with self.assertRaisesRegex(ValueError,'CURRENT_TIME'):past_filter_message(history,float('nan'))


class FlightPairTests(unittest.TestCase):
    def test_moving_body_is_not_misclassified_as_a_coordinate_bias(self):
        source=observer()
        row=flight_control_sample(source,1.18,1.2)
        # At 0.1m/s the old 1.10s pose would manufacture an 8mm EKF bias.
        np.testing.assert_allclose(row['body'][:3,3],[.018,0.,0.],atol=1e-12)
        np.testing.assert_allclose(row['linear_v_map'],[.1,0.,0.],atol=1e-12)
        self.assertEqual(row['stamp_s'],1.18)
        self.assertEqual(row['last_lio_stamp_s'],1.1)
        self.assertAlmostEqual(row['prediction_s'],.08)
        self.assertEqual(source.last_stamp,1.05)

    def test_filter_bracket_uses_interpolation_and_requires_no_target_or_encoder(self):
        source=observer();source.target=None;source.joints.clear()
        row=flight_control_sample(source,1.05,1.12)
        np.testing.assert_allclose(row['body'][:3,3],[.005,0.,0.],atol=1e-12)
        self.assertEqual(row['prediction_s'],0.)

    def test_stale_future_and_overlong_prediction_are_rejected(self):
        with self.assertRaisesRegex(ValueError,'STALE'):
            flight_control_sample(observer(),1.18,1.31)
        self.assertIsNone(flight_control_sample(observer(),1.221,1.23))
        source=observer();source.body[-1]=(1.21,pose_values())
        self.assertIsNone(flight_control_sample(source,1.18,1.2))
        self.assertIsNone(flight_control_sample(observer(),1.21,1.2))

    def test_small_clock_race_abstains_without_changing_the_source_stamp(self):
        source=observer()
        self.assertIsNone(flight_control_sample(source,1.201,1.2))
        self.assertEqual(source.body[-1][0],1.1)
        self.assertIsNotNone(flight_control_sample(source,1.201,1.202))


class ReferenceVelocityTests(unittest.TestCase):
    def test_translation_and_body_omega_include_arm_mount_lever(self):
        cfg=configuration();cfg['mount_pose'][0]=1.
        actual=tool_velocity(cfg,np.zeros(6),np.zeros(6),np.eye(4),[.01,.02,0.],[0.,0.,.1],[1.,0.,0.])
        np.testing.assert_allclose(actual,[.01,.22,0.],atol=1e-12)

    def test_joint_and_body_angular_velocity_transform_to_map_by_hand(self):
        cfg=configuration();cfg['mount_pose'][0]=1.
        q=np.zeros(6);q[0]=math.pi/2;dq=np.zeros(6);dq[0]=.2
        body=rpy_transform([5.,-2.,3.,0.,0.,math.pi/2])
        actual=tool_velocity(cfg,q,dq,body,[.01,.02,0.],[0.,0.,.1],[1.,0.,0.])
        # Endpoint_B=[1,1,0], body lever_M=[-1,1,0], joint_v_M=[0,-.2,0].
        np.testing.assert_allclose(actual,[-.09,-.28,0.],atol=1e-12)

    def test_gripper_joint_does_not_move_gripper_link_reference(self):
        cfg=configuration();q=np.zeros(6);dq=np.zeros(6);dq[5]=100.
        np.testing.assert_array_equal(tool_velocity(cfg,q,dq,np.eye(4),np.zeros(3),np.zeros(3),[.1,0.,0.]),np.zeros(3))

    def test_analytic_joint_velocity_matches_independent_fk_difference(self):
        cfg=configuration();cfg['mount_pose']=[.02,-.03,.05,.2,-.1,.4]
        cfg['joint_origins']=[[.03*i,.02,-.01*i,.1*i,-.1,.3] for i in range(5)]
        q=np.array([.2,-.4,.3,.1,-.6,.3]);dq=np.array([.1,-.2,.05,.03,-.07,1.])
        reference=np.array([.02,-.01,-.1]);body=rpy_transform([2.,3.,4.,-.1,.2,.5]);dt=1e-6
        mount=rpy_transform(cfg['mount_pose'])
        before=(body@mount@fk_gripper(cfg,q-dq*dt)@np.r_[reference,1])[:3]
        after=(body@mount@fk_gripper(cfg,q+dq*dt)@np.r_[reference,1])[:3]
        actual=tool_velocity(cfg,q,dq,body,np.zeros(3),np.zeros(3),reference)
        np.testing.assert_allclose(actual,(after-before)/(2*dt),atol=1e-9)

    def test_nonrigid_or_nonfinite_inputs_are_rejected(self):
        bad=np.eye(4);bad[0,0]=2.
        for values in ((np.zeros(5),np.zeros(6),np.eye(4)),(np.zeros(6),np.full(6,float('nan')),np.eye(4)),(np.zeros(6),np.zeros(6),bad)):
            with self.subTest(values=values),self.assertRaises(ValueError):
                tool_velocity(configuration(),*values,np.zeros(3),np.zeros(3),np.zeros(3))


class ControlSampleTests(unittest.TestCase):
    def test_joint_measurement_stamp_and_original_lio_stamp_remain_distinct(self):
        source=observer();saved=copy.deepcopy((list(source.body),list(source.joints),source.target,source.last_stamp))
        row=sample_control(source,1.2)
        self.assertEqual(row['stamp_s'],1.18)
        self.assertEqual(row['last_lio_stamp_s'],1.1)
        self.assertAlmostEqual(row['source_age_s'],.02)
        self.assertAlmostEqual(row['prediction_s'],.08)
        self.assertAlmostEqual(row['lio_source_age_s'],.1)
        self.assertEqual(row['source'],'LIO_BOUNDED_TWIST_ENCODER_FK')
        np.testing.assert_allclose(np.array(row['map_from_grasp_reference'])[:3,3],[.118,0.,0.],atol=1e-12)
        np.testing.assert_allclose(row['tcp_velocity_map_m_s'],[.1,0.,0.],atol=1e-12)
        json.dumps(row,allow_nan=False)
        self.assertEqual(source.last_stamp,saved[3]);self.assertEqual(list(source.body),saved[0])
        for a,b in zip(source.joints[-1][1:],saved[1][-1][1:]):np.testing.assert_array_equal(a,b)
        np.testing.assert_array_equal(source.target,saved[2]);self.assertFalse(row['motion_permission']);self.assertFalse(row['close_permission'])

    def test_strict_audit_still_refuses_the_same_unbracketed_encoder_sample(self):
        source=observer()
        self.assertIsNone(LioToolObserver.sample(source,1.2))
        self.assertIsNotNone(sample_control(source,1.2))
        self.assertEqual(source.last_stamp,1.05)

    def test_missing_sources_return_no_control_estimate(self):
        for field,value in [('target',None),('body',deque()),('joints',deque())]:
            source=observer();setattr(source,field,value)
            with self.subTest(field=field):self.assertIsNone(sample_control(source,1.2))

    def test_stale_or_future_joint_and_lio_sources_are_rejected(self):
        cases=[('joint_old',1.12,1.1),('joint_future',1.21,1.1),('lio_old',1.18,1.04),('lio_future',1.18,1.21)]
        for name,joint_stamp,lio_stamp in cases:
            source=observer();source.joints[-1]=(joint_stamp,np.zeros(6),np.zeros(6))
            source.body=deque([(lio_stamp-.1,pose_values()),(lio_stamp,pose_values())])
            with self.subTest(name=name):self.assertIsNone(sample_control(source,1.2))

    def test_small_clock_callback_race_abstains_until_clock_catches_without_restamping(self):
        for name in ('joint','lio'):
            source=observer()
            if name=='joint':source.joints[-1]=(1.201,np.zeros(6),np.zeros(6))
            else:source.body=deque([(1.101,pose_values()),(1.201,pose_values())])
            body_before=copy.deepcopy(list(source.body));joint_time=source.joints[-1][0]
            with self.subTest(source=name):
                self.assertIsNone(sample_control(source,1.2))
                self.assertEqual(list(source.body),body_before)
                self.assertEqual(source.joints[-1][0],joint_time)
                self.assertEqual(source.last_stamp,1.05)
                row=sample_control(source,1.202)
                self.assertIsNotNone(row)
                self.assertEqual(row['stamp_s'],joint_time)
                self.assertLessEqual(row['last_lio_stamp_s'],1.202)

    def test_large_future_source_is_not_hidden_by_another_small_clock_race(self):
        source=observer();source.joints[-1]=(1.201,np.zeros(6),np.zeros(6))
        source.body=deque([(1.11,pose_values()),(1.21,pose_values())])
        self.assertIsNone(sample_control(source,1.2))
        source=observer();source.joints[-1]=(1.21,np.zeros(6),np.zeros(6))
        source.body=deque([(1.101,pose_values()),(1.201,pose_values())])
        self.assertIsNone(sample_control(source,1.2))

    def test_future_callbacks_use_recent_history_without_restamping_any_source(self):
        source=observer()
        source.joints.append((1.23,np.ones(6),np.ones(6)))
        source.body.append((1.23,pose_values(.023)))
        row=sample_control(source,1.2)
        self.assertEqual(row['stamp_s'],1.18)
        self.assertEqual(row['last_lio_stamp_s'],1.1)
        np.testing.assert_array_equal(row['joint_position_rad'],[0.,0.,0.,0.,0.,.3])
        np.testing.assert_allclose(np.array(row['map_from_tcp'])[:3,3],[.218,0.,0.],atol=1e-12)
        self.assertEqual(source.body[-1][0],1.23)
        self.assertEqual(source.joints[-1][0],1.23)
        flight=flight_control_sample(source,1.18,1.2)
        self.assertEqual(flight['last_lio_stamp_s'],1.1)
        self.assertEqual(source.last_stamp,1.05)

    def test_no_historical_measurement_abstains_instead_of_using_expired_data(self):
        source=observer();source.joints.clear()
        source.joints.extend([(1.149,np.zeros(6),np.zeros(6)),(1.24,np.ones(6),np.ones(6))])
        self.assertIsNone(sample_control(source,1.2))
        source=observer();source.body.append((1.3,pose_values()))
        self.assertIsNone(sample_control(source,1.4))

    def test_prediction_gap_abstains_then_recovers_with_new_source_without_restamping(self):
        source=observer();source.joints[-1]=(1.221,np.zeros(6),np.zeros(6))
        saved=copy.deepcopy(list(source.body))
        self.assertIsNone(sample_control(source,1.23))
        self.assertIsNone(flight_control_sample(source,1.221,1.23))
        self.assertEqual(list(source.body),saved)
        source.body.append((1.2,pose_values(.02)))
        row=sample_control(source,1.23)
        self.assertEqual(row['stamp_s'],1.221)
        self.assertEqual(row['last_lio_stamp_s'],1.2)
        self.assertAlmostEqual(row['prediction_s'],.021)
        self.assertIsNotNone(flight_control_sample(source,1.221,1.23))

    def test_newest_pairable_encoder_is_selected_with_all_original_bounds(self):
        source=observer()
        source.joints.extend([(1.22,np.zeros(6),np.zeros(6)),(1.24,np.ones(6),np.ones(6))])
        row=sample_control(source,1.24)
        self.assertEqual(row['stamp_s'],1.22)
        self.assertAlmostEqual(row['prediction_s'],.12)
        self.assertAlmostEqual(row['source_age_s'],.02)
        np.testing.assert_array_equal(row['joint_position_rad'],np.zeros(6))
        self.assertEqual(source.joints[-1][0],1.24)
        self.assertEqual(source.last_stamp,1.05)
        source.joints=deque([(1.18,np.zeros(6),np.zeros(6)),(1.24,np.ones(6),np.ones(6))])
        self.assertIsNone(sample_control(source,1.24))

    def test_prediction_abstention_does_not_hide_invalid_motion(self):
        source=observer();source.body[-1]=(1.1,pose_values(.1))
        with self.assertRaisesRegex(ValueError,'LIO_DERIVATIVE_LIMIT'):
            sample_control(source,1.2)
        with self.assertRaisesRegex(ValueError,'LIO_DERIVATIVE_LIMIT'):
            flight_control_sample(source,1.18,1.2)

    def test_observer_errors_are_preserved_and_target_source_time_is_checked(self):
        source=observer();source.error='LIO_FRAME'
        with self.assertRaisesRegex(ValueError,'LIO_FRAME'):sample_control(source,1.2)
        source=observer();source.target_stamp=1.19
        self.assertIsNone(sample_control(source,1.2))
        source.target_stamp=float('nan')
        with self.assertRaisesRegex(ValueError,'TARGET_SOURCE_TIME'):sample_control(source,1.2)
        for now in (0.,float('nan')):
            with self.assertRaises(ValueError):sample_control(observer(),now)


if __name__=='__main__':unittest.main()
