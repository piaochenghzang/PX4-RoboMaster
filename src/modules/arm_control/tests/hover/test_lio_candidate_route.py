"""Folded candidate route bounds and independent-frame disagreement checks."""
import math
import unittest
import numpy as np
from lio_candidate_route import CandidateRoute,check_frame_agreement,wrap_yaw


class CandidateRouteTests(unittest.TestCase):
    def test_climbs_in_place_then_translates_at_fixed_height(self):
        start=np.array([.3,0.,.9]);goal=np.array([1.32,.02,1.59])
        route=CandidateRoute(start,goal,.1,-.1,20.)
        position,velocity,yaw=route.sample(20.)
        np.testing.assert_array_equal(position,start);np.testing.assert_array_equal(velocity,np.zeros(3))
        self.assertAlmostEqual(yaw,.1)
        position,velocity,_=route.sample(20.+route.climb_duration*.5)
        np.testing.assert_allclose(position,[.3,0.,1.245],atol=1e-12)
        np.testing.assert_array_equal(velocity[:2],np.zeros(2))
        position,velocity,_=route.sample(20.+route.climb_duration)
        np.testing.assert_allclose(position,[.3,0.,1.59],atol=1e-12)
        np.testing.assert_allclose(velocity,np.zeros(3),atol=1e-12)
        position,velocity,_=route.sample(20.+route.climb_duration+route.travel_duration*.5)
        np.testing.assert_allclose(position,[.81,.01,1.59],atol=1e-12)
        self.assertEqual(velocity[2],0.)

    def test_peak_speeds_and_yaw_rate_remain_within_requested_limits(self):
        route=CandidateRoute([0.,0.,.5],[1.3,-.2,1.3],-1.5,1.4,0.)
        times=np.linspace(0.,route.duration,5001)
        samples=[route.sample(t) for t in times]
        vertical=[abs(v[2]) for _,v,_ in samples]
        horizontal=[np.linalg.norm(v[:2]) for _,v,_ in samples]
        yaw=np.unwrap([sample[2] for sample in samples])
        self.assertLessEqual(max(vertical),.100000001)
        self.assertLessEqual(max(horizontal),.150000001)
        self.assertLessEqual(max(abs(np.diff(yaw)/np.diff(times))),.150000001)

    def test_endpoint_is_held_with_zero_velocity(self):
        route=CandidateRoute([0.,0.,1.],[.2,.1,1.1],.3,.4,10.)
        self.assertFalse(route.finished(10.))
        self.assertFalse(route.finished(10.+route.duration-1e-6))
        self.assertTrue(route.finished(10.+route.duration))
        for t in (10.+route.duration,1000.):
            p,v,yaw=route.sample(t)
            np.testing.assert_allclose(p,[.2,.1,1.1],atol=1e-12)
            np.testing.assert_array_equal(v,np.zeros(3));self.assertAlmostEqual(yaw,.4)

    def test_yaw_takes_shortest_path_across_pi(self):
        route=CandidateRoute([0.,0.,1.],[.1,0.,1.],math.radians(179),math.radians(-179),0.)
        self.assertAlmostEqual(route.yaw_delta,math.radians(2.))
        middle=route.sample(route.duration*.5)[2]
        self.assertAlmostEqual(abs(middle),math.pi)
        self.assertAlmostEqual(wrap_yaw(route.sample(route.duration)[2]-math.radians(-179)),0.)

    def test_same_altitude_and_stationary_requests_are_valid(self):
        for goal in ([.2,0.,1.],[0.,0.,1.]):
            route=CandidateRoute([0.,0.,1.],goal,0.,0.,0.)
            self.assertGreaterEqual(route.climb_duration,4.)
            self.assertGreaterEqual(route.travel_duration,4.)
            for t in np.linspace(0.,route.duration,101):
                p,v,_=route.sample(t);self.assertEqual(p[2],1.);self.assertEqual(v[2],0.)

    def test_invalid_range_inputs_and_clock_are_rejected(self):
        for start,goal in [([0.,0.],[0.,0.,1.]),([0.,0.,1.],[0.,0.,.9]),
                ([0.,0.,1.],[2.01,0.,1.]),([0.,0.,float('nan')],[0.,0.,1.]),
                ([0.,0.,1.],[0.,float('inf'),1.])]:
            with self.subTest(start=start,goal=goal),self.assertRaises(ValueError):
                CandidateRoute(start,goal,0.,0.,0.)
        for yaw1,yaw2,begin in [(float('nan'),0.,0.),(0.,float('inf'),0.),(0.,0.,float('nan'))]:
            with self.assertRaises(ValueError):CandidateRoute([0.,0.,1.],[.1,0.,1.],yaw1,yaw2,begin)
        route=CandidateRoute([0.,0.,1.],[.1,0.,1.],0.,0.,10.)
        for now in (9.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):route.sample(now)
            with self.assertRaises(ValueError):route.finished(now)

    def test_route_copies_input_arrays(self):
        start=np.array([0.,0.,1.]);goal=np.array([.2,0.,1.2])
        route=CandidateRoute(start,goal,0.,0.,0.)
        start[:]=99.;goal[:]=99.
        np.testing.assert_array_equal(route.start,[0.,0.,1.])
        np.testing.assert_array_equal(route.goal,[.2,0.,1.2])


class NavigationFrameTests(unittest.TestCase):
    def test_identity_and_small_known_discrepancy_are_accepted(self):
        identity=np.eye(4);offset=np.eye(4);offset[:3,3]=[.03,.04,0.]
        angle=math.radians(3);offset[:3,:3]=[[math.cos(angle),-math.sin(angle),0.],
            [math.sin(angle),math.cos(angle),0.],[0.,0.,1.]]
        self.assertTrue(check_frame_agreement(identity,identity)['valid'])
        result=check_frame_agreement(identity,offset)
        self.assertTrue(result['valid']);self.assertAlmostEqual(result['position_error_m'],.05)
        self.assertAlmostEqual(result['rotation_error_deg'],3.)
        self.assertFalse(result['origin_correction_applied'])

    def test_large_origin_or_rotation_disagreement_is_refused_without_correction(self):
        for distance,angle in [(.081,0.),(0.,5.1)]:
            offset=np.eye(4);offset[0,3]=distance;a=math.radians(angle)
            offset[:3,:3]=[[math.cos(a),-math.sin(a),0.],[math.sin(a),math.cos(a),0.],[0.,0.,1.]]
            before=offset.copy();result=check_frame_agreement(np.eye(4),offset)
            self.assertFalse(result['valid']);self.assertFalse(result['origin_correction_applied'])
            np.testing.assert_array_equal(offset,before)

    def test_common_map_rotation_and_translation_preserve_errors(self):
        first=np.eye(4);second=np.eye(4);second[:3,3]=[.03,.04,0.]
        change=np.array([[0.,-1.,0.,2.],[1.,0.,0.,-3.],[0.,0.,1.,.7],[0.,0.,0.,1.]])
        a=check_frame_agreement(first,second);b=check_frame_agreement(change@first,change@second)
        self.assertAlmostEqual(a['position_error_m'],b['position_error_m'])
        self.assertAlmostEqual(a['rotation_error_deg'],b['rotation_error_deg'])

    def test_invalid_body_transforms_are_rejected(self):
        matrices=[np.eye(3)]
        for index,value in [((0,0),2.),((3,0),.1),((0,3),float('nan')),((2,2),-1.)]:
            matrix=np.eye(4);matrix[index]=value;matrices.append(matrix)
        for matrix in matrices:
            with self.subTest(matrix=matrix),self.assertRaises(ValueError):
                check_frame_agreement(matrix,np.eye(4))
            with self.assertRaises(ValueError):check_frame_agreement(np.eye(4),matrix)


if __name__=='__main__':unittest.main()
