#!/usr/bin/env python3
"""Conservative swept-clearance bounds and native workspace recovery rules."""
import unittest

import numpy as np

from unfold_precheck import certify_segment, workspace_transition


class SweptClearanceTests(unittest.TestCase):
    def test_endpoint_bound_certifies_a_sufficiently_distant_segment(self):
        result = certify_segment([0.], [.1], [1.], lambda q: .2)
        self.assertTrue(result['certified'])
        self.assertEqual(result['samples'], 2)
        self.assertAlmostEqual(result['swept_lower_bound_m'], .1)

    def test_safe_endpoints_do_not_hide_a_midpoint_collision(self):
        def clearance(q):
            # A physical 1-Lipschitz distance profile: both endpoints are
            # distant, but the middle reaches the obstacle.
            return .01 + abs(q[0] - .5)

        result = certify_segment([0.], [1.], [1.], clearance)
        self.assertFalse(result['certified'])
        self.assertLess(result['minimum_sampled_m'], .016)

    def test_recursive_bound_can_certify_a_safe_segment(self):
        result = certify_segment([0.], [.1], [1.], lambda q: .03, max_depth=4)
        self.assertTrue(result['certified'])
        self.assertGreater(result['samples'], 2)
        self.assertGreaterEqual(result['swept_lower_bound_m'], .016)

    def test_depth_limit_rejects_an_unproven_segment(self):
        result = certify_segment([0.], [.1], [1.], lambda q: .03, max_depth=0)
        self.assertFalse(result['certified'])
        self.assertIsNone(result['swept_lower_bound_m'])

    def test_an_endpoint_below_the_limit_is_rejected(self):
        result = certify_segment([0.], [.01], [1.], lambda q: .015)
        self.assertFalse(result['certified'])

    def test_joint_travel_bounds_add_across_all_axes(self):
        result = certify_segment([0., 0.], [.1, .2], [1., 2.], lambda q: .6)
        self.assertTrue(result['certified'])
        self.assertAlmostEqual(result['swept_lower_bound_m'], .1)

    def test_invalid_transition_is_rejected_even_with_large_clearance(self):
        result = certify_segment([0.], [.01], [1.], lambda q: 1.,
                                 transition_ok=lambda a, b: False)
        self.assertFalse(result['certified'])
        self.assertFalse(result['transition_valid'])

    def test_callbacks_cannot_mutate_caller_inputs(self):
        start = np.array([0., .01])
        end = np.array([.01, .02])
        radii = np.array([1., 2.])
        saved = [value.copy() for value in (start, end, radii)]

        def clearance(q):
            q[:] = 123.
            return 1.

        def transition(a, b):
            a[:] = -123.
            b[:] = 123.
            return True

        result = certify_segment(start, end, radii, clearance, transition)
        self.assertTrue(result['certified'])
        for actual, expected in zip((start, end, radii), saved):
            np.testing.assert_array_equal(actual, expected)

    def test_invalid_inputs_and_nonfinite_distance_are_rejected(self):
        invalid = [
            ([], [], []),
            ([0.], [0., 1.], [1.]),
            ([0.], [.1], [1., 2.]),
            ([float('nan')], [.1], [1.]),
            ([0.], [float('inf')], [1.]),
            ([0.], [.1], [-1.]),
            ([0.], [.1], [float('nan')]),
        ]
        for args in invalid:
            with self.subTest(args=args), self.assertRaises(ValueError):
                certify_segment(*args, lambda q: 1.)
        for options in [dict(minimum=0.), dict(minimum=float('nan')),
                        dict(max_depth=-1), dict(max_depth=17), dict(max_depth=.5)]:
            with self.subTest(options=options), self.assertRaises(ValueError):
                certify_segment([0.], [.1], [1.], lambda q: 1., **options)
        for distance in [float('nan'), float('inf')]:
            with self.subTest(distance=distance), self.assertRaises(ValueError):
                certify_segment([0.], [.1], [1.], lambda q: distance)


class WorkspaceRecoveryTests(unittest.TestCase):
    def test_folded_z_can_recover_monotonically_before_entering_workspace(self):
        self.assertTrue(workspace_transition([.1, 0., -.005], [.1, 0., -.004]))
        self.assertTrue(workspace_transition([.1, 0., .019], [.1, 0., .020]))

    def test_recovery_cannot_move_farther_out_or_make_no_outside_progress(self):
        self.assertFalse(workspace_transition([.1, 0., -.005], [.1, 0., -.006]))
        self.assertFalse(workspace_transition([.1, 0., -.005], [.101, 0., -.005]))

    def test_recovery_cannot_create_a_new_violation_on_another_axis(self):
        self.assertFalse(workspace_transition([.051, 0., -.005], [.049, 0., -.004]))

    def test_upper_boundary_recovers_toward_workspace(self):
        self.assertTrue(workspace_transition([.401, 0., .1], [.400, 0., .1]))
        self.assertFalse(workspace_transition([.401, 0., .1], [.402, 0., .1]))

    def test_inside_workspace_must_remain_inside(self):
        self.assertTrue(workspace_transition([.1, 0., .1], [.101, 0., .1]))
        self.assertFalse(workspace_transition([.051, 0., .1], [.049, 0., .1]))

    def test_cartesian_step_limit_applies_even_to_recovery(self):
        self.assertFalse(workspace_transition([.1, 0., -.005], [.1, 0., .025]))
        self.assertFalse(workspace_transition([.1, 0., .1], [.121, 0., .1]))

    def test_nonfinite_workspace_coordinates_are_rejected(self):
        for start, end in [([.1, 0., float('nan')], [.1, 0., .1]),
                           ([.1, 0., .1], [.1, 0., float('inf')]),
                           ([.1, 0.], [.1, 0., .1])]:
            with self.subTest(start=start, end=end), self.assertRaises(ValueError):
                workspace_transition(start, end)


class ProductionFoldRecoveryTests(unittest.TestCase):
    def test_small_wrist_only_step_reproduces_outward_arc(self):
        from fixed_demo_plan import START
        from check_approach_route import fk
        next_q=START.copy();next_q[3]-=.005
        self.assertLess(fk(next_q)[2],fk(START)[2])
        self.assertFalse(workspace_transition(fk(START),fk(next_q)))

    def test_budget_scaled_coupled_first_steps_recover_inward(self):
        from fixed_demo_plan import START
        from check_approach_route import fk
        for scale in (.01,.05,.1,.25,.5,1.):
            next_q=START+scale*np.array([0.,0.,-.001,-.02,0.])
            with self.subTest(scale=scale):
                self.assertTrue(workspace_transition(fk(START),fk(next_q)))

    def test_coupled_unfold_restores_exact_original_posture_inside_workspace(self):
        from fixed_demo_plan import START,UNFOLDED
        from check_approach_route import fk
        for scale in (.05,.25,1.):
            q=START.copy()
            for _ in range(4000):
                goal=UNFOLDED.copy()
                if fk(q)[2]<.022:goal[2]-=.001
                if np.max(np.abs(goal-q))<.001:break
                following=q+scale*np.clip(goal-q,-.02,.02)
                self.assertTrue(workspace_transition(fk(q),fk(following)))
                q=following
            else:self.fail('Coupled unfold did not converge')
            self.assertLess(np.max(np.abs(q-UNFOLDED)),.001)
            self.assertAlmostEqual(q[2],UNFOLDED[2],places=6)


if __name__ == '__main__':
    unittest.main()
