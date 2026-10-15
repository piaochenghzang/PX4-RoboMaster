"""Pure coordinate and relative-velocity checks; no processes or commands."""
import math
from types import SimpleNamespace as NS
import unittest
import numpy as np
from hover_permit_probe import metrics, CUBE_REFERENCE, prepare_outcome


def vec(x):
    return NS(x=x[0], y=x[1], z=x[2])


def fixture():
    g = NS(position=vec([0, 0, 1]), orientation=NS(w=1, x=0, y=0, z=0))
    o = NS(position=vec(np.array([0, 0, 1])+CUBE_REFERENCE), orientation=g.orientation)
    truth = NS(link_names=['so101::gripper_link'], link_poses=[g], target_pose=o,
        gripper_velocity_valid=True, target_velocity_valid=True,
        gripper_linear_velocity_world=vec([0, 0, 0]),
        gripper_angular_velocity_world=vec([0, 0, 0]), target_linear_velocity_world=vec([0, 0, 0]))
    row = dict(truth=truth, actual_v=np.zeros(3), actual_w=np.zeros(3), body=[0, 0, 1, 1, 0, 0, 0])
    target = NS(hover=np.eye(4)); target.hover[2, 3] = 1
    return row, target


class HoverMetricsTests(unittest.TestCase):
    def test_prepare_revoked_before_poll_is_not_acceptance_or_inconsistent(self):
        base = dict(last_request_id=10, request_result=1, prepared=True, start_ready=True)
        self.assertEqual(prepare_outcome(base, 9), 'accepted')
        self.assertEqual(prepare_outcome(base, 10), 'pending')
        self.assertEqual(prepare_outcome(dict(base, request_result=2, prepared=False), 9), 'rejected')
        self.assertEqual(prepare_outcome(dict(base, request_result=3, prepared=False, start_ready=False), 9), 'revoked')
        with self.assertRaises(RuntimeError): prepare_outcome(dict(base, start_ready=False), 9)

    def test_matching_reference_is_not_link_origin(self):
        row, target = fixture()
        np.testing.assert_allclose(metrics(row, target), np.zeros(7), atol=1e-12)

    def test_rotating_link_reference_velocity(self):
        row, target = fixture(); truth = row['truth']
        truth.gripper_angular_velocity_world = vec([0, 1, 0])
        velocity = np.cross([0, 1, 0], CUBE_REFERENCE)
        self.assertAlmostEqual(metrics(row, target)[5], np.linalg.norm(velocity))
        truth.target_linear_velocity_world = vec(velocity)
        self.assertAlmostEqual(metrics(row, target)[5], 0)

    def test_missing_or_nonfinite_velocity_does_not_become_zero(self):
        row, target = fixture()
        for field in ('gripper_velocity_valid', 'target_velocity_valid'):
            setattr(row['truth'], field, False)
            with self.assertRaises(RuntimeError): metrics(row, target)
            setattr(row['truth'], field, True)
        row['truth'].target_linear_velocity_world = vec([math.nan, 0, 0])
        with self.assertRaises(RuntimeError): metrics(row, target)

    def test_orientation_error_not_euler_axis_difference(self):
        row, target = fixture()
        row['truth'].target_pose.orientation = NS(w=math.cos(.1), x=0, y=0, z=math.sin(.1))
        self.assertAlmostEqual(metrics(row, target)[6], .2)


if __name__ == '__main__':
    unittest.main()
