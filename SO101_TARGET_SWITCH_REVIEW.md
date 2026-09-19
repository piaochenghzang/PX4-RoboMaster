# SO101 end-effector target switching (2026-09-19)

## Scope

WSL Ubuntu-20.04, headless yhang550 SITL, unloaded arm, vehicle unarmed.
ROS package changes remain uncommitted at the user's request.

## Test entry

`arm_control target x y z gripper` publishes arm_cartesian_setpoint through
uORB rather than modifying the work-queue IK state directly. Coordinates
are metres in SO101 base_link; gripper is radians. The publication persists
after the shell command exits. Numeric arguments must be finite and the
module must be running. Publication success is not IK acceptance or arrival.

## Observations

| Target | Accepted sequence | Position error | position_reached |
| --- | ---: | ---: | --- |
| Initial folded pose | 0 | 0.23 mm | true |
| A intermediate: 0.10920, 0.01249, 0.01475 | 1 | 1.43 mm | true |
| A: 0.11000, 0.01250, 0.02000 | 2 | 0.62 mm | true |
| B: 0.12000, 0.01500, 0.03000 | 3 | 2.41 mm initially; 1.80 mm later | true |
| Return to A | 4 | 0.50 mm | true |

All commands kept gripper=0.3 rad. The first A request was capped by the
existing 20 mm Cartesian step limit, so A was sent twice. This is a limit
per accepted message, not an autonomous trajectory to the original goal.
The reached flag refers to ee_target, not the original requested point.
A future planner must compare these explicitly; this behavior must not be
treated as final-goal success for a larger requested move.

The snapshots establish arrival at each accepted point, not the exact
duration of the false-to-true transition. Sampled contacts after the
return to A were only the two landing skids against asphalt. This short
sample is not a full collision-free-path proof.

## Remaining validation

Finger contact, contact force, object slip, lift and payload retention
have not passed testing in this run. No grasp success is claimed.
Current SDF finger friction coefficients alone cannot establish a stable
grasp. A known object, calibrated finger contact geometry and measured
object motion are required before payload testing.
