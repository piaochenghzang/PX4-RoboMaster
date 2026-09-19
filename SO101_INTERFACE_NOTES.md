# SO101 transport and control hardening (2026-09-19)

## Implemented
- Gazebo bridge sends feedback only after every joint supplies a new sample; cached samples cannot be retransmitted indefinitely.
- Non-finite joint positions/velocities are rejected at the PX4 receiver. The ordinary MAVLink receiver now sets joint_count=6 too.
- Current is unmeasured and reported as NaN, not a fabricated zero. Consumers must handle this explicitly.
- arm_control stops advancing trajectories and publishing commands after its existing 1-second simulation-time feedback timeout.
- arm_control status reports consumed valid feedback count, published command count and feedback age. Counts are at the controller's 10 Hz polling cadence, NOT wire packet counts.
- Controller controlSource selects ros or gazebo exclusively. This model explicitly uses gazebo. For standalone ROS commands change all six controller entries to ros and reload the model.
- Gazebo command loss for more than 1 simulation second holds the measured position as a position target once. It does not remove gravity or guarantee zero tracking error.
- Folded wrist target is -1.570 rad in PX4 and SDF. Conservative PX4 limits remain; shoulder_lift/wrist_flex/gripper speed limits and gripper lower angle are tightened to respect SDF constraints.
- The previously tested 1 ms physics step / 1000 Hz update rate remains enabled in the shared empty.world.

## Files
PX4: src/modules/arm_control/arm_control.{cpp,hpp}, src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp, src/modules/mavlink/mavlink_receiver.cpp.
Gazebo submodule: src/gazebo_mavlink_interface.cpp, models/so101/so101.sdf.
Separate ROS workspace: /home/pcz/super_ws/src/so101_gazebo/src/joint_position_controller.cpp.
The ROS workspace change is not included in the PX4 repository or its Gazebo submodule.

## Verification
- PX4 SITL and Gazebo Classic plugins compiled successfully.
- catkin_make --pkg so101_gazebo -j2 passed.
- git diff --check passed for PX4 and the Gazebo submodule.
- Headless Transport-only run: valid feedback, initialized commands, counters advancing.
- Stopped arm_control; Gazebo feedback continued, with low measured velocities. Restarted arm_control; feedback and commands recovered.
- After restart, sampled wrist velocity -0.00072 rad/s and gripper -0.00066 rad/s; wrist angle -1.57339 rad.
- ROS joint-state publication was not tested in this Transport-only run (ROS API was not initialized).

## Remaining limitations
- Elbow steady error persists: command 1.450 rad versus feedback about 1.307 rad.
- Holding a measured angle still has gravity-related tracking error; do not describe this as a mechanical brake.
- Feedback wire format still lacks source timestamps, sequence IDs and validity bits. New-sample gating is not an exact simultaneous six-joint snapshot and does not measure end-to-end latency.
- Feedback fault injection, ROS-mode runtime isolation and planning trajectory tests remain to be performed.
- Command timeout advances with simulation time; pausing physics intentionally pauses this timeout.
- No autonomous recovery/replan policy, collision checking or gravity compensation was added.
