#!/usr/bin/env python3
"""Known-target preview only. No ROS, flight or joint commands."""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def vector(value, length, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(name + ' requires ' + str(length) + ' finite numbers')
    return result


def rotation(q):
    q = vector(q, 4, 'quaternion_wxyz')
    norm = float(np.linalg.norm(q))
    if norm < 1e-9 or abs(norm - 1) > .01:
        raise ValueError('quaternion must have unit length')
    w, x, y, z = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose(position, quaternion):
    out = np.eye(4)
    out[:3, 3] = vector(position, 3, 'position_m')
    out[:3, :3] = rotation(quaternion)
    return out


def mount_transform(values):
    x, y, z, roll, pitch, yaw = vector(values, 6, 'arm_mount_xyz_rpy')
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    out = np.eye(4)
    out[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                   [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    out[:3, 3] = [x, y, z]
    return out


def inverse(transform):
    out = np.eye(4)
    out[:3, :3] = transform[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ transform[:3, 3]
    return out


class KnownTarget:
    def __init__(self, config):
        if config.get('frame') != 'gazebo_world':
            raise ValueError('requires explicit gazebo_world frame; map/world aliases are not accepted')
        if not isinstance(config.get('target_name'), str) or not config['target_name'].strip():
            raise ValueError('target_name is required')
        self.name = config['target_name']
        self.object = pose(config['object_position_world_m'], config['object_quaternion_world_wxyz'])
        self.mount = mount_transform(config['arm_mount_xyz_rpy'])
        self.reference = vector(config['nominal_object_center_arm_m'], 3, 'nominal_object_center_arm_m')
        yaw = float(config['body_hover_yaw_world_rad'])
        if not math.isfinite(yaw):
            raise ValueError('hover yaw must be finite')
        self.yaw = yaw
        self.hover = np.eye(4)
        self.hover[:3, :3] = rotation([math.cos(yaw/2), 0, 0, math.sin(yaw/2)])
        offset = self.mount @ np.r_[self.reference, 1]
        self.hover[:3, 3] = self.object[:3, 3] - self.hover[:3, :3] @ offset[:3]
        if self.hover[2, 3] <= 0:
            raise ValueError('derived hover body origin is not above world ground')
        self.nominal_object_arm = inverse(self.hover @ self.mount) @ self.object

    def preview(self, snapshot=None):
        result = dict(preview_only=True, flight_commands_sent=False, joint_commands_sent=False,
            close_permitted=False, flight_ready=False, ik_reachability_tested=False,
            target_name=self.name, frame='gazebo_world', object_pose_world=self.object.tolist(),
            planned_body_hover_pose_world=self.hover.tolist(), planned_body_hover_yaw_world_rad=self.yaw,
            nominal_object_pose_arm=self.nominal_object_arm.tolist(),
            nominal_basis='PLANNED_BODY_POSE_NOT_MEASUREMENT',
            grasp_reference_scope='cube20 collision-reference calibration; not arbitrary TCP/IK target',
            mavros_goal_generated=False)
        if snapshot is not None:
            if snapshot.get('frame') != 'gazebo_world':
                raise ValueError('snapshot requires actual world-relative poses, not MAVROS local odometry')
            stamp = float(snapshot['sample_time_s'])
            if not math.isfinite(stamp) or stamp <= 0 or not isinstance(snapshot['source_instance'], str) or not snapshot['source_instance']:
                raise ValueError('snapshot source identity/time invalid')
            body_values = vector(snapshot['body_pose_world'], 7, 'body_pose_world xyz+wxyz')
            arm_values = vector(snapshot['arm_pose_world'], 7, 'arm_pose_world xyz+wxyz')
            body = pose(body_values[:3], body_values[3:])
            arm = pose(arm_values[:3], arm_values[3:])
            result.update(recorded_sample_time_s=stamp, source_instance=snapshot['source_instance'],
                actual_arm_pose_source='RECORDED_GAZEBO_SNAPSHOT',
                known_target_pose_in_recorded_arm=(inverse(arm) @ self.object).tolist(),
                planned_hover_displacement_world_m=(self.hover[:3, 3]-body[:3, 3]).tolist(),
                mount_translation_residual_m=float(np.linalg.norm((body @ self.mount)[:3, 3]-arm[:3, 3])),
                snapshot_runtime_freshness_checked=False, known_target_actually_observed=False)
        return result


def quintic_segment(start, goal, elapsed, duration):
    start = vector(start, 3, 'trajectory_start')
    goal = vector(goal, 3, 'trajectory_goal')
    if not math.isfinite(elapsed) or not math.isfinite(duration) or duration <= 0:
        raise ValueError('trajectory time invalid')
    u = float(np.clip(elapsed/duration, 0, 1))
    s = 10*u**3 - 15*u**4 + 6*u**5
    ds = (30*u**2 - 60*u**3 + 30*u**4)/duration
    return start + s*(goal-start), ds*(goal-start)


def local_motion(world_position, world_velocity, world_yaw, body_world_values,
                 body_local_values, actual_velocity_world, estimated_velocity_body):
    """Live SIM truth control transform, NOT a navigation TF or EKF injection."""
    gw = vector(body_world_values, 7, 'body_world_xyz_wxyz')
    lb = vector(body_local_values, 7, 'body_local_xyz_wxyz')
    world = pose(gw[:3], gw[3:])
    local = pose(lb[:3], lb[3:])
    transform = local @ inverse(world)
    position = transform @ np.r_[vector(world_position,3,'world_position'),1]
    # Odometry twist is body FLU. Cancel estimated velocity offset only for this
    # truth-localized SITL demo; do not pretend the estimator became accurate.
    estimated = local[:3,:3] @ vector(estimated_velocity_body,3,'estimated_body_velocity')
    velocity = transform[:3,:3] @ (vector(world_velocity,3,'world_velocity') -
        vector(actual_velocity_world,3,'actual_world_velocity')) + estimated
    if not math.isfinite(world_yaw):
        raise ValueError('world yaw invalid')
    desired_rotation = transform[:3,:3] @ rotation([math.cos(world_yaw/2),0,0,math.sin(world_yaw/2)])
    yaw = math.atan2(desired_rotation[1,0],desired_rotation[0,0])
    return position[:3], velocity, yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', type=Path, default=Path(__file__).with_name('known_target.json'))
    parser.add_argument('--snapshot', type=Path, help='Saved actual world body/arm pose snapshot; never restamped')
    args = parser.parse_args()
    try:
        target = KnownTarget(json.loads(args.target.read_text()))
        snapshot = json.loads(args.snapshot.read_text()) if args.snapshot else None
        print(json.dumps(target.preview(snapshot), indent=2, allow_nan=False))
    except (KeyError, TypeError, ValueError, OSError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
