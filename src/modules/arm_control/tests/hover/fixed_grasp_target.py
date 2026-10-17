#!/usr/bin/env python3
"""Fixed cube20 coordinates and strict native IK diagnostics. No control transport."""
import argparse
import json
import math
import re
from pathlib import Path
import numpy as np
import yaml
from known_target import KnownTarget, inverse, mount_transform, vector

ROOT=Path(__file__).resolve().parents[5]
AUDIT=Path('/home/pcz/super_ws/src/super_px4_bridge/config/grasp_frame_audit.yaml')
MIN_CLEARANCE=.016  # unchanged, including BOTH fingers vs cube/support
RECORDS={
    'META': {'read_only', 'workspace_valid', 'ik_evaluated'},
    'RESULT': {'ik_success', 'joint_valid'},
    'ERROR': {'error_m', 'seed_sequence'}, 'Q': {'q'}, 'FK': {'fk'}, 'QUAT': {'quat'},
}


def parse_native(text):
    """Fail closed on truncated/duplicate/nonfinite records, not just IK failure."""
    parts={}
    def invalid_constant(value):
        raise ValueError('Nonfinite native JSON: '+value)
    for line in text.splitlines():
        if 'IK_CHECK_' not in line: continue
        match=re.search(r'IK_CHECK_([A-Z]+)\s+(\{[^\r\n]+\})',line)
        if not match: raise ValueError('Truncated native diagnostic record')
        kind,raw=match.groups()
        value=json.loads(raw,parse_constant=invalid_constant)
        if kind not in RECORDS or kind in parts or set(value)!=RECORDS[kind]:
            raise ValueError('Unexpected or duplicate native diagnostic record')
        parts[kind]=value
    if not {'META','RESULT'}<=parts.keys(): raise ValueError('Missing read-only native result')
    result=dict(parts['META'],**parts['RESULT'])
    if any(type(result[key]) is not bool for key in result) or not result['read_only']:
        raise ValueError('Invalid native diagnostic flags')
    if not result['workspace_valid'] and result['ik_evaluated']:
        raise ValueError('IK evaluated outside workspace')
    if result['joint_valid']:
        if not result['ik_evaluated'] or set(parts)!=set(RECORDS):
            raise ValueError('Missing native solution')
        for kind in ('ERROR','Q','FK','QUAT'): result.update(parts[kind])
        for key,n in [('q',5),('fk',3),('quat',4)]: vector(result[key],n,key)
        if abs(np.linalg.norm(result['quat'])-1)>1e-5:
            raise ValueError('Invalid native quaternion')
        # %.9g round-trips native float32. Compare in that same precision,
        # e.g. native -1.57f is printed as -1.57000005, not a real limit overrun.
        q=np.asarray(result['q'],dtype=np.float32)
        limit=np.asarray([1.57,1.2,1.5,1.57,1.57],dtype=np.float32)
        if np.any(q < -limit) or np.any(q > limit):
            raise ValueError('Native joint solution outside limits')
        error=float(result['error_m']); sequence=result['seed_sequence']
        if not math.isfinite(error) or error<0 or type(sequence) is not int or not 0<=sequence<=0xffffffff:
            raise ValueError('Invalid native error or sequence')
        if result['ik_success'] and error>=1e-4:
            raise ValueError('Invalid successful native IK result')
    elif result['ik_success'] or set(parts)!={'META','RESULT'}:
        raise ValueError('Invalid native rejection')
    return result


class FixedGraspTarget:
    def __init__(self,config,audit):
        self.known=KnownTarget(config)
        if self.known.name!='test_cube': raise ValueError('This fixed template is cube20 only')
        self.reference=vector(audit['reference_gripper'],3,'grasp reference')
        self.tool=mount_transform(audit['tool_pose'])
        # Convert the tested pad reference to the FK tool, not the cube center.
        # Position-only native IK does not certify arbitrary 6D orientation.
        self.gripper_world=self.known.object.copy()
        self.gripper_world[:3,3]-=self.known.object[:3,:3]@self.reference
        self.tool_world=self.gripper_world@self.tool
        self.planned_arm=self.known.hover@self.known.mount

    def in_arm(self,actual_arm):
        return inverse(actual_arm)@self.tool_world

    def preview(self):
        return dict(preview_only=True,motion_permitted=False,closing_permitted=False,
            nominal_only=True,ik_evaluated=False,min_clearance_m=MIN_CLEARANCE,contact_exemptions=[],
            object_world=self.known.object.tolist(),grasp_reference_gripper_m=self.reference.tolist(),
            fk_tool_offset_gripper=self.tool.tolist(),tool_target_world=self.tool_world.tolist(),
            planned_arm_pose_world=self.planned_arm.tolist(),
            tool_target_planned_arm=self.in_arm(self.planned_arm).tolist())


def load_target(path):
    return FixedGraspTarget(json.loads(Path(path).read_text()),yaml.safe_load(AUDIT.read_text()))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target',type=Path,default=Path(__file__).with_name('known_target_approach.json'))
    print(json.dumps(load_target(parser.parse_args().target).preview(),indent=2,allow_nan=False))
