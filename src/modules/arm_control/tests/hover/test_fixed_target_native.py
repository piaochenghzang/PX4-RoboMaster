#!/usr/bin/env python3
"""Pure regression of fixed-point diagnostics and no-mutation evidence."""
import copy
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
from fixed_grasp_target import FixedGraspTarget, parse_native
from check_fixed_target_native import assert_unchanged, evidence_json
from known_target import pose
from native_client import NativeClient


def records(success=True):
    return dict(
        META=dict(read_only=True,workspace_valid=True,ik_evaluated=True),
        RESULT=dict(ik_success=success,joint_valid=True),
        ERROR=dict(error_m=.00003 if success else .04,seed_sequence=1),
        Q=dict(q=[-.07,-1.17,1.4,1.3,-1.57]),
        FK=dict(fk=[.21849,.0158,.0773]),
        QUAT=dict(quat=[1,0,0,0]))


def output(parts):
    return '\n'.join('INFO [arm_control] IK_CHECK_'+kind+' '+json.dumps(value,separators=(',',':'))
                     for kind,value in parts.items())


def state():
    return dict(arm_control_status=dict(feedback_valid=True,feedback_age=.02,
        joint_target=[-.07,-1.175,1.45,1.3,-1.57,.3],target_sequence=1,ik_status=0,ik_error=0),
        arm_grasp_status=dict(phase=0,ownership_locked=False,last_request_id=0),
        arm_approach_status=dict(phase=0,ownership_locked=False,last_request_id=0))


class FixedNativeTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads(Path(__file__).with_name('known_target_approach.json').read_text())
        self.fixed=FixedGraspTarget(self.config,dict(reference_gripper=[.0025,-.000218,-.088],
            tool_pose=[-.0079,-.000218121,-.0981274,0,np.pi,0]))

    def test_tool_reference_not_object_center(self):
        f=self.fixed
        np.testing.assert_allclose(f.gripper_world@np.r_[f.reference,1],f.known.object[:,3],atol=1e-12)
        np.testing.assert_allclose(f.tool_world,f.gripper_world@f.tool,atol=1e-12)
        self.assertGreater(np.linalg.norm(f.tool_world[:3,3]-f.known.object[:3,3]),.01)

    def test_full_pose_roundtrip_and_no_authority(self):
        rng=np.random.default_rng(101)
        for _ in range(20):
            q=rng.normal(size=4);q/=np.linalg.norm(q);arm=pose(rng.normal(size=3),q)
            np.testing.assert_allclose(arm@self.fixed.in_arm(arm),self.fixed.tool_world,atol=1e-12)
        preview=self.fixed.preview()
        self.assertEqual(preview['min_clearance_m'],.016)
        self.assertEqual(preview['contact_exemptions'],[])
        self.assertFalse(preview['motion_permitted'] or preview['closing_permitted'] or preview['ik_evaluated'])

    def test_complete_native_success(self):
        result=parse_native(output(records()))
        self.assertTrue(result['ik_success'])
        self.assertEqual(result['seed_sequence'],1)
        self.assertEqual(len(result['q']),5)

    def test_native_float32_limit_roundtrip(self):
        r=records();r['Q']['q'][4]=-1.57000005
        self.assertTrue(parse_native(output(r))['joint_valid'])
        r['Q']['q'][4]=-1.57001
        with self.assertRaises(ValueError): parse_native(output(r))

    def test_workspace_and_solver_rejections_distinct(self):
        r=records(False);self.assertFalse(parse_native(output(r))['ik_success'])
        short=dict(META=dict(read_only=True,workspace_valid=False,ik_evaluated=False),
            RESULT=dict(ik_success=False,joint_valid=False))
        result=parse_native(output(short))
        self.assertFalse(result['workspace_valid'] or result['ik_evaluated'])
        self.assertNotIn('q',result)

    def test_truncated_duplicate_missing_and_unknown_records_rejected(self):
        good=output(records())
        for bad in [good[:-3],good+'\n'+good,good.replace('IK_CHECK_Q','OTHER_Q'),
                    good+'\nIK_CHECK_UNKNOWN {}','',good.replace('"read_only":true','"read_only":false')]:
            with self.subTest(bad=bad[-40:]),self.assertRaises(ValueError):
                parse_native(bad)

    def test_success_finite_shape_limits_error_and_quaternion_checked(self):
        for kind,key,value in [('Q','q',[0]*4),('Q','q',[float('nan')]*5),
                ('Q','q',[0,0,0,0,1.6]),('FK','fk',[0,0,float('inf')]),
                ('QUAT','quat',[0,0,0,0]),('ERROR','error_m',.001),
                ('ERROR','seed_sequence',1.5),('RESULT','joint_valid',False)]:
            r=records();r[kind][key]=value
            with self.subTest(kind=kind,key=key),self.assertRaises(ValueError):
                parse_native(output(r))

    def test_short_records_fit_native_log(self):
        self.assertLess(max(len(line) for line in output(records()).splitlines()),127)

    def test_invariant_success_and_changed_target_rejected(self):
        before=state()
        self.assertEqual(assert_unchanged(before,copy.deepcopy(before))['joint_target_max_delta_rad'],0)
        for key,value in [('joint_target',[0]*6),('target_sequence',2),('ik_status',3),('ik_error',.1),
                          ('joint_target',[float('nan')]*6),('feedback_age',.3),('feedback_valid',False)]:
            after=copy.deepcopy(before);after['arm_control_status'][key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                assert_unchanged(before,after)

    def test_request_or_owner_change_rejected(self):
        for topic in ('arm_grasp_status','arm_approach_status'):
            for key,value in [('phase',1),('ownership_locked',True),('last_request_id',1)]:
                before=state();after=copy.deepcopy(before);after[topic][key]=value
                with self.subTest(topic=topic,key=key),self.assertRaises(RuntimeError):
                    assert_unchanged(before,after)

    def test_idle_sentinels_saved_but_feedback_still_rejected(self):
        self.assertEqual(evidence_json(dict(clearance=float('inf'))),dict(clearance='inf'))
        bad=state();bad['arm_control_status']['feedback_age']=float('inf')
        with self.assertRaises(RuntimeError): assert_unchanged(state(),bad)

    def test_native_client_construction_has_no_side_effect(self):
        with patch('native_client.sp.run') as run:
            NativeClient(Path('/tmp/not_executed'),{})
            run.assert_not_called()

    def test_shared_status_reader(self):
        client=NativeClient(Path('/tmp/not_executed'),{})
        with patch.object(client,'client',return_value='TOPIC: arm\n    timestamp: 123 (0.01 seconds ago)\n'
                '    joint_target: [0.1, -0.2]\n    feedback_valid: True\n'):
            result=client.status('arm_control_status')
        self.assertEqual(result,dict(timestamp=123,joint_target=[.1,-.2],feedback_valid=True))
        with patch.object(client,'client',return_value='never published'),self.assertRaises(RuntimeError):
            client.status('arm_control_status')


if __name__=='__main__':
    unittest.main()
