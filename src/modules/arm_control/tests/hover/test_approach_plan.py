"""Task binding and actual-arm coordinate tests; no simulation or commands."""
import json
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from known_target import KnownTarget, pose
from hover_approach_probe import ApproachPlan,validate_swept_bounds


class ApproachPlanTests(unittest.TestCase):
    def test_swept_joint_bounds_cover_actual_sdf_collision_tree(self):
        result=validate_swept_bounds(Path(__file__).resolve().parents[5])
        np.testing.assert_allclose(result['derived_m'],[.502506970,.437731749,.321731727,.186731542,.123006979],atol=1e-9)
        self.assertTrue(np.all(np.asarray(result['native_upper_bounds_m'])>=result['derived_m']))

    def setUp(self):
        self.config=json.loads(Path(__file__).with_name('known_target_approach.json').read_text())
        self.truth=NS(source_instance='synthetic_math_only',world_epoch=1,vehicle_id=10,target_id=20)

    def test_planned_pregrasp_world_and_standoff(self):
        target=KnownTarget(self.config); plan=ApproachPlan(target,self.truth)
        np.testing.assert_allclose(plan.pre_world,[.576721237,.065129817,1.75],atol=2e-7)
        self.assertGreater(np.linalg.norm(plan.pre_world-target.object[:3,3]),.07)
        self.assertLess(np.linalg.norm(plan.pre_world-target.object[:3,3]),.072)
        np.testing.assert_allclose(plan.waypoints_world[0],[.486721237,.105129813,1.82],atol=2e-7)
        self.assertEqual(len(plan.waypoints_world),2)

    def test_approach_fixture_moves_hover_not_object_or_limits(self):
        old=json.loads(Path(__file__).with_name('known_target_high.json').read_text())
        previous=KnownTarget(old); current=KnownTarget(self.config)
        np.testing.assert_allclose(current.object,previous.object,atol=1e-12)
        np.testing.assert_allclose(current.mount,previous.mount,atol=1e-12)
        np.testing.assert_allclose(current.hover[:3,3]-previous.hover[:3,3],[-.03,0,0],atol=1e-12)

    def test_uses_actual_arm_full_pose_not_nominal_body(self):
        plan=ApproachPlan(KnownTarget(self.config),self.truth)
        q=np.array([.8,.2,-.1,.3]); q=q/np.linalg.norm(q)
        arm=[.1,-.2,1.5,*q]
        point=plan.base_goal(dict(arm=arm))
        np.testing.assert_allclose((pose(arm[:3],arm[3:]) @ np.r_[point,1])[:3],plan.pre_world,atol=1e-12)

    def test_identity_and_target_changes_cannot_reuse_binding(self):
        target=KnownTarget(self.config); a=ApproachPlan(target,self.truth)
        for field in ('source_instance','world_epoch','vehicle_id','target_id'):
            changed=NS(**vars(self.truth)); setattr(changed,field,'changed' if field=='source_instance' else 100)
            b=ApproachPlan(target,changed); self.assertNotEqual(a.ids,b.ids)
        self.config['object_position_world_m'][0]+=.01
        b=ApproachPlan(KnownTarget(self.config),self.truth)
        self.assertNotEqual(a.plan,b.plan)

    def test_matches_all_four_ids_and_rejects_missing(self):
        a=ApproachPlan(KnownTarget(self.config),self.truth)
        s=NS(task_id=a.task,plan_id=a.plan,target_id=a.target,scene_id=a.scene)
        self.assertTrue(a.matches(s)); self.assertFalse(a.matches(None))
        s.plan_id+=1; self.assertFalse(a.matches(s))


if __name__=='__main__': unittest.main()
