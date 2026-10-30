"""Batch gap equivalence to the existing scalar geometry, not collision physics."""
import copy
import unittest
import numpy as np
from lio_box_gaps import box_gaps
from lio_tool_observer import rpy_transform
from perception.check_installation import box_gap


def scalar_pairs(first,second):
    return np.array([[box_gap(A,ah,B,bh) for _,B,bh in second] for _,A,ah in first])


class BatchBoxGapTests(unittest.TestCase):
    def setUp(self):
        self.box=('box',np.eye(4),np.array([.1,.2,.3]))

    def random_boxes(self,rng,count):
        rows=[]
        for i in range(count):
            T=rpy_transform([*rng.uniform(-2,2,3),*rng.uniform(-np.pi,np.pi,3)])
            rows.append((str(i),T,rng.uniform(.01,.4,3)))
        return rows

    def test_axis_aligned_known_distance_and_overlap(self):
        far=np.eye(4);far[:3,3]=[.7,.8,.9]
        second=[('far',far,np.array([.2,.1,.1])),self.box]
        result=box_gaps([self.box],second)
        self.assertEqual(result.shape,(1,2))
        self.assertAlmostEqual(result[0,0],np.linalg.norm([.4,.5,.5]),places=14)
        self.assertEqual(result[0,1],0.)

    def test_random_rotated_asymmetric_boxes_match_scalar_oracle(self):
        rng=np.random.RandomState(728)
        for _ in range(3):
            first=self.random_boxes(rng,10);second=self.random_boxes(rng,11)
            np.testing.assert_allclose(box_gaps(first,second),scalar_pairs(first,second),atol=2e-12,rtol=0)

    def test_near_contact_rotated_slender_boxes_match_scalar_oracle(self):
        first=[('thin',rpy_transform([0,0,0,.2,-.3,.7]),np.array([.3,.004,.01]))]
        second=[('other',rpy_transform([x,.03,-.01,-.4,.2,-.1]),np.array([.01,.15,.006]))
            for x in np.linspace(-.3,.3,21)]
        np.testing.assert_allclose(box_gaps(first,second),scalar_pairs(first,second),atol=2e-12,rtol=0)

    def test_symmetry_swaps_pair_axes(self):
        rng=np.random.RandomState(29)
        first=self.random_boxes(rng,4);second=self.random_boxes(rng,7)
        np.testing.assert_allclose(box_gaps(first,second),box_gaps(second,first).T,atol=2e-12,rtol=0)

    def test_common_world_transform_still_matches_same_three_frame_oracle(self):
        rng=np.random.RandomState(71)
        change=rpy_transform([2,-3,.7,.6,-.3,1.2])
        first=[(n,change@T,h) for n,T,h in self.random_boxes(rng,5)]
        second=[(n,change@T,h) for n,T,h in self.random_boxes(rng,6)]
        # A common rotation can change the world-axis contribution. Compare
        # the transformed scalar method, not an unsupported invariance claim.
        np.testing.assert_allclose(box_gaps(first,second),scalar_pairs(first,second),atol=2e-12,rtol=0)

    def test_inputs_are_not_mutated(self):
        rng=np.random.RandomState(8)
        first=self.random_boxes(rng,3);second=self.random_boxes(rng,2)
        before=copy.deepcopy((first,second));box_gaps(first,second)
        for old,new in zip(before,(first,second)):
            for (name,T,h),(after_name,after_T,after_h) in zip(old,new):
                self.assertEqual(name,after_name)
                np.testing.assert_array_equal(T,after_T);np.testing.assert_array_equal(h,after_h)

    def test_empty_or_malformed_collections_are_rejected(self):
        for rows in ([],[('missing_size',np.eye(4))],
                [('wrong_pose',np.eye(3),np.ones(3))],
                [('wrong_half',np.eye(4),np.ones(2))]):
            with self.subTest(rows=rows),self.assertRaises(ValueError):box_gaps(rows,[self.box])
            with self.assertRaises(ValueError):box_gaps([self.box],rows)

    def test_nonfinite_or_nonpositive_geometry_is_rejected(self):
        for half in ([0,.1,.1],[-.1,.1,.1],[.1,float('nan'),.1],[.1,.1,float('inf')]):
            with self.subTest(half=half),self.assertRaises(ValueError):
                box_gaps([('bad',np.eye(4),half)],[self.box])
        for value in (float('nan'),float('inf')):
            T=np.eye(4);T[0,3]=value
            with self.assertRaises(ValueError):box_gaps([self.box],[('bad',T,np.ones(3))])

    def test_nonrigid_transforms_are_rejected(self):
        matrices=[]
        for index,value in (((3,0),.01),((3,3),.9),((0,0),2.),((2,2),-1.)):
            T=np.eye(4);T[index]=value;matrices.append(T)
        T=np.eye(4);T[0,1]=.01;matrices.append(T)
        for T in matrices:
            with self.subTest(T=T),self.assertRaises(ValueError):box_gaps([('bad',T,np.ones(3))],[self.box])


if __name__=='__main__':unittest.main()
