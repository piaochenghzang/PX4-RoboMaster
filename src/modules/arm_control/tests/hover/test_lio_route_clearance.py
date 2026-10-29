"""LIO return clearance certificates, including sweep and model rejection."""
import copy
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from lio_candidate_route import CandidateRoute
from lio_route_clearance import body_collision_boxes,check_route


class RouteClearanceTests(unittest.TestCase):
    def setUp(self):
        self.boxes=[('body',np.eye(4),np.array([.05,.05,.05]))]
        self.route=CandidateRoute([0,0,1],[1,0,1.1],0.,0.,10.)

    def check(self,points,**kwargs):
        return check_route(self.route,self.boxes,points,**kwargs)

    def test_distant_returns_certify_with_an_explicit_limited_scope(self):
        result=self.check([[0.,3.,1.],[1.,3.,1.]])
        self.assertTrue(result['valid'])
        self.assertGreater(result['min_clearance_m'],.016)
        self.assertEqual(result['collision_boxes'],1)
        self.assertFalse(result['unknown_space_certified'])
        self.assertFalse(result['truth_inputs'])
        self.assertEqual(result['scope'],'STATIC_OBSERVED_LIO_RETURNS_ONLY')

    def test_an_obstacle_between_route_endpoints_is_rejected(self):
        # Both whole-route endpoints are clear; the horizontal leg crosses it.
        result=self.check([[.5,0.,1.1]])
        self.assertFalse(result['valid'])
        self.assertEqual(result['min_sampled_observed_clearance_m'],0.)

    def test_margins_include_tracking_points_and_attitude(self):
        result=self.check([[0.,3.,1.]],tracking_margin=.04,point_margin=.03)
        expected_attitude=2*np.linalg.norm([.05]*3)*np.sin(np.deg2rad(2.5))
        self.assertAlmostEqual(result['attitude_margin_m'],expected_attitude)
        self.assertLessEqual(result['min_clearance_m'],.5-.04-.03-expected_attitude)
        self.assertLessEqual(result['maximum_interval_travel_bound_m'],.020001)

    def test_near_clearance_must_still_cover_the_reserved_motion(self):
        nominal=self.check([[.5,.12,1.1]],tracking_margin=0.,point_margin=0.)
        reserved=self.check([[.5,.12,1.1]],tracking_margin=.06,point_margin=.02)
        self.assertTrue(nominal['valid'])
        self.assertFalse(reserved['valid'])

    def test_obb_rotation_is_used_instead_of_axis_aligned_size(self):
        box=np.eye(4);box[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        boxes=[('slender',box,np.array([.2,.01,.01]))]
        result=check_route(self.route,boxes,[[0.,.15,1.]])
        self.assertFalse(result['valid'])
        self.assertEqual(result['min_sampled_observed_clearance_m'],0.)

    def test_shortest_yaw_sweep_is_included_across_pi(self):
        route=CandidateRoute([0,0,1],[.1,0,1],3.13,-3.13,1.)
        result=check_route(route,self.boxes,[[3.,0.,1.]])
        self.assertTrue(result['valid'])
        self.assertLess(result['maximum_interval_travel_bound_m'],.02)

    def test_invalid_cloud_box_and_margins_are_rejected(self):
        for points in ([],[[0,0]],[[0,0,float('nan')]],[[0,0,float('inf')]]):
            with self.subTest(points=points),self.assertRaises(ValueError):self.check(points)
        for options in (dict(minimum=0),dict(tracking_margin=-.1),dict(point_margin=float('nan'))):
            with self.subTest(options=options),self.assertRaises(ValueError):self.check([[3,0,1]],**options)
        for boxes in ([],[('bad',np.eye(3),np.ones(3))],[('bad',np.eye(4),np.array([1,1,-1]))]):
            with self.subTest(boxes=boxes),self.assertRaises(ValueError):
                check_route(self.route,boxes,[[3,0,1]])

    def test_inputs_are_not_mutated(self):
        points=np.array([[3.,0.,1.]])
        original=copy.deepcopy((self.boxes,points,self.route.__dict__))
        self.check(points)
        np.testing.assert_array_equal(points,original[1])
        for before,after in zip(original[0],self.boxes):
            self.assertEqual(before[0],after[0]);np.testing.assert_array_equal(before[1],after[1]);np.testing.assert_array_equal(before[2],after[2])
        for key in ('start','goal','top'):np.testing.assert_array_equal(self.route.__dict__[key],original[2][key])


class CollisionGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from check_approach_route import links,joints
        cls.config=dict(mount_pose=[0,0,-.05,np.pi,0,0])
        cls.q=np.array([-.07,-1.175,1.45,1.3,-1.57,.3])
        root=ET.Element('sdf');world=ET.SubElement(root,'world');model=ET.SubElement(world,'model',name='yhang550')
        ET.SubElement(model,'pose').text='9 8 7 0 0 1'  # Must never enter body geometry.
        frame=ET.SubElement(model,'frame',name='so101::__model__')
        ET.SubElement(frame,'pose').text='0 0 -.05 3.141592653589793 0 0'
        base=ET.SubElement(model,'link',name='yhang550_base::base_link')
        def box(link,name,size):
            c=ET.SubElement(link,'collision',name=name)
            ET.SubElement(ET.SubElement(ET.SubElement(c,'geometry'),'box'),'size').text=size
        box(base,'body','.2 .2 .05')
        rotor=ET.SubElement(model,'link',name='yhang550_base::rotor_0')
        ET.SubElement(rotor,'pose').text='.2 0 .06 0 0 0';box(rotor,'prop','.28 .02 .001')
        joint=ET.SubElement(model,'joint',name='rotor',type='revolute')
        ET.SubElement(joint,'parent').text='yhang550_base::base_link';ET.SubElement(joint,'child').text='yhang550_base::rotor_0'
        ET.SubElement(ET.SubElement(joint,'axis'),'xyz').text='0 0 1'
        arm_mount=ET.SubElement(model,'joint',name='arm_mount',type='fixed')
        ET.SubElement(arm_mount,'parent').text='yhang550_base::base_link';ET.SubElement(arm_mount,'child').text='so101::base_link'
        for name,link in links.items():
            expanded=copy.deepcopy(link);expanded.set('name','so101::'+name)
            if name=='base_link':ET.SubElement(expanded,'pose',relative_to='so101::__model__').text='0 0 0 0 0 0'
            elif expanded.find('pose') is not None:
                expanded.find('pose').set('relative_to','so101::'+expanded.find('pose').get('relative_to'))
            model.append(expanded)
        for source in joints:
            expanded=copy.deepcopy(source);expanded.set('name','so101::'+source.get('name'))
            for key in ('parent','child'):expanded.find(key).text='so101::'+expanded.findtext(key)
            expanded.find('pose').set('relative_to','so101::'+source.find('pose').get('relative_to'))
            model.append(expanded)
        cls.fixture=root

    def extract(self,root=None):
        with tempfile.TemporaryDirectory() as folder:
            file=Path(folder)/'scene.world';ET.ElementTree(self.fixture if root is None else root).write(file)
            return body_collision_boxes(file,self.config,self.q)

    def test_rotor_includes_all_spin_angles_and_world_pose_is_ignored(self):
        boxes=self.extract();rotor=next(b for b in boxes if 'prop' in b[0]);body=boxes[0]
        np.testing.assert_allclose(body[1],np.eye(4))
        np.testing.assert_allclose(rotor[2],[np.hypot(.14,.01)]*2+[.0005])
        np.testing.assert_allclose(rotor[1][:3,3],[.2,0,.06])
        self.assertGreater(len(boxes),2)

    def test_unknown_collision_mesh_and_arm_geometry_change_are_rejected(self):
        root=copy.deepcopy(self.fixture);model=root.find('world/model')
        unknown=ET.SubElement(model,'link',name='unmodelled')
        collision=ET.SubElement(unknown,'collision',name='new')
        ET.SubElement(ET.SubElement(ET.SubElement(collision,'geometry'),'box'),'size').text='.1 .1 .1'
        with self.assertRaises(ValueError):self.extract(root)
        root=copy.deepcopy(self.fixture)
        geometry=root.find('world/model/link/collision/geometry');geometry.clear();ET.SubElement(geometry,'mesh')
        with self.assertRaises(ValueError):self.extract(root)
        root=copy.deepcopy(self.fixture)
        root.find("world/model/link[@name='so101::upper_arm_link']/collision/geometry/box/size").text='1 1 1'
        with self.assertRaises(ValueError):self.extract(root)

    def test_unsupported_frame_is_rejected(self):
        root=copy.deepcopy(self.fixture)
        col=root.find('world/model/link/collision');ET.SubElement(col,'pose',relative_to='world').text='0 0 0 0 0 0'
        with self.assertRaises(ValueError):self.extract(root)

    def test_changed_arm_joint_axis_origin_or_child_frame_is_rejected(self):
        changes=[("joint[@name='so101::wrist_roll']/axis/xyz",'1 0 0'),
                 ("joint[@name='so101::wrist_roll']/pose",'0 -.07 .0181 1.5708 .04868 3.14159'),
                 ("link[@name='so101::gripper_link']/pose",'.01 0 0 0 0 0')]
        for selector,value in changes:
            root=copy.deepcopy(self.fixture);root.find('world/model/'+selector).text=value
            with self.subTest(selector=selector),self.assertRaises(ValueError):self.extract(root)

    def test_rotor_joint_frame_rotation_cannot_change_the_spin_axis(self):
        root=copy.deepcopy(self.fixture)
        ET.SubElement(root.find("world/model/joint[@name='rotor']"),'pose').text='0 0 0 .2 0 0'
        with self.assertRaises(ValueError):self.extract(root)

    def test_missing_link_pose_cannot_fall_back_to_the_wrong_model_frame(self):
        for name in ('so101::base_link','so101::gripper_link'):
            root=copy.deepcopy(self.fixture);link=root.find("world/model/link[@name='%s']"%name)
            link.remove(link.find('pose'))
            with self.subTest(link=name),self.assertRaises(ValueError):self.extract(root)


if __name__=='__main__':unittest.main()
