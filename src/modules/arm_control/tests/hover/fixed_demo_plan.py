#!/usr/bin/env python3
"""One fixed known-object demo template, not a general grasp planner."""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from known_target import KnownTarget, mount_transform, inverse
from fixed_grasp_target import load_target
# Existing offline native-equivalent position-IK approximation, not actuation.
from check_approach_route import fk, ik, origins, links, joints, names, aabb

PAD_NAMES={'gripper_fixed_finger_collision','moving_jaw_collision'}
SUPPORT_HALF=np.array([.002,.012,.002])  # 4x24x4 mm, experiment copy only
SUPPORT_OFFSET_X=-.0015  # Covers both open-centered and closed cube gravity projections.
OPEN_REFERENCE=np.array([.005,-.000218,-.102])  # 3mm side gap; grip upper cube region above support.
ROUTE=[[.2357102098,.0142010304,.3282294517],[.26,.015,.36]]
START=np.array([-.07,-1.175,1.45,1.3,-1.57])
UNFOLDED=np.array([-.07,-1.175,1.45,-1.3,-1.57])
ARM_HOLD={'shoulder_lift':dict(i=1.,i_max=.18,torque_limit=.8),
          'wrist_flex':dict(i=.8,i_max=.14,torque_limit=.22)}


def configure_arm_hold(model):
    """Bounded gravity-bias removal in the experiment copy, no raised torque."""
    for name,cfg in ARM_HOLD.items():
        controller=model.find("plugin[@name='%s_controller']"%name)
        if controller is None or abs(float(controller.findtext('cmdMax'))-cfg['torque_limit'])>1e-8:
            raise ValueError('Unexpected production torque cap for '+name)
        controller.find('i').text=str(cfg['i'])
        element=controller.find('iMax')
        if element is None:element=ET.SubElement(controller,'iMax')
        element.text=str(cfg['i_max'])


def quaternion(R):
    t=float(np.trace(R))
    q=np.array([math.sqrt(max(0,1+t)),
        math.copysign(math.sqrt(max(0,1+2*R[0,0]-t)),R[2,1]-R[1,2]),
        math.copysign(math.sqrt(max(0,1+2*R[1,1]-t)),R[0,2]-R[2,0]),
        math.copysign(math.sqrt(max(0,1+2*R[2,2]-t)),R[1,0]-R[0,1])])*.5
    return q/np.linalg.norm(q)


class FixedDemoPlan:
    def __init__(self,target_file):
        source=load_target(target_file);config=json.loads(Path(target_file).read_text())
        if config.get('demo_calibrate_orientation') is not True:
            raise ValueError('This first fixed demo requires explicit fixture orientation calibration opt-in')
        if abs(source.known.yaw)>1e-6:raise ValueError('First fixed template requires zero world hover yaw')
        q=START.copy();self.samples=[q.copy()]
        # A fixed, bounded wrist unfold selects the existing downward-facing
        # IK branch. No new general orientation solver or joint limits.
        for wrist in np.linspace(1.3,-1.3,131)[1:]:
            q=START.copy();q[3]=wrist;self.samples.append(q.copy())
        for goal in map(np.asarray,ROUTE):
            for _ in range(1500):
                p=fk(q);delta=goal-p;length=np.linalg.norm(delta)
                if length<.0002:break
                step_limit=.001 if length<.06 else .003
                solved=ik(q,p+delta*min(1,step_limit/length))
                if solved is None:raise ValueError('Fixed demo offline IK rejected')
                step=solved-q;q+=step*min(1,.02/max(abs(step)));self.samples.append(q.copy())
            else:raise ValueError('Fixed demo numerical route did not finish')
        self.expected_q=q.copy()
        grip=np.eye(4)
        for origin,value in zip(origins,q):grip=grip@origin@mount_transform([0,0,0,0,0,value])
        # Calibrate this FIRST known-object demo to the reachable position-IK
        # branch. Never rotate/teleport an object after the simulation starts.
        arm=source.known.mount.copy()
        center=(grip@np.r_[OPEN_REFERENCE,1])[:3]
        arm[:3,3]=source.known.object[:3,3]-arm[:3,:3]@center
        obj=arm@grip;obj[:3,3]=source.known.object[:3,3]
        config['object_quaternion_world_wxyz']=quaternion(obj[:3,:3]).tolist()
        config['nominal_object_center_arm_m']=center.tolist()
        self.config=config;self.target=KnownTarget(config)
        self.arm=self.target.hover@self.target.mount
        self.support=obj.copy()
        self.support[:3,3]+=(1 if obj[2,2]<0 else -1)*obj[:3,2]*.012
        self.support[:3,3]+=obj[:3,0]*SUPPORT_OFFSET_X
        self.waypoints_world=[(self.arm@np.r_[p,1])[:3] for p in ROUTE]
        self.grasp_world=self.waypoints_world[-1]
        self.lift_world=self.grasp_world+np.array([0,0,.03])
        # Reject the ENTIRE demo before launch if the unchanged position IK
        # cannot also lift. A reachable grasp point alone is not sufficient.
        goal=(inverse(self.arm)@np.r_[self.lift_world,1])[:3]
        self.lift_samples=[q.copy()]
        for _ in range(600):
            p=fk(q);delta=goal-p;length=np.linalg.norm(delta)
            if length<.0002:break
            solved=ik(q,p+delta*min(1,.003/length))
            if solved is None:raise ValueError('Fixed demo lift IK rejected before launch')
            step=solved-q;q+=step*min(1,.02/max(abs(step)));self.lift_samples.append(q.copy())
        else:raise ValueError('Fixed demo lift IK did not finish')

    def boxes(self,q,jaw):
        poses={'base_link':self.arm};pending=list(joints);values=dict(zip(names,[*q,jaw]))
        while pending:
            rest=[]
            for joint in pending:
                parent=joint.findtext('parent')
                if parent not in poses:rest.append(joint);continue
                axis=np.fromstring(joint.findtext('axis/xyz'),sep=' ');axis/=np.linalg.norm(axis)
                v=values[joint.get('name')]
                K=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
                R=np.eye(4);R[:3,:3]=np.eye(3)+math.sin(v)*K+(1-math.cos(v))*(K@K)
                poses[joint.findtext('child')]=poses[parent]@mount_transform(
                    np.fromstring(joint.findtext('pose','0 0 0 0 0 0'),sep=' '))@R
            if len(rest)==len(pending):raise ValueError('Unsupported production joint tree')
            pending=rest
        for name,link in links.items():
            for col in link.findall('collision'):
                T=poses[name]@mount_transform(np.fromstring(col.findtext('pose','0 0 0 0 0 0'),sep=' '))
                h=np.fromstring(col.findtext('geometry/box/size',''),sep=' ')*.5
                if h.shape!=(3,) or np.any(h<=0):raise ValueError('Unsupported production collision proxy')
                yield col.get('name'),T,h

    def geometry(self,q,jaw,unfold=False):
        rows=[];obj=self.target.object;frames=[np.eye(4)] if unfold else [np.eye(4),inverse(obj)]
        for name,T,h in self.boxes(q,jaw):
            lo,_=aabb(T,h)
            rows.append(dict(collision=name,obstacle='ground/asphalt',distance=float(lo[2]-.05),required=.016))
            for obstacle,O,half in [('target',obj,np.array([.01]*3)),('support',self.support,SUPPORT_HALF)]:
                if not unfold and name in PAD_NAMES and obstacle=='target':continue
                lower=0.
                # Both coordinate projections are conservative lower bounds.
                # Their MAX tightens rotated boxes without reducing real clearance.
                for frame in frames:
                    a,b=aabb(frame@T,h);x,y=aabb(frame@O,half)
                    lower=max(lower,float(np.linalg.norm(np.maximum(np.maximum(x-b,a-y),0))))
                rows.append(dict(collision=name,obstacle=obstacle,distance=lower,
                    required=.003 if not unfold and name in PAD_NAMES and obstacle=='support' else .016))
        return sorted(rows,key=lambda row:row['distance']-row['required'])

    def preview(self):
        route=min((self.geometry(q,.3,unfold=n<=130)[0] for n,q in enumerate(self.samples)),key=lambda r:r['distance']-r['required'])
        unfold=min((self.geometry(q,.3,unfold=True)[0] for q in self.samples[:131]),key=lambda r:r['distance'])
        sweep=min((self.geometry(self.expected_q,jaw)[0] for jaw in np.linspace(.04,.3,53)),
            key=lambda r:r['distance']-r['required'])
        margin=min(route['distance']-route['required'],sweep['distance']-sweep['required'])
        pad_gap=100.
        for q in self.samples:
            for name,T,h in self.boxes(q,.3):
                if name not in PAD_NAMES:continue
                gap=0.
                for F in (np.eye(4),inverse(self.target.object)):
                    a,b=aabb(F@T,h);lo,hi=aabb(F@self.target.object,np.array([.01]*3))
                    gap=max(gap,float(np.linalg.norm(np.maximum(np.maximum(lo-b,a-hi),0))))
                pad_gap=min(pad_gap,gap)
        return dict(numerical_precheck_only=True,motion_permitted=False,grasp_tested=False,
            contact_exception='exact two finger pads versus test_cube ONLY',
            pad_support_min_m=.003,other_pairs_min_m=.016,support_size_m=(SUPPORT_HALF*2).tolist(),
            support_offset_local_x_m=SUPPORT_OFFSET_X,open_object_reference_gripper_m=OPEN_REFERENCE.tolist(),
            nominal_fixed_pad_axial_overlap_m=max(0.,min(-.075,OPEN_REFERENCE[2]+.01)-max(-.101,OPEN_REFERENCE[2]-.01)),
            jaw_stroke_rad=[.04,.3],gripper_p=.25,gripper_torque_limit_nm=.01,
            experiment_arm_hold=ARM_HOLD,
            known_target_config=self.config,expected_q=self.expected_q.tolist(),
            route_world=[p.tolist() for p in self.waypoints_world],lift_world=self.lift_world.tolist(),
            samples=len(self.samples),route_worst_pair=route,full_jaw_sweep_worst_pair=sweep,
            unfold_all_pairs_min_m=.016,unfold_worst_pair=unfold,
            lift_ik_samples=len(self.lift_samples),lift_ik_final_q=self.lift_samples[-1].tolist(),
            lift_ik_error_m=float(np.linalg.norm(fk(self.lift_samples[-1])-
                (inverse(self.arm)@np.r_[self.lift_world,1])[:3])),
            open_route_min_pad_target_gap_m=pad_gap,
            sampled_geometry_feasible=bool(margin>0 and pad_gap>.001),minimum_sampled_slack_m=margin,
            continuous_or_flight_proof=False)


if __name__=='__main__':
    result=FixedDemoPlan(Path(__file__).with_name('known_target_fixed.json')).preview()
    print(json.dumps(result,indent=2,allow_nan=False))
    raise SystemExit(0 if result['sampled_geometry_feasible'] else 2)
