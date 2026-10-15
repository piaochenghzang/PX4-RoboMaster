#!/usr/bin/env python3
"""Numerical route precheck only; no transport or actuation. Not dynamic/swept-scene proof."""
import json,sys,xml.etree.ElementTree as E
import hashlib
from types import SimpleNamespace
import yaml
from pathlib import Path
import numpy as np
sys.path.insert(0,'/home/pcz/super_ws/src/super_px4_bridge/scripts')
from grasp_audit.geometry import rpy_transform,pose_transform
ROOT=Path(__file__).resolve().parents[5]
from known_target import KnownTarget
from hover_approach_probe import ApproachPlan,validate_swept_bounds
cfg=json.loads(Path(__file__).with_name('known_target_approach.json').read_text())
target=KnownTarget(cfg)
plan=ApproachPlan(target,SimpleNamespace(source_instance='offline_only',world_epoch=1,vehicle_id=1,target_id=1))
fk_config=yaml.safe_load(Path('/home/pcz/super_ws/src/super_px4_bridge/config/grasp_frame_audit.yaml').read_text())
model=E.parse(ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/so101/so101.sdf').find('model')
links={l.get('name'):l for l in model.findall('link')}; joints=model.findall('joint')
names=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
origins=[rpy_transform(x) for x in fk_config['joint_origins']]
tool=rpy_transform(fk_config['tool_pose'])
def fk(q,jac=False):
    T=np.eye(4); pivots=[]; axes=[]
    for o,v in zip(origins,q):
        T=T@o; pivots.append(T[:3,3].copy()); axes.append(T[:3,2].copy())
        T=T@rpy_transform([0,0,0,0,0,v])
    p=(T@tool)[:3,3]
    return (p,np.array([np.cross(a,p-b) for a,b in zip(axes,pivots)]).T) if jac else p
lo=np.array([-1.57,-1.2,-1.5,-1.57,-1.57]); hi=np.array([1.57,1.2,1.5,1.57,1.57])
def ik(seed,goal):
    q=seed.copy(); prev=np.inf; stagnant=0
    for it in range(80):
        p,J=fk(q,True);e=goal-p;err=np.linalg.norm(e)
        if err<1e-4:return q
        stagnant=stagnant+1 if prev-err<1e-5 else 0
        if stagnant>=8:return None
        prev=err;dq=J.T@np.linalg.solve(J@J.T+.0001*np.eye(3),e)
        q=np.clip(q+np.clip(.3*dq,-.15,.15),lo,hi)
    return None
GA=target.hover@target.mount
GO=pose_transform([*cfg['object_position_world_m'],*cfg['object_quaternion_world_wxyz']])
S=GO.copy();S[:3,3]+=(1 if GO[2,2]<0 else -1)*GO[:3,2]*.012
def aabb(T,h):
    e=np.abs(T[:3,:3])@h;return T[:3,3]-e,T[:3,3]+e
obs=[aabb(GO,np.array([.01]*3)),aabb(S,np.array([.006,.007,.002]))]
def clearance(q):
    poses={'base_link':GA};pending=joints[:];vals=dict(zip(names,[*q,.28]))
    while pending:
        rest=[]
        for j in pending:
            parent=j.findtext('parent')
            if parent not in poses:rest.append(j);continue
            a=np.fromstring(j.findtext('axis/xyz','0 0 1'),sep=' ');a=a/np.linalg.norm(a);v=vals[j.get('name')]
            K=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
            R=np.eye(4);R[:3,:3]=np.eye(3)+np.sin(v)*K+(1-np.cos(v))*(K@K)
            poses[j.findtext('child')]=poses[parent]@rpy_transform(np.fromstring(j.findtext('pose','0 0 0 0 0 0'),sep=' '))@R
        assert len(rest)<len(pending);pending=rest
    value=100
    for name,l in links.items():
        for c in l.findall('collision'):
            T=poses[name]@rpy_transform(np.fromstring(c.findtext('pose','0 0 0 0 0 0'),sep=' '))
            h=np.fromstring(c.findtext('geometry/box/size'),sep=' ')*.5;a,b=aabb(T,h);value=min(value,a[2]-.05)
            for x,y in obs:value=min(value,np.linalg.norm(np.maximum(np.maximum(x-b,a-y),0)))
    return value
bounds=validate_swept_bounds(ROOT)
radii=np.array(bounds['native_upper_bounds_m'])
# Representative tracking bias, not a bound on dynamic tracking error.
bias=np.array([-.0018,.0017,-.00005,-.0013,-.00375])
def check(label,route):
    q=np.array([-.07,-1.175,1.45,1.3,-1.57]);minimum=100;status='ok';steps=0;leg=0
    for leg,goal in enumerate(map(np.array,route)):
        # Native context requires every requested waypoint to be INSIDE the
        # workspace; only the folded accepted start may recover from outside.
        if not (.05<=goal[0]<=.4 and abs(goal[1])<=.25 and .02<=goal[2]<=.4):
            status='goal_outside_workspace';break
        for n in range(700):
            measured=q+bias;margin=clearance(measured);minimum=min(minimum,margin);p=fk(q)
            delta=goal-p;distance=np.linalg.norm(delta)
            if distance<.001:break
            result=ik(q,p+delta*min(1,.003/distance))
            if result is None:status='IK';break
            d=result-q;desired=q+d*min(1,.02/max(abs(d)))
            reserve=radii@abs(q-measured);travel=radii@abs(desired-q);budget=margin-.016
            if budget<=reserve:status='clearance';break
            candidate=q+(desired-q)*min(1,(budget-reserve)/travel if travel>0 else 1);nxt=fk(candidate)
            if p[2]<.02 and nxt[2]<p[2]-1e-6:status='outward';break
            q=candidate;steps+=1
        else:status='iteration_limit'
        if status!='ok':break
    return dict(label=label,status=status,leg=leg,steps=steps,min_gap=minimum,ee=fk(q).tolist(),q=q.tolist())
if __name__=='__main__':
    nominal=GA.copy(); results=[]
    for offset in ([0,0,0],[.017,-.007,.001],[.03,0,0],[0,.03,0],[0,-.03,0],[0,0,.03],[0,0,-.03]):
        GA=nominal.copy();GA[:3,3]+=offset
        route=[(np.linalg.inv(GA)@np.r_[p,1])[:3] for p in plan.waypoints_world]
        results.append(check(str(offset),route))
    output=dict(numerical_precheck_only=True,not_dynamic_or_arbitrary_scene_proof=True,
        tracking_bias_rad=bias.tolist(),joint_radii_m=bounds,
        waypoints_world_m=[p.tolist() for p in plan.waypoints_world],
        sdf_sha256=hashlib.sha256((ROOT/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/so101/so101.sdf').read_bytes()).hexdigest(),
        nominal_feasible=results[0]['status']=='ok',
        all_sampled_offsets_feasible=all(r['status']=='ok' for r in results),samples=results)
    print(json.dumps(output,indent=2))
    # Offset probes are advisory counterexamples, not permission to execute.
    # Runtime still rejects any out-of-workspace/faulting actual-arm waypoint.
    raise SystemExit(0 if output['nominal_feasible'] else 1)
