"""Read-only native IK along the existing unfold route, ending before contact."""
from pathlib import Path
from types import SimpleNamespace
import json
import math
import numpy as np
from lio_tool_observer import finite, fk_gripper, rpy_transform


def workspace_transition(previous, following):
    """Existing 20 mm step / monotonic recovery rules; no workspace bypass."""
    a=finite(previous,(3,));b=finite(following,(3,))
    low=np.array([.05,-.25,.02]);high=np.array([.4,.25,.4])
    if np.linalg.norm(b-a)>.02+1e-6:return False
    outside=(a<low)|(a>high)
    if not outside.any():return bool(np.all(b>=low) and np.all(b<=high))
    progress=False
    for k in range(3):
        if a[k]<low[k]:
            if b[k]<a[k] or b[k]>high[k]:return False
            progress=progress or b[k]>a[k]+1e-6
        elif a[k]>high[k]:
            if b[k]>a[k] or b[k]<low[k]:return False
            progress=progress or b[k]<a[k]-1e-6
        elif b[k]<low[k] or b[k]>high[k]:return False
    return bool(progress)


def certify_segment(q0,q1,radii,clearance_fn,transition_ok=lambda a,b:True,
                    minimum=.016,max_depth=8):
    """Bound every point of a nominal linear joint segment, not just vertices.

    Each endpoint clearance is a lower bound on real geometry separation.
    Descendant-corner travel is bounded by sum(radius * abs(delta angle)).
    Subtraction therefore remains conservative even if the geometric lower
    bound itself is not smooth. Subdivide only when this bound is too loose.
    This does not bound tracking error or motion of the aircraft/obstacles.
    """
    a=np.asarray(q0,dtype=float).copy();b=np.asarray(q1,dtype=float).copy()
    r=np.asarray(radii,dtype=float)
    if (a.ndim!=1 or not a.size or b.shape!=a.shape or r.shape!=a.shape or
            not np.isfinite([a,b,r]).all() or np.any(r<0) or
            not math.isfinite(minimum) or minimum<=0 or
            type(max_depth) is not int or not 0<=max_depth<=16):
        raise ValueError('Invalid swept segment')
    samples=0;sample_min=math.inf;leaf_bounds=[]
    def distance(q):
        nonlocal samples,sample_min
        value=float(clearance_fn(q.copy()));samples+=1
        if not math.isfinite(value):raise ValueError('Nonfinite clearance')
        sample_min=min(sample_min,value);return value
    def check(x,y,lx,ly,depth):
        bound=min(lx,ly)-float(r@np.abs(y-x))
        if min(lx,ly)<minimum:return False
        if bound>=minimum:leaf_bounds.append(bound);return True
        if depth>=max_depth:return False
        middle=(x+y)*.5;lm=distance(middle)
        return check(x,middle,lx,lm,depth+1) and check(middle,y,lm,ly,depth+1)
    valid_transition=bool(transition_ok(a.copy(),b.copy()))
    accepted=check(a,b,distance(a),distance(b),0) if valid_transition else False
    return dict(certified=bool(accepted),transition_valid=valid_transition,samples=samples,
        minimum_sampled_m=None if not samples else sample_min,
        swept_lower_bound_m=min(leaf_bounds) if accepted else None,required_m=minimum)


def inspect_unfold(client,row,config,geometry,q_reference,folder):
    """Simulate diagnostic seeds only. No target / ownership / jaw writes."""
    from candidate_hover import derive_hover,angle_deg
    from check_fixed_target_native import snapshot,assert_unchanged,evidence_json
    from fixed_grasp_target import parse_native
    from fixed_demo_plan import FixedDemoPlan,START,UNFOLDED,ROUTE
    from hover_approach_probe import validate_swept_bounds
    from lio_ik_inspection import local_geometry_check
    from grasp_audit.geometry import pose_transform
    if row.get('frame')!='map':raise ValueError('Unfold inspection requires map frame')
    folder=Path(folder);folder.mkdir()
    def serial(value):
        if isinstance(value,np.ndarray):return value.tolist()
        if isinstance(value,dict):return {k:serial(v) for k,v in value.items()}
        if isinstance(value,(list,tuple)):return [serial(v) for v in value]
        return value
    def save(name,value):
        (folder/(name+'.json')).write_text(json.dumps(evidence_json(serial(value)),indent=2,allow_nan=False)+'\n')
    candidate=derive_hover(row['map_from_target'],config,q_reference)
    original=finite(row['map_from_target'],(4,4)).copy()
    before=snapshot(client);assert_unchanged(before,before);save('before',before)
    q=finite(before['arm_control_status']['joint_target'],(6,))
    if np.max(np.abs(q[:5]-START))>1e-5:raise RuntimeError('Expected existing folded control target')
    jaw=float(q[5]);q=q[:5].copy()
    bounds=validate_swept_bounds(Path(__file__).resolve().parents[5])
    radii=np.array(bounds['native_upper_bounds_m']);arm=candidate['map_from_arm']
    def tcp(joints):return fk_gripper(config,np.r_[joints,jaw])@rpy_transform(config['tool_pose'])
    def geometry_at(joints):
        return local_geometry_check(list(FixedDemoPlan.boxes(SimpleNamespace(arm=arm),joints,jaw)),original,geometry)
    def gap(joints):return geometry_at(joints)['min_lower_bound_m']
    certificates=[];route=[];queries=[];query_log=folder/'native_queries.log'
    def unchanged():return assert_unchanged(before,snapshot(client))
    def append(label,next_q):
        nonlocal q
        next_q=finite(next_q,(5,));old=tcp(q);new=tcp(next_q)
        if np.max(np.abs(next_q-q))>.020001:raise RuntimeError('Joint increment exceeds existing bound')
        certificate=certify_segment(q,next_q,radii,gap,
            transition_ok=lambda a,b:workspace_transition(tcp(a)[:3,3],tcp(b)[:3,3]))
        certificates.append(dict(stage=label,**certificate))
        if not certificate['certified']:raise RuntimeError('Unfold route segment not certified: '+label)
        q=next_q.copy();route.append(dict(stage=label,q=q.copy(),tcp_arm=new,
            cartesian_step_m=float(np.linalg.norm(new[:3,3]-old[:3,3])),geometry=geometry_at(q)))
    def query(point,seed,label):
        args=[*map(float,point),*map(float,seed)]
        output=client.client('arm_control','ik_check',*args)
        with query_log.open('a') as log:log.write(label+' '+json.dumps(args)+'\n'+output+'\n')
        state=unchanged();native=parse_native(output)
        queries.append(dict(stage=label,input_tcp_arm_m=point,input_seed=seed.copy(),native=native,invariants=state))
        if not native.get('ik_success') or not native.get('joint_valid'):raise RuntimeError('Seeded native IK refused: '+label)
        # Compare the solver's own reported FK with the configured SDF/FK chain.
        calculated=tcp(native['q']);reported=pose_transform([*native['fk'],*native['quat']])
        if (np.linalg.norm(calculated[:3,3]-reported[:3,3])>2e-6 or
                angle_deg(calculated[:3,:3],reported[:3,:3])>.01):
            raise RuntimeError('Native / configured FK mismatch')
        return native
    try:
        route.append(dict(stage='folded_start',q=q.copy(),tcp_arm=tcp(q),geometry=geometry_at(q)))
        # Exactly the existing first stage: wrist only, +1.3 -> -1.3.
        for wrist in np.linspace(START[3],UNFOLDED[3],131)[1:]:
            next_q=START.copy();next_q[3]=wrist;append('wrist_unfold',next_q)
        # Retain only the first existing transit waypoint. Later old waypoints
        # enter the contact region, outside this no-contact inspection.
        pre=candidate['pre_goal']['arm_from_desired_tcp']
        for label,goal in [('existing_transit',np.array(ROUTE[0])),('observed_preapproach',pre[:3,3])]:
            for _ in range(300):
                current=tcp(q)[:3,3];delta=goal-current;length=float(np.linalg.norm(delta))
                if length<.0001:break
                point=current+delta*min(1.,.001/length)
                solved=query(point,q,label)
                change=np.array(solved['q'])-q
                change*=min(1.,.02/max(float(np.max(np.abs(change))),1e-12))
                append(label,q+change)
            else:raise RuntimeError('Read-only native route did not converge')
        endpoint_seed=q.copy()
        final=query(pre[:3,3],endpoint_seed,'preapproach_endpoint')
        repeat=query(pre[:3,3],endpoint_seed,'repeat_preapproach_endpoint')
        if final!=repeat:raise RuntimeError('Explicit seed query is not deterministic')
        append('preapproach_endpoint',np.array(final['q']))
        error=float(np.linalg.norm(tcp(q)[:3,3]-pre[:3,3]))
        orientation=angle_deg(tcp(q)[:3,:3],pre[:3,:3])
        if error>=.0001 or orientation>5.:raise RuntimeError('Unfold branch failed preapproach pose check')
        rejections={}
        values=[*map(float,pre[:3,3]),*map(float,q)]
        for label,args in [('seed_out_of_range',values[:3]+[2.,*values[4:]]),
                           ('invalid_seed_before_workspace_refusal',[.5,0.,.05,2.,*values[4:]]),
                           ('seed_nan',values[:3]+['nan',*values[4:]]),
                           ('seed_wrong_arity',values[:-1])]:
            try:client.client('arm_control','ik_check',*args)
            except RuntimeError as exc:rejections[label]=dict(refused=True,error=str(exc),invariants=unchanged())
            else:raise RuntimeError('Invalid explicit seed accepted: '+label)
        if query(pre[:3,3],endpoint_seed,'positive_after_refusals')!=final:
            raise RuntimeError('Post-refusal positive changed the solution')
        save('rejections',rejections)
        invariants=unchanged();save('after',snapshot(client))
        if not np.array_equal(original,np.array(row['map_from_target'])):raise RuntimeError('Locked target changed')
        result=dict(inspection_verified=True,native_unfold_branch_valid=True,
            route_segments=len(certificates),native_queries=len(queries),
            final_position_error_m=error,final_orientation_error_deg=orientation,orientation_limit_deg=5.,
            min_sampled_local_clearance_m=min(c['minimum_sampled_m'] for c in certificates),
            min_swept_local_lower_bound_m=min(c['swept_lower_bound_m'] for c in certificates),
            maximum_cartesian_step_m=max(p.get('cartesian_step_m',0) for p in route),
            final_q=q.tolist(),fixed_jaw_rad=jaw,swept_bounds=bounds,invariants=invariants,
            input_seed_source='EXPLICIT_READ_ONLY_SEED_NOT_PUBLISHED_TARGET',
            waypoint_order=['START','UNFOLDED','ROUTE[0]','OBSERVED_40MM_PREAPPROACH'],
            geometry_scope=['observed_target_shape','assumed_static_support_profile'],
            unchecked=['ground','vehicle_and_self_collision','other_obstacles','estimation_uncertainty',
                'dynamic_tracking_error','aircraft_motion','continuous_workspace_between_command_vertices'],
            target_unchanged=True,control_targets_unchanged=True,arm_commands_sent=False,
            closing_permitted=False,motion_permitted=False,complete_approach_safety_validated=False)
        save('result',result);return result
    finally:
        save('candidate',candidate);save('route',route);save('certificates',certificates);save('queries',queries)
