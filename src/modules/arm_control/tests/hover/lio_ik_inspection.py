"""Read-only native IK and local no-contact geometry inspection, not execution."""
from pathlib import Path
from types import SimpleNamespace
import json
import numpy as np
from lio_tool_observer import finite,inverse,rpy_transform


def tcp_goal_from_observation(row,config,reference_gripper):
    if row['frame']!='map':raise ValueError('TARGET_FRAME')
    target=finite(row['map_from_target'],(4,4));arm=finite(row['map_from_arm'],(4,4))
    inverse(target);inverse(arm);reference=finite(reference_gripper,(3,))
    gripper=target.copy();gripper[:3,3]-=target[:3,:3]@reference
    tcp=gripper@rpy_transform(config['tool_pose'])
    return dict(map_from_desired_gripper=gripper,map_from_desired_tcp=tcp,
        arm_from_desired_tcp=inverse(arm)@tcp,
        reference_roundtrip_error_m=float(np.linalg.norm((gripper@np.r_[reference,1])[:3]-target[:3,3])))


def local_geometry_check(boxes,object_map,geometry):
    from perception.check_installation import transform,box_gap
    target=finite(object_map,(4,4));inverse(target)
    cfg=geometry['target'];obstacles=[]
    for part in cfg['parts']:
        half=finite(part['size_m'],(3,))*.5
        if np.any(half<=0):raise ValueError('Invalid target size')
        obstacles.append((part['name'],target@transform(part['xyz_rpy_deg']),half))
    half=finite(cfg['support_size_m'],(3,))*.5
    if np.any(half<=0):raise ValueError('Invalid support size')
    obstacles.append(('support_profile',target@transform(cfg['object_from_support_xyz_rpy_deg']),half))
    pairs=[]
    for name,T,h in boxes:
        T=finite(T,(4,4));inverse(T);h=finite(h,(3,))
        if np.any(h<=0):raise ValueError('Invalid arm box')
        for obstacle,O,half in obstacles:
            pairs.append(dict(arm_box=name,obstacle=obstacle,lower_bound_m=float(box_gap(T,h,O,half))))
    if not pairs:raise ValueError('Empty local geometry')
    minimum=min(pairs,key=lambda p:p['lower_bound_m'])
    return dict(local_clearance_valid=minimum['lower_bound_m']>=.016,min_lower_bound_m=minimum['lower_bound_m'],
        minimum_pair=minimum,required_m=.016,contact_exemptions=[],
        support_source='ASSUMED_STATIC_FIXTURE_RELATIVE_TO_OBSERVED_TARGET',
        scope=['target_shape','support_profile'],
        unchecked=['ground','vehicle_body','other_obstacles','entire_trajectory','estimation_uncertainty'],
        motion_permitted=False,closing_permitted=False)


def inspect_native(client,row,config,geometry,folder):
    from check_fixed_target_native import snapshot,assert_unchanged,evidence_json
    from fixed_grasp_target import load_target,parse_native
    from fixed_demo_plan import FixedDemoPlan
    from hover_approach_probe import validate_swept_bounds
    from perception.check_installation import transform
    folder=Path(folder);folder.mkdir()
    def save(name,value):
        (folder/(name+'.json')).write_text(json.dumps(evidence_json(value),indent=2,allow_nan=False)+'\n')
    original=finite(row['map_from_target'],(4,4)).copy()
    goal=tcp_goal_from_observation(row,config,config['reference_gripper'])
    save('tcp_input',dict(input_stamp_s=row['stamp_s'],input_policy='FROZEN_READ_ONLY_SNAPSHOT_NOT_MOTION_TARGET',
        orientation_policy='DEMO_OBJECT_ROTATION_AS_DESIRED_GRIPPER_ROTATION',position_only_native_ik=True,
        **{k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in goal.items()}))
    before=snapshot(client);assert_unchanged(before,before);save('before',before)
    # Independent, existing native-solver regression control. This is not an
    # assertion that the observed object is currently reachable.
    fixture=load_target(Path(__file__).with_name('known_target_approach.json'))
    positive=np.array(fixture.preview()['tool_target_planned_arm'])[:3,3]
    observed=goal['arm_from_desired_tcp'][:3,3]
    cases=[('observed_target',observed),('existing_solver_positive',positive),
        ('outside_workspace',[.5,0,.05]),('inside_workspace_unreachable',[.39,.24,.39]),
        ('repeat_positive',positive)]
    checks={};invariants={}
    for label,point in cases:
        output=client.client('arm_control','ik_check',*map(float,point))
        (folder/(label+'.log')).write_text(output)
        state=snapshot(client);save(label+'_state',state)
        invariants[label]=assert_unchanged(before,state)
        checks[label]=dict(input_tcp_position_arm_m=list(map(float,point)),native=parse_native(output))
        save('native_checks',checks);save('invariants',invariants)
    if checks['observed_target']['native']['workspace_valid']:raise RuntimeError('Distant observed target should be rejected')
    if not checks['existing_solver_positive']['native']['ik_success']:raise RuntimeError('Existing native IK positive failed')
    if checks['outside_workspace']['native']['workspace_valid']:raise RuntimeError('Workspace rejection failed')
    if not checks['inside_workspace_unreachable']['native']['workspace_valid'] or checks['inside_workspace_unreachable']['native']['ik_success']:
        raise RuntimeError('Native unreachable rejection failed')
    if checks['repeat_positive']['native']!=checks['existing_solver_positive']['native']:raise RuntimeError('Read-only IK not deterministic')
    boxes=list(FixedDemoPlan.boxes(SimpleNamespace(arm=np.array(row['map_from_arm'])),
        np.array(row['joint_position_rad'][:5]),row['joint_position_rad'][5]))
    current=local_geometry_check(boxes,original,geometry)
    if not current['local_clearance_valid']:raise RuntimeError('Far folded geometry unexpectedly lacks local clearance')
    body=next(p for p in geometry['target']['parts'] if p['name']=='body')
    synthetic=original.copy()
    synthetic[:3,3]=boxes[0][1][:3,3]-original[:3,:3]@transform(body['xyz_rpy_deg'])[:3,3]
    collision=local_geometry_check(boxes,synthetic,geometry)
    if collision['local_clearance_valid']:raise RuntimeError('Synthetic insufficient-clearance case accepted')
    if not np.array_equal(original,np.array(row['map_from_target'])):raise RuntimeError('Observed target mutated')
    after=snapshot(client);invariants['final']=assert_unchanged(before,after);save('after',after)
    save('invariants',invariants)
    root=Path(__file__).resolve().parents[5]
    result=dict(inspection_verified=True,observed_target_rejected=True,
        existing_solver_positive_source='KNOWN_TARGET_APPROACH_FIXTURE_NOT_THE_OBSERVED_TARGET',
        current_local_geometry=current,synthetic_intrusion=collision,
        swept_bounds=validate_swept_bounds(root),target_unchanged=True,control_targets_unchanged=True,
        motion_permitted=False,closing_permitted=False,complete_approach_safety_validated=False)
    save('result',result)
    return result
