"""Candidate hover geometry and read-only IK inspection; no flight/arm targets."""
from pathlib import Path
from types import SimpleNamespace
import math
import json
import numpy as np
from lio_tool_observer import finite,inverse,rpy_transform,fk_gripper
from lio_ik_inspection import tcp_goal_from_observation,local_geometry_check


def angle_deg(a,b):
    return math.degrees(math.acos(float(np.clip((np.trace(a.T@b)-1)/2,-1,1))))


def derive_hover(object_map,config,q_reference,pre_distance=.04):
    target=finite(object_map,(4,4)).copy();inverse(target)
    q=finite(q_reference,(6,));reference=finite(config['reference_gripper'],(3,))
    if not math.isfinite(pre_distance) or not 0<pre_distance<=.1:raise ValueError('Invalid preapproach distance')
    mount=rpy_transform(config['mount_pose']);body_gripper=mount@fk_gripper(config,q)
    inferred_body_rotation=target[:3,:3]@body_gripper[:3,:3].T
    yaw=math.atan2(inferred_body_rotation[1,0],inferred_body_rotation[0,0])
    hover=rpy_transform([0,0,0,0,0,yaw])
    hover[:3,3]=target[:3,3]-hover[:3,:3]@(body_gripper@np.r_[reference,1])[:3]
    arm=hover@mount
    row=dict(frame='map',map_from_target=target,map_from_arm=arm)
    final=tcp_goal_from_observation(row,config,reference)
    pre_target=target.copy();pre_target[:3,3]+=target[:3,2]*pre_distance
    pre=tcp_goal_from_observation(dict(row,map_from_target=pre_target),config,reference)
    return dict(map_from_body_hover=hover,map_from_arm=arm,map_from_target=target,
        final_goal=final,pre_goal=pre,yaw_rad=yaw,pre_distance_m=pre_distance,
        template_orientation_error_deg=angle_deg((hover@body_gripper)[:3,:3],target[:3,:3]))


def inspect_hover(client,row,config,geometry,q_reference,folder):
    if row.get('frame')!='map':raise ValueError('Candidate inspection requires map frame')
    from check_fixed_target_native import snapshot,assert_unchanged,evidence_json
    from fixed_grasp_target import parse_native
    from fixed_demo_plan import FixedDemoPlan
    from check_approach_route import ik
    from grasp_audit.geometry import pose_transform
    folder=Path(folder);folder.mkdir()
    def serial(value):
        if isinstance(value,np.ndarray):return value.tolist()
        if isinstance(value,dict):return {k:serial(v) for k,v in value.items()}
        return value
    def save(name,value):
        (folder/(name+'.json')).write_text(json.dumps(evidence_json(serial(value)),indent=2,allow_nan=False)+'\n')
    original=np.array(row['map_from_target']).copy()
    q_reference=finite(q_reference,(6,));candidate=derive_hover(original,config,q_reference)
    save('candidate',dict(candidate,input_stamp_s=row['stamp_s'],frame='map',
        source='OBSERVED_TARGET_PLUS_EXISTING_DEMO_GRASP_POSTURE',hypothetical_body_pose_not_a_flight_command=True))
    arm=candidate['map_from_arm']
    def clearance(q):
        boxes=FixedDemoPlan.boxes(SimpleNamespace(arm=arm),np.array(q[:5]),float(q[5]))
        return local_geometry_check(list(boxes),original,geometry)
    final_geometry=clearance(q_reference)
    # Existing numerical approximation is an advisory branch probe; it is
    # recorded separately from the live native solver's current-seed result.
    pre_q=ik(q_reference[:5],candidate['pre_goal']['arm_from_desired_tcp'][:3,3])
    planned_pre=None
    if pre_q is not None:
        pre_q=np.r_[pre_q,q_reference[5]]
        actual=fk_gripper(config,pre_q)@rpy_transform(config['tool_pose'])
        desired=candidate['pre_goal']['arm_from_desired_tcp']
        planned_pre=dict(source='EXISTING_NUMERICAL_IK_APPROXIMATION_NOT_NATIVE_PREVIEW',q=pre_q,
            position_error_m=float(np.linalg.norm(actual[:3,3]-desired[:3,3])),
            orientation_error_deg=angle_deg(actual[:3,:3],desired[:3,:3]),geometry=clearance(pre_q))
    save('template_geometry',dict(final_no_contact=final_geometry,planned_preapproach=planned_pre))
    before=snapshot(client);assert_unchanged(before,before);save('before',before)
    checks={};invariants={}
    for label in ('pre_goal','final_goal','repeat_pre'):
        goal=candidate['pre_goal' if label=='repeat_pre' else label]['arm_from_desired_tcp']
        output=client.client('arm_control','ik_check',*map(float,goal[:3,3]))
        (folder/(label+'.log')).write_text(output)
        after=snapshot(client);invariants[label]=assert_unchanged(before,after)
        native=parse_native(output)
        item=dict(input_tcp_arm=goal,native=native)
        if native.get('joint_valid'):
            actual=pose_transform([*native['fk'],*native['quat']])
            item['orientation_error_deg']=angle_deg(actual[:3,:3],goal[:3,:3])
            item['geometry']=clearance(np.r_[native['q'],q_reference[5]])
        checks[label]=item;save('native_checks',checks);save('invariants',invariants)
    if checks['pre_goal']['native']!=checks['repeat_pre']['native']:raise RuntimeError('Candidate IK query changed its result')
    after=snapshot(client);invariants['final']=assert_unchanged(before,after);save('after',after);save('invariants',invariants)
    if not np.array_equal(original,np.array(row['map_from_target'])):raise RuntimeError('Locked target changed')
    native_ready=all(checks[k]['native'].get('ik_success') and checks[k].get('orientation_error_deg',math.inf)<=5
        for k in ('pre_goal','final_goal')) and checks['pre_goal'].get('geometry',{}).get('local_clearance_valid',False)
    nominal=bool(candidate['template_orientation_error_deg']<=5 and planned_pre is not None and
        planned_pre['position_error_m']<1e-4 and planned_pre['orientation_error_deg']<=5 and
        planned_pre['geometry']['local_clearance_valid'] and all(checks[k]['native']['workspace_valid'] for k in ('pre_goal','final_goal')))
    result=dict(inspection_verified=True,nominal_preapproach_geometry_valid=nominal,
        template_orientation_error_deg=candidate['template_orientation_error_deg'],orientation_limit_deg=5.,
        native_current_seed_pose_acceptable=bool(native_ready),
        native_orientation_error_deg={k:checks[k].get('orientation_error_deg') for k in ('pre_goal','final_goal')},
        final_all_16mm_geometry_valid=final_geometry['local_clearance_valid'],
        planned_preapproach_local_clearance_m=None if planned_pre is None else planned_pre['geometry']['min_lower_bound_m'],
        target_unchanged=True,control_targets_unchanged=True,flight_commands_sent=False,
        motion_permitted=False,closing_permitted=False,complete_approach_safety_validated=False)
    save('result',result)
    return result
