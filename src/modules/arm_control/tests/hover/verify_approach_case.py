#!/usr/bin/env python3
"""Offline evidence cross-check. No ROS connection, motion or jaw commands."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def finite(value):
    if isinstance(value, dict): return all(finite(x) for x in value.values())
    if isinstance(value, list): return all(finite(x) for x in value)
    return not isinstance(value, (int, float)) or math.isfinite(value)


def verify(folder):
    result=json.loads((folder/'result.json').read_text())
    if not result.get('success') or not result.get('hover_approach',{}).get('approach_validated'):
        raise ValueError('Incomplete/failed approach case; cannot certify a positive result')
    manifest=json.loads((folder/'manifest.json').read_text())
    rows=[json.loads(x) for x in (folder/'approach_status.jsonl').read_text().splitlines()]
    flight=[json.loads(x) for x in (folder/'telemetry.jsonl').read_text().splitlines()]
    graph=json.loads((folder/'ros_graph.json').read_text())
    p=result['hover_approach']; ids=p['binding']; jaw=p['initial_joint_target'][5]
    assert result['success'] and result['flight_goal_validated'] and p['approach_validated']
    assert manifest['no_close_approach'] and not manifest['vehicle_fixed'] and not manifest['object_attached']
    assert not manifest['lidar'] and not manifest['flight_pid_modified'] and not manifest['ekf_truth_injected']
    assert result['arm_motion_requests_sent'] and p['approach_request_count']>=2
    assert not result['gripper_requests_sent'] and not result['grasp_tested']
    assert not p['close_authorized'] and not p['grasp_executed'] and not p['grasp_alignment_validated']
    assert result['own_processes_stopped'] and finite(result) and rows and flight
    assert all(finite(r) for r in rows+flight)
    assert all(a['sim_s']<b['sim_s'] for data in (rows,flight) for a,b in zip(data,data[1:]))
    events=p['events']; names=[e['event'] for e in events]
    assert set(names)>={'task_bound_after_terminal_hover','wrong_plan_rejected_no_motion',
        'no_close_approach_started','side_unfold_waypoint_reached','no_close_pregrasp_held'}
    wrong=next(e for e in events if e['event']=='wrong_plan_rejected_no_motion')
    started=next(e for e in events if e['event']=='no_close_approach_started')
    assert wrong['sim_s']<started['sim_s']
    assert wrong['status']['phase']==0 and wrong['status']['request_result']==3
    before=[r for r in rows if r['sim_s']<=wrong['sim_s']]
    assert before and all(r['arm']['target_sequence']==p['initial_target_sequence'] for r in before)
    assert all(np.max(np.abs(np.asarray(r['arm']['joint_target'])-p['initial_joint_target']))<=1e-6 for r in before)
    active=[r for r in rows if r['approach']['ownership_locked']]
    assert len(active)>30 and any(r['hover']['prepared'] for r in rows)
    lower=np.array([-1.57,-1.2,-1.5,-1.57,-1.57]); upper=-lower
    for r in rows:
        a=r['arm']; s=r['approach']; h=r['hover']; q=np.asarray(a['joint_target'])
        assert len(q)==6 and np.all(q[:5]>=lower-1e-6) and np.all(q[:5]<=upper+1e-6)
        assert abs(q[5]-jaw)<=1e-6 and abs(s['jaw_target']-jaw)<=1e-6
        assert not s['close_authorized'] and not r['external_contact'] and r['geometry_valid']
        assert r['fk_position_error']<=.002 and r['fk_rotation_error']<=.001
        assert r['object_position_error']<=.004
        if s['ownership_locked']:
            assert [s[k] for k in ('task_id','plan_id','target_id','scene_id')]==ids
            assert s['phase'] in (1,2,3) and s['fault']==0 and s['lease_valid']
            assert r['clearance']>.016 and a['feedback_valid'] and a['feedback_age']<=.200001
        if s['motion_authorized'] and abs(s['timestamp']-h['timestamp'])<=20000:
            assert h['start_ready'] and h['denial_mask']==0 and h['native_heading_ready']
            assert h['hover_position_error']<=.03001 and h['body_speed']<=.05001
            assert h['body_angular_speed']<=.10001 and h['tilt']<=math.radians(5)+1e-5
            assert h['motor_max']<.95
        if s['phase']==2 and abs(s['timestamp']-h['timestamp'])<=20000:
            assert h['relative_speed']<=.01001 and s['goal_error']<=.01501
    # Ignore the first two observer frames of each transition (separate RPCs
    # can straddle one native update), then require a truly frozen setpoint.
    paused=[]; stable_pauses=[]
    for r in rows:
        if r['approach']['phase']==3:
            paused.append(r)
        else:
            if len(paused)>=6: stable_pauses.append(paused[2:])
            paused=[]
    if len(paused)>=6: stable_pauses.append(paused[2:])
    for segment in stable_pauses:
        anchor=np.asarray(segment[0]['arm']['joint_target'])
        assert all(np.max(np.abs(np.asarray(r['arm']['joint_target'])-anchor))<=1e-6 for r in segment)
    final=rows[-1]; final_phase=final['approach']['phase']
    assert final_phase==2 and final['route_index']==len(p['waypoints_world'])-1
    assert final['actual_goal_error']<=.015 and p['final_error_m']<=.015
    hold=[]
    for r in reversed(rows):
        if r['route_index']!=final['route_index'] or r['approach']['phase']!=2 or r['actual_goal_error']>.015: break
        hold.append(r)
    duration=hold[0]['sim_s']-hold[-1]['sim_s']
    assert duration>=2.8
    assert p['final_target_sequence']>p['initial_target_sequence']+10
    assert np.max(np.abs(np.asarray(p['final_joint_target'])[:5]-p['initial_joint_target'][:5]))>.1
    assert p['approach_standoff_m']>.07
    unfolding=[r for r in flight if r['sim_s']>=started['sim_s']]
    assert unfolding and all(r['hover_error_m']<=.05 and r['tilt_rad']<=math.radians(5) for r in unfolding)
    assert all(not r['close_permitted'] for r in flight)
    publishers=[nodes for topic,nodes in graph['publishers'] if topic=='/mavros/setpoint_raw/local']
    assert publishers==[['/known_flight_manager_probe']] and result['sole_flight_publisher']==publishers[0][0]
    scene=ET.parse(folder/'scene.world').find('world')
    vehicle=scene.find("model[@name='yhang550']"); cube=scene.find("model[@name='test_cube']")
    assert vehicle is not None and vehicle.findtext('static','false')=='false'
    assert vehicle.find("joint[@name='bench_anchor']") is None
    assert cube is not None and cube.findtext('static','false')=='false'
    assert not any('velodyne' in str(e.attrib).lower() for e in vehicle.iter())
    return dict(verified=True,wrong_plan_rejected_without_motion=True,task_binding=ids,
        accepted_target_updates=p['final_target_sequence']-p['initial_target_sequence'],
        final_approach_error_m=final['actual_goal_error'],observed_final_hold_sim_s=duration,
        min_arm_obstacle_clearance_m=min(r['clearance'] for r in active),
        unfold_max_hover_error_m=max(r['hover_error_m'] for r in unfolding),
        unfold_max_tilt_deg=math.degrees(max(r['tilt_rad'] for r in unfolding)),
        frozen_pause_segments_checked=len(stable_pauses),
        jaw_target_unchanged=True,no_external_arm_contact=True,all_numeric_evidence_finite=True,
        grasp_alignment_validated=False,physical_grasp_tested=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('folder',type=Path)
    print(json.dumps(verify(parser.parse_args().folder),indent=2))
