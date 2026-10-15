#!/usr/bin/env python3
"""Offline cross-check of no-close permission evidence, never a control client."""
import argparse
import json
import math
from pathlib import Path


def verify(folder):
    result = json.loads((folder/'result.json').read_text())
    manifest = json.loads((folder/'manifest.json').read_text())
    samples = [json.loads(s) for s in (folder/'hover_status.jsonl').read_text().splitlines()]
    flight = [json.loads(s) for s in (folder/'telemetry.jsonl').read_text().splitlines()]
    probe = result['hover_prepare']; events = probe['events']
    assert result['success'] and result['flight_goal_validated'] and probe['hover_prepare_validated']
    assert manifest['hover_prepare_only'] and not manifest['vehicle_fixed'] and not manifest['object_attached']
    assert not result['grasp_tested'] and not result['hover_grasp_permission_validated']
    assert not result['arm_motion_requests_sent'] and not result['gripper_requests_sent']
    assert result['sole_flight_publisher'] == '/known_flight_manager_probe'
    assert samples and flight and len(samples)>100
    for rows, key in [(samples, 'sim_s'), (flight, 'sim_s')]:
        assert all(a[key] < b[key] for a, b in zip(rows, rows[1:]))
    for row in samples:
        s = row['status']
        assert not s['motion_authorized'] and not s['close_authorized']
        if s['prepared']:
            assert s['start_ready'] and s['denial_mask'] == 0 and s['native_heading_ready']
            assert s['request_result'] == 1
            assert s['hover_position_error'] <= .03001 and s['body_speed'] <= .05001
            assert s['body_angular_speed'] <= .10001 and s['tilt'] <= .087277
            assert s['motor_max'] < .95
            assert -.020001 <= (s['timestamp']-s['source_sample_time_us'])*1e-6 <= .100001
    accepted = [e for e in events if e['event'] == 'prepare_accepted_no_actuation']
    assert len(accepted) == 5
    assert {e['event'] for e in events} >= {'unarmed_prepare_rejected', 'wrong_task_rejected'}
    faults = [('invalid',4), ('speed',64), ('position',32), ('drop',2), ('replay',2)]
    latencies = {}
    for fault, bit in faults:
        revoked = next(e for e in events if e['event'] == fault+'_revoked')
        refused = next(e for e in events if e['event'] == fault+'_prepare_rejected')
        assert revoked['injected_permission_fault']
        assert revoked['status']['request_result'] == 3 and not revoked['status']['prepared']
        assert int(revoked['status']['denial_mask']) & bit
        assert refused['status']['request_result'] == 2 and not refused['status']['prepared']
        assert refused['sim_s'] > revoked['sim_s']
        assert 0 <= revoked['observed_latency_sim_s'] <= 1
        latencies[fault] = revoked['observed_latency_sim_s']
    recovered = [e for e in events if e['event'] == 'recovery_no_auto_prepare']
    assert len(recovered) == 5 and all(not e['status']['prepared'] for e in recovered)
    initial = probe['initial_arm_target']; final = probe['final_arm_state']
    assert initial['target_sequence'] == final['target_sequence']
    assert len(initial['joint_target']) == len(final['joint_target']) == 6
    assert max(abs(a-b) for a,b in zip(initial['joint_target'],final['joint_target'])) <= 1e-6
    assert final['grasp_phase'] == 0 and not final['grasp_ownership_locked']
    assert all(not r['close_permitted'] and r['fold_error_rad'] <= .08 for r in flight)
    assert all(math.isfinite(x) for r in flight for x in r['body_world'])
    return dict(verified=True, accepted_prepare_count=len(accepted),
        fault_revocation_latency_sim_s=latencies, arm_target_unchanged=True,
        motion_and_close_authorization=False, physical_grasp_tested=False,
        hold_sim_seconds=result['hold_sim_seconds'], hold_max_error_m=result['hold_max_error_m'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('folder', type=Path)
    print(json.dumps(verify(p.parse_args().folder), indent=2))
