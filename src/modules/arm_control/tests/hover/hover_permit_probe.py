"""No-close local SITL tests of native hover PREPARE, refusal and revocation."""
import json
import math
import threading
import numpy as np
from known_target import rotation
from native_client import NativeClient

CUBE_REFERENCE = np.array([-.0079, -.000218121, -.0981274])


def prepare_outcome(status, previous_id):
    if status.get('last_request_id', 0) <= previous_id: return 'pending'
    result = status.get('request_result')
    if result in (2, 3) and not status.get('prepared'):
        return 'rejected' if result == 2 else 'revoked'
    if result == 1 and status.get('prepared') and status.get('start_ready'): return 'accepted'
    raise RuntimeError('Inconsistent PREPARE result')


def metrics(row, target, reference=CUBE_REFERENCE):
    """Same-step physical geometry; velocity includes omega cross reference offset."""
    truth = row['truth']
    names = list(truth.link_names)
    if ('so101::gripper_link' not in names or not truth.gripper_velocity_valid
            or not truth.target_velocity_valid):
        raise RuntimeError('Missing actual gripper/object velocity')
    g = truth.link_poses[names.index('so101::gripper_link')]
    gp = np.array([g.position.x, g.position.y, g.position.z])
    gr = rotation([g.orientation.w, g.orientation.x, g.orientation.y, g.orientation.z])
    obj = truth.target_pose
    op = np.array([obj.position.x, obj.position.y, obj.position.z])
    objr = rotation([obj.orientation.w, obj.orientation.x, obj.orientation.y, obj.orientation.z])
    def xyz(v): return np.array([v.x, v.y, v.z])
    offset = gr @ reference
    reference_v = xyz(truth.gripper_linear_velocity_world) + np.cross(
        xyz(truth.gripper_angular_velocity_world), offset)
    bodyr = rotation(row['body'][3:])
    result = [
        float(np.linalg.norm(row['actual_v'])), float(np.linalg.norm(row['actual_w'])),
        math.acos(float(np.clip(bodyr[2, 2], -1, 1))),
        float(np.linalg.norm(np.asarray(row['body'][:3]) - target.hover[:3, 3])),
        float(np.linalg.norm(op - gp - offset)),
        float(np.linalg.norm(xyz(truth.target_linear_velocity_world) - reference_v)),
        math.acos(float(np.clip((np.trace(gr.T @ objr) - 1) / 2, -1, 1)))]
    if not np.isfinite(result).all(): raise RuntimeError('Nonfinite hover geometry')
    return result


class HoverPermitProbe(NativeClient):
    """Fault injection affects permission inputs ONLY, never vehicle/arm setpoints."""
    FAULTS = [('invalid', 4), ('speed', 64), ('position', 32), ('drop', 2), ('replay', 2)]

    def __init__(self, build, env, sources, target, folder):
        super().__init__(build, env)
        self.sources, self.target = sources, target
        self.log = (folder / 'hover_status.jsonl').open('x')
        self.events = []; self.complete = False; self.error = None; self.latest = {}
        self.stop_event = threading.Event()
        self.stage = 'unarmed'; self.fault = None; self.fault_index = 0
        self.last_source = None; self.last_context = None
        self.last_status = -math.inf; self.last_invariant = -math.inf
        self.expected_request = 0; self.wrong_task_checked = False
        self.streaming = False
        self.baseline = self.status('arm_control_status')
        self.assert_unchanged()
        self.thread = threading.Thread(target=self.run, daemon=True)

    def assert_unchanged(self):
        arm = self.status('arm_control_status'); grasp = self.status('arm_grasp_status')
        if not arm.get('feedback_valid') or arm.get('feedback_age', math.inf) > .2:
            raise RuntimeError('Native joint feedback unhealthy')
        if (arm.get('target_sequence') != self.baseline.get('target_sequence') or
            np.max(np.abs(np.asarray(arm['joint_target']) - self.baseline['joint_target'])) > 1e-6 or
            grasp.get('phase') != 0 or grasp.get('ownership_locked')):
            raise RuntimeError('PREPARE changed an arm target or entered grasp')
        return dict(target_sequence=arm['target_sequence'], joint_target=arm['joint_target'],
            grasp_phase=grasp['phase'], grasp_ownership_locked=grasp['ownership_locked'])

    def event(self, name, row, status, **extra):
        self.events.append(dict(event=name, sim_s=row['now'], status=status, **extra))
        print('HOVER PERMIT: ' + name, flush=True)

    def request(self, task, status):
        self.expected_request = status.get('last_request_id', 0)
        self.client('arm_control', 'hover_grasp_prepare', task)

    def acknowledged(self, s, result):
        return s.get('last_request_id', 0) > self.expected_request and s.get('request_result') == result

    def run(self):
        try:
            while not self.stop_event.wait(.002):
                row = self.sources.read()
                if row is None: continue  # No source means native freshness expires.
                now = row['now']; truth = row['truth']
                source = (truth.header.seq, truth.header.stamp.to_nsec() // 1000)
                # Do not bind a task through PX4's expected in-flight heading
                # reset. First context starts AFTER native final yaw alignment.
                # Once bound, any later reset still latches the gate closed.
                if (not self.streaming and row['manager'] == 'TRACK_KNOWN'
                        and self.latest.get('native_heading_ready')):
                    self.streaming = True
                    self.event('context_bound_after_native_heading_alignment', row, self.latest)
                if self.streaming and source != self.last_source:
                    self.last_source = source
                    fields = [1, source[0], source[1], truth.world_epoch, 1, *metrics(row, self.target)]
                    if self.fault == 'invalid': fields[4] = 0
                    if self.fault == 'speed': fields[5] = max(fields[5], .06)
                    if self.fault == 'position': fields[8] = max(fields[8], .04)
                    if self.fault == 'replay': fields = self.last_context
                    else: self.last_context = fields
                    if self.fault != 'drop': self.client('arm_control', 'hover_context', *fields)
                if now - self.last_status < .1: continue
                self.last_status = now; s = self.status('arm_hover_status'); self.latest = s
                if s.get('motion_authorized') or s.get('close_authorized'):
                    raise RuntimeError('Unexpected actuation authorization')
                self.log.write(json.dumps(dict(sim_s=now, stage=self.stage, fault=self.fault, status=s)) + '\n')
                self.log.flush()
                if now - self.last_invariant >= 1:
                    self.assert_unchanged(); self.last_invariant = now
                if s.get('reset_blocked'): raise RuntimeError('Native reset gate latched; new reviewed task required')
                if self.stage == 'unarmed':
                    if row['state'].armed: raise RuntimeError('Missed unarmed negative test')
                    self.request(1, s); self.stage = 'unarmed_ack'
                elif self.stage == 'unarmed_ack' and self.acknowledged(s, 2):
                    if s.get('prepared'): raise RuntimeError('Unarmed PREPARE accepted')
                    self.event('unarmed_prepare_rejected', row, s); self.stage = 'ready'
                elif self.stage == 'ready' and s.get('start_ready'):
                    if not self.wrong_task_checked:
                        self.request(2, s); self.stage = 'wrong_ack'
                    else:
                        self.request(1, s); self.stage = 'accept_ack'
                elif self.stage == 'wrong_ack' and self.acknowledged(s, 2):
                    self.event('wrong_task_rejected', row, s)
                    self.wrong_task_checked = True; self.stage = 'ready'
                elif self.stage == 'accept_ack' and s.get('last_request_id', 0) > self.expected_request:
                    outcome = prepare_outcome(s, self.expected_request)
                    if outcome in ('rejected', 'revoked'):
                        self.event('prepare_' + outcome + '_before_observation', row, s)
                        self.stage = 'ready'; continue  # Never count a fleeting ticket as an acceptance.
                    self.event('prepare_accepted_no_actuation', row, s, next_fault=self.FAULTS[self.fault_index][0])
                    self.fault, self.expected_denial = self.FAULTS[self.fault_index]
                    self.fault_start = now; self.stage = 'fault'
                elif self.stage == 'fault':
                    if (not s.get('prepared') and s.get('request_result') == 3
                            and int(s.get('denial_mask', 0)) & self.expected_denial):
                        self.event(self.fault + '_revoked', row, s,
                            observed_latency_sim_s=now-self.fault_start, injected_permission_fault=True)
                        self.request(1, s); self.stage = 'refuse_ack'
                    elif now-self.fault_start > 1:
                        raise RuntimeError(self.fault + ' did not revoke PREPARE')
                elif self.stage == 'refuse_ack' and self.acknowledged(s, 2):
                    if s.get('prepared'): raise RuntimeError('Faulted PREPARE accepted')
                    self.event(self.fault + '_prepare_rejected', row, s)
                    self.fault = None; self.stage = 'recover'
                elif self.stage == 'recover' and s.get('start_ready'):
                    if s.get('prepared'): raise RuntimeError('Automatic prepare after recovery')
                    self.event('recovery_no_auto_prepare', row, s)
                    self.fault_index += 1
                    if self.fault_index == len(self.FAULTS):
                        self.stage = 'done'; self.complete = True
                    else: self.stage = 'ready'
                if self.stage == 'done' and s.get('prepared'):
                    raise RuntimeError('Unexpected automatic prepare')
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.log.close()

    def start(self): self.thread.start()

    def finish(self):
        self.stop_event.set(); self.thread.join(timeout=6)
        if self.thread.is_alive(): raise RuntimeError('Hover probe did not stop')
        final_arm_state = self.assert_unchanged()
        if self.error: raise RuntimeError(self.error)
        return dict(hover_prepare_validated=self.complete, events=self.events,
            final_status=self.latest, arm_target_unchanged=True,
            initial_arm_target=dict(target_sequence=self.baseline['target_sequence'],
                joint_target=self.baseline['joint_target']), final_arm_state=final_arm_state,
            arm_motion_authorized=False, close_authorized=False, grasp_executed=False)
