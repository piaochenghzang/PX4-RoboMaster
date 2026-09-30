"""Observe firmware-owned grasping independently; never publish arm targets.

Only START/CANCEL requests are written. No binding, teleport or runtime scene
mutations. Failed trials are preserved by the caller alongside the raw CSV.
"""
import math
import time
import numpy as np

PHASES=['idle','precheck','approach','align','close','verify','exit_support',
        'lift','retract_clear','retract','settle','hold','done','aborted','fault']


def exit_retention_checks(data,initial,cube_local,contact_summary):
    """Independent physical hold proof; FAULT/UNKNOWN itself proves no retention."""
    if not data or initial is None: return dict(retained=False)
    contact=contact_summary(data)
    reference=cube_local(initial)
    separations=[float(np.linalg.norm(cube_local(row)-reference)) for row in data]
    finite=all(all(math.isfinite(row[key]) for key in
                   ['cx','cy','cz','gx','gy','gz','jaw_angle','jaw_speed','jaw_torque_Nm',
                    'fixed_normal_N','moving_normal_N','bilateral_fraction','cube_support_fraction',
                    'cube_other_fraction','support_arm_fraction']) for row in data)
    lost_since=None;continuous_contact=True
    for row in data:
        if row['bilateral_fraction']<.5:
            if lost_since is None: lost_since=row['sim_s']
            elif row['sim_s']-lost_since>.5: continuous_contact=False
        else: lost_since=None
    min_height=min(row['cz']-initial['cz'] for row in data)
    peak_torque=max(abs(row['jaw_torque_Nm']) for row in data)
    unobstructed=max(row['cube_other_fraction'] for row in data)<=.2 and max(row['support_arm_fraction'] for row in data)<=.2
    # Arm watchdogs can sag after measured-position hold; record world height,
    # but this check concerns grip retention, not fixed end-effector accuracy.
    retained=bool(finite and unobstructed and continuous_contact and max(separations)<=.045
                  and contact['bilateral_fraction']>=.9 and contact['cube_support_fraction']<=.001
                  and min(contact['fixed_normal_N'],contact['moving_normal_N'])>.01
                  and peak_torque<=.010001)
    return dict(retained=retained,contact=contact,max_relative_separation_m=max(separations),
                final_relative_separation_m=separations[-1],min_height_from_grasp_m=min_height,
                max_abs_jaw_torque_Nm=peak_torque,finite=finite,unobstructed=unobstructed,
                continuous_contact=continuous_contact,
                observed_sim_seconds=data[-1]['sim_s']-data[0]['sim_s'])


def run_auto_case(args,folder,client,sample,status,event,wait_sim,rows,cube_local,contact_summary,retention_checks,proxy=None):
    start=sample()['sim_s']
    while True:
        r=sample();f=status('arm_grasp_feedback')
        if 'timestamp_sample' in f:
            age=r['sim_s']-f['timestamp_sample']*1e-6
            clock_delta=(f['timestamp']-f['timestamp_sample'])*1e-6
            if (-.15<=age<=.3 and -.02<=clock_delta<=.2
                and (f.get('capabilities')==31 or args.fsm_test=='missing_object')): break
        if r['sim_s']-start>10:
            raise RuntimeError('grasp feedback capability/clock readiness failed: '+str(f))
        time.sleep(.1)
    event('grasp_feedback_ready',r,source_age_s=age,capabilities=f.get('capabilities'),
          source_sequence=f.get('source_sequence'),world_epoch=f.get('world_epoch'),receipt_minus_sample_s=clock_delta)
    arm=status();measured=arm['ee_measured']
    # Independent actual link pose versus firmware FK. The world transform is
    # deliberately restricted to this fixed upside-down bench installation.
    fk_row=dict(r,cx=measured[0],cy=-measured[1],cz=1.15-measured[2])
    frame_error=float(np.linalg.norm(cube_local(fk_row)-np.array([-.0079,-.000218121,-.0981274])))
    source_error=(float(np.linalg.norm(np.array(f['object_center_gripper'])-cube_local(r)))
                  if r['cube_seen'] else None)
    event('grasp_feedback_frames_verified',r,fk_frame_error_m=frame_error,source_object_frame_error_m=source_error)
    if frame_error>.003 or (source_error is not None and source_error>.003):
        raise RuntimeError('independent FK/grasp-feedback frame mismatch')
    if args.fsm_test=='feedback_only':
        reference=f['source_sequence'];end=wait_sim(5);after=status('arm_grasp_feedback')
        healthy=(after.get('capabilities')==31 and after.get('source_sequence',0)>reference+20
                 and abs(end['sim_s']-after.get('timestamp_sample',0)*1e-6)<.3)
        return dict(success=bool(healthy),execution_status='feedback_checked',grasp_validated=False,
                    source_sequence_delta=after.get('source_sequence',0)-reference)
    before=status('arm_grasp_status').get('last_request_id',0)
    client('arm_control','grasp_start','bench_cube20',args.hold_seconds)
    event('automatic_grasp_requested',sample(),hold_seconds=args.hold_seconds,test=args.fsm_test)
    last_phase=-1;cancel_sent=False;initial=None;hold_reference=None;hold_start=None;fault_start=None
    owned_id=None;ownership_verified=False
    while True:
        r=sample();s=status('arm_grasp_status')
        if r['sim_s']-start>450: raise RuntimeError('automatic grasp sim-time watchdog')
        phase=int(s.get('phase',0));result=int(s.get('result',0))
        if s.get('last_request_id',0)<=before:
            if r['sim_s']-start>15: raise RuntimeError('grasp request not acknowledged')
            time.sleep(.1);continue
        if args.fsm_test=='missing_object':
            rejected=(phase==0 and s.get('last_request_result')==3 and not s.get('ownership_locked'))
            if not rejected: raise RuntimeError('missing-object request was not safely rejected')
            end=wait_sim(2);after=status('arm_grasp_status')
            passed=after.get('phase')==0 and not after.get('completed_success')
            event('missing_object_rejected',end,status=after)
            return dict(success=bool(passed),execution_status='negative_case_verified',
                        negative_case=args.fsm_test,grasp_validated=False,final_grasp_status=after)
        if owned_id is None:
            if s.get('last_request_result')!=1: raise RuntimeError('START rejected: '+str(s))
            owned_id=s.get('active_request_id')
        if s.get('active_request_id')!=owned_id: raise RuntimeError('active grasp request was replaced')
        if phase!=last_phase:
            event('fsm_phase_changed',r,phase=PHASES[phase],status=s);last_phase=phase
            if phase==6: initial=r.copy()
            if phase==11:
                hold_start=r['sim_s']-s.get('phase_elapsed_s',0)
                candidates=[x for x in rows(folder/'poses.csv') if x['sim_s']>=hold_start]
                hold_reference=cube_local(candidates[0] if candidates else r)
                event('unsupported_hold_started',r,hold_seconds=args.hold_seconds,
                      firmware_start_sim_s=hold_start,cube_lift_m=s.get('actual_lift'))
        if args.fsm_test=='ownership' and phase==11 and not ownership_verified:
            client('arm_control','grasp_start','bench_cube20',5);wait_sim(.3)
            busy=status('arm_grasp_status')
            if busy.get('last_request_result')!=2 or busy.get('active_request_id')!=owned_id or busy.get('phase')!=11:
                raise RuntimeError('duplicate START was not rejected as BUSY')
            for command in [('target',.12,.015,.03,.6),('test_cartesian_ik',)]:
                try: client('arm_control',*command)
                except RuntimeError as exc:
                    if 'BUSY' not in str(exc): raise
                else: raise RuntimeError('manual/mutating command unexpectedly accepted')
            client('arm_control','grasp_reset');wait_sim(.3)
            reset=status('arm_grasp_status')
            if reset.get('last_request_result')!=3 or reset.get('phase')!=11 or not reset.get('ownership_locked'):
                raise RuntimeError('RESET unexpectedly released running grasp')
            ownership_verified=True;event('command_ownership_verified',sample(),active_request_id=owned_id)
        cancel_phase=2 if args.fsm_test=='cancel_approach' else 7
        moving_started=args.fsm_test!='cancel_approach' or s.get('target_sequence',0)>0
        if args.fsm_test in ['cancel_approach','cancel_carry'] and phase==cancel_phase and moving_started and not cancel_sent:
            jaw_before=r['jaw_angle'];client('arm_control','grasp_cancel');cancel_sent=True
            event('cancel_requested',r,jaw_angle=jaw_before)
        if phase==13:
            if not cancel_sent or result!=3 or not s.get('ownership_locked'):
                raise RuntimeError('unexpected/unsafe cancellation')
            stopped=status();exit_start=r['sim_s']
            end=wait_sim(args.exit_observe_seconds);after=status('arm_grasp_status');arm=status()
            locked=after.get('phase')==13 and after.get('ownership_locked') and not after.get('completed_success')
            command_shift=max(abs(a-b) for a,b in zip(stopped['joint_target'][:5],arm['joint_target'][:5]))
            locked=locked and command_shift<1e-5
            if args.fsm_test=='cancel_carry': locked=locked and abs(end['jaw_angle']-jaw_before)<.08
            physical=exit_retention_checks([x for x in rows(folder/'poses.csv') if exit_start<=x['sim_s']<=end['sim_s']],
                                           initial,cube_local,contact_summary) if args.fsm_test=='cancel_carry' else None
            passed=locked and (physical['retained'] if physical else True)
            event('cancel_exit_verified',end,status=after,jaw_delta=end['jaw_angle']-jaw_before,
                  max_arm_target_change_rad=command_shift,physical_retention=physical)
            return dict(success=bool(passed),execution_status='negative_case_verified',
                        exit_control_verified=bool(locked),exit_retention=physical,
                        negative_case=args.fsm_test,grasp_validated=False,final_grasp_status=after)
        if args.fsm_test=='drop_command' and phase==11 and fault_start is None:
            if proxy is None: raise RuntimeError('missing test bridge')
            proxy.drop_id=42000;fault_start=r['sim_s']
            event('arm_command_drop_started',r,message_id=42000)
        if args.fsm_test in ['drop_grasp','drop_joint'] and phase==7 and fault_start is None:
            if proxy is None: raise RuntimeError('missing test bridge')
            proxy.drop_id=42002 if args.fsm_test=='drop_grasp' else 42001
            fault_start=r['sim_s'];jaw_before=r['jaw_angle']
            event('feedback_drop_started',r,message_id=proxy.drop_id)
        if phase==14:
            expected=3 if args.fsm_test=='drop_grasp' else 2
            if fault_start is None or s.get('fault_reason')!=expected or result!=4:
                raise RuntimeError('firmware grasp fault: '+str(s))
            observed_latency=r['sim_s']-fault_start
            latency=((s['timestamp']-proxy.first_drop_clock_us)*1e-6
                     if proxy.first_drop_clock_us else observed_latency)
            stopped=status();command_stop=status('arm_joint_command')
            exit_start=r['sim_s']
            restored=False;stopped_until_restore=True
            def fault_observer(row):
                nonlocal restored,stopped_until_restore
                if (args.restore_feedback_after is not None and not restored
                    and row['sim_s']-exit_start>=args.restore_feedback_after):
                    if args.fsm_test=='drop_joint':
                        stopped_until_restore=status('arm_joint_command').get('timestamp')==command_stop.get('timestamp')
                    proxy.drop_id=None;restored=True
                    event('feedback_restored',row,automatic_resume_allowed=False)
            end=wait_sim(args.exit_observe_seconds,validate=fault_observer);after=status('arm_grasp_status');arm=status()
            command_after=status('arm_joint_command')
            kept=(after.get('phase')==14 and after.get('ownership_locked') and not after.get('completed_success')
                  and after.get('payload_state')==5 and proxy.dropped[42002 if args.fsm_test=='drop_grasp' else 42001]>0 and latency<1.
                  and after.get('target_sequence')==s.get('target_sequence'))
            if args.fsm_test=='drop_grasp':
                target_shift=max(abs(a-b) for a,b in zip(stopped['joint_target'][:5],arm['joint_target'][:5]))
                kept=kept and abs(end['jaw_angle']-jaw_before)<.08 and arm.get('feedback_valid') and target_shift<1e-5
            else:
                kept=kept and (stopped_until_restore and arm.get('feedback_valid') if restored
                               else command_after.get('timestamp')==command_stop.get('timestamp'))
            if args.restore_feedback_after is not None: kept=kept and restored
            event('feedback_fault_exit_verified',end,status=after,latency_sim_s=latency,
                  bilateral_fraction=end['bilateral_fraction'],jaw_delta=end['jaw_angle']-jaw_before)
            physical=exit_retention_checks([x for x in rows(folder/'poses.csv') if exit_start<=x['sim_s']<=end['sim_s']],
                                           initial,cube_local,contact_summary)
            return dict(success=bool(kept and physical['retained']),execution_status='negative_case_verified',
                        exit_control_verified=bool(kept),exit_retention=physical,
                        feedback_restored=restored,stopped_until_restore=stopped_until_restore,
                        negative_case=args.fsm_test,grasp_validated=False,fault_latency_sim_s=latency,
                        observer_fault_latency_sim_s=observed_latency,final_grasp_status=after,
                        exit_bilateral_fraction=end['bilateral_fraction'],exit_payload_retained=physical['retained'],
                        exit_relative_separation_m=physical['final_relative_separation_m'])
        if phase==12:
            if args.fsm_test not in ['normal','ownership','drop_command'] or result!=2 or not s.get('completed_success'):
                raise RuntimeError('unexpected completion')
            if initial is None or hold_start is None or hold_reference is None:
                raise RuntimeError('independent observer missed a required grasp phase')
            end=wait_sim(2);after=status('arm_grasp_status')
            if args.fsm_test=='ownership':
                client('arm_control','grasp_reset');wait_sim(.3)
                after=status('arm_grasp_status')
                if after.get('last_request_result')!=4 or after.get('phase')!=12 or not after.get('ownership_locked'):
                    raise RuntimeError('DONE payload RESET not locked')
            held=[x for x in rows(folder/'poses.csv') if hold_start<=x['sim_s']<=end['sim_s']]
            if not held: raise RuntimeError('no independent unsupported hold data')
            contact=contact_summary(held)
            late=float(np.linalg.norm(cube_local(end)-hold_reference))
            peak=max(float(np.linalg.norm(cube_local(x)-hold_reference)) for x in held)
            min_lift=float(min(x['cz']-initial['cz'] for x in held))
            retained,strict=retention_checks(contact,late,peak,min_lift)
            finite=all(all(math.isfinite(x[k]) for k in ['cx','cy','cz','gx','gy','gz','jaw_angle','jaw_speed']) for x in held)
            unobstructed=max(x['cube_other_fraction'] for x in held)<=.2 and max(x['support_arm_fraction'] for x in held)<=.2
            duration=end['sim_s']-hold_start
            verified=bool(retained and finite and unobstructed and duration>=args.hold_seconds-.3
                          and after.get('phase')==12 and after.get('result')==2)
            passed=verified and (strict if args.acceptance=='strict' else True)
            if args.fsm_test=='drop_command':
                passed=passed and fault_start is not None and proxy.dropped[42000]>20
            data=dict(success=bool(passed),execution_status='automatic_grasp_completed',
                      grasp_validated=bool(passed),strict_retention_passed=bool(strict),hold_sim_seconds=duration,
                      min_hold_lift_m=min_lift,late_relative_drift_m=late,max_hold_drift_m=peak,
                      contact=contact,final_grasp_status=after,ownership_verified=ownership_verified,
                      command_delivery_validated=args.fsm_test!='drop_command')
            event('retention_complete',end,**data);return data
        time.sleep(.1)
