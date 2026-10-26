"""Bounded fixed-point flight grasp coordinator; native ArmControl owns joints."""
import json
import math
import threading
import time
from collections import deque
from pathlib import Path
import numpy as np
import yaml
from known_target import pose,inverse
from native_client import NativeClient
from hover_permit_probe import metrics
from hover_approach_probe import ApproachPlan,token,pose_values
from fixed_demo_plan import SUPPORT_HALF,OPEN_REFERENCE,LIFT_DISTANCE

REFERENCE=OPEN_REFERENCE


class ObservationCursor:
    """A duplicate truth frame must not consume a pending native status ACK."""
    def __init__(self):
        self.source=None;self.serial=0

    def take(self,source,snapshot):
        if source==self.source or snapshot is None:return None
        self.source=source
        changed=snapshot[0]!=self.serial;self.serial=snapshot[0]
        return changed


class StabilityWindow:
    """One sim-second of uninterrupted evidence; a source gap resets dwell."""
    def __init__(self):self.since=None;self.last=None

    def update(self,now,ready):
        if not math.isfinite(now) or now<0 or (self.last is not None and now<self.last):
            raise ValueError('Invalid stability-window clock')
        gap=self.last is not None and now-self.last>.100000001
        self.last=now
        if not ready or gap:self.since=None;return False
        if self.since is None:self.since=now
        return now-self.since>=1.


def waypoint_dwell_seconds(index,count):
    # OPEN-jaw approach points are passages, not grasp permits. Keep their
    # instantaneous distance/motion checks; leave final closure to native Align.
    return 0. if 0<=index<count-1 else 1.


def world_progress(current,goal,elapsed,enabled=True,speed=.02):
    current=np.asarray(current,dtype=float);goal=np.asarray(goal,dtype=float)
    if current.shape!=(3,) or goal.shape!=(3,) or not np.isfinite([*current,*goal,elapsed,speed]).all() or elapsed<0 or not 0<speed<=.02:
        raise ValueError('Invalid world trajectory state/time')
    delta=goal-current;distance=float(np.linalg.norm(delta))
    step=min(speed*.1,speed*elapsed) if enabled else 0.
    return current+delta*min(1.,step/max(distance,1e-12))


def lateral_center(correction,rotation,object_x):
    if not np.isfinite([*correction,*np.asarray(rotation).ravel(),object_x]).all():
        raise ValueError('Invalid lateral centering evidence')
    error=object_x-REFERENCE[0]
    if abs(error)<=.0005:return np.asarray(correction).copy()
    delta=np.asarray(rotation)[:,0]*float(np.clip(.1*error,-.0001,.0001))
    candidate=np.asarray(correction)+delta;length=float(np.linalg.norm(candidate))
    return candidate*min(1.,.005/max(length,1e-12))


def may_trim(phase,close_requested,paused,hover_ready):
    # A revocable hover pause must also freeze the coordinator's endpoint.
    return phase in (1,2) and not close_requested and not paused and bool(hover_ready)


def compensation_speed(body_position,linear_velocity,angular_velocity,tool_position):
    """World-fixed TCP compensation demand from the SAME physical sample."""
    values=[np.asarray(v,dtype=float) for v in
        (body_position,linear_velocity,angular_velocity,tool_position)]
    if any(v.shape!=(3,) or not np.isfinite(v).all() for v in values):
        raise ValueError('Invalid compensation velocity evidence')
    body,linear,angular,tool=values
    return float(np.linalg.norm(linear+np.cross(angular,tool-body)))


def precision_motion_ready(compensation,arm_error,world_error,joint_speed,body_error,*,body_limit=.005):
    values=np.asarray([compensation,arm_error,world_error,joint_speed,body_error,body_limit],dtype=float)
    # Keep compensation below 20mm/s of the 50mm/s native capacity,
    # reserving room for insertion/correction. This gates WORLD route progress,
    # never body-motion compensation or native safety checks.
    return bool(np.isfinite(values).all() and np.all(values>=0) and
        0<body_limit<=.010 and compensation<=.020 and arm_error<=.003 and world_error<=.003 and joint_speed<=.08 and body_error<=body_limit)


def unfold_motion_ready(compensation,arm_error,joint_speed,body_error):
    """Admission by actual TCP demand, with native capacity left for correction."""
    values=np.asarray([compensation,arm_error,joint_speed,body_error],dtype=float)
    return bool(np.isfinite(values).all() and np.all(values>=0) and
        compensation<=.020 and arm_error<=.003 and joint_speed<=.05 and body_error<=.005)


def centered_world_target(tool,rotation,local):
    """Correct the measured <=3mm entry residual before freezing the goal."""
    return np.asarray(tool)+np.asarray(rotation)@(np.asarray(local)-REFERENCE)


def insertion_goal(current,desired,rotation,local,relative_rotation,jaw,orientation_error):
    """OPEN-jaw plane corridor only; native 3/16mm obstacle budgets still apply."""
    values=[*current,*desired,*np.asarray(rotation).ravel(),*local,
        *np.asarray(relative_rotation).ravel(),jaw,orientation_error]
    if not np.isfinite(values).all() or not .29<=jaw<=.301:
        raise ValueError('Invalid open insertion evidence')
    half_x=.01*sum(abs(np.asarray(relative_rotation)[0,:]))
    # Production pads: fixed inner plane x=-8mm; moving pad rotated about -Y.
    # Its conservative min-X plane at q is 20.2 -12*cos(q) +58*sin(q), mm.
    moving=.0202-.012*math.cos(jaw)+.058*math.sin(jaw)
    gap=min(local[0]-half_x+.008,moving-local[0]-half_x)
    if gap>=.001 and orientation_error<=math.radians(5):
        return np.asarray(desired),float(gap),True
    # No forward insertion while side clearance/orientation is insufficient.
    # Only a slow lateral correction is allowed, with the same native budgets.
    axis=np.asarray(rotation)[:,0];delta=np.asarray(desired)-current
    return np.asarray(current)+axis*float(delta@axis),float(gap),False


def context_geometry(truth,unfold_complete,closing=False):
    valid=truth.fixed_grasp_geometry_valid and not truth.fixed_grasp_forbidden_contact
    clearance=truth.fixed_grasp_clearance if closing else truth.fixed_grasp_current_clearance
    if not unfold_complete:
        # No target-contact exception while selecting the IK branch. Reuse
        # the original ALL-pairs 16mm geometry and external-contact observer.
        valid=valid and truth.approach_geometry_valid and not truth.arm_external_contact
        clearance=min(clearance,truth.arm_obstacle_clearance)
    return bool(valid),float(clearance)


def pair_feedback(history,stamp,identity):
    """Bracket both rigid poses at RAW Gazebo sample time; never extrapolate."""
    from grasp_audit.geometry import interpolate_pose,pose_support
    # Validate ALL source times/identity, but only convert the two bracket
    # poses. Rebuilding hundreds of unused matrices can delay ROS callbacks.
    for t in history:
        if (t.source_instance,t.world_epoch,t.vehicle_id,t.target_id)!=identity:
            raise ValueError('TRUTH_IDENTITY_CHANGED')
    before,after,_=pose_support([(t.header.stamp.to_sec(),t) for t in history],stamp,.025)
    bracket=[before[1]] if before[0]==after[0] else [before[1],after[1]]
    gripper=[];objects=[]
    for t in bracket:
        names=list(t.link_names)
        g=pose_values(t.link_poses[names.index('so101::gripper_link')])
        gripper.append((t.header.stamp.to_sec(),g))
        objects.append((t.header.stamp.to_sec(),pose_values(t.target_pose)))
    G,skew=interpolate_pose(gripper,stamp,.025)
    O,other_skew=interpolate_pose(objects,stamp,.025)
    return (inverse(G)@np.r_[O[:3,3],1])[:3],max(skew,other_skew)


class FixedBinding(ApproachPlan):
    def __init__(self,template,truth):
        super().__init__(template.target,truth)
        self.waypoints_world=template.waypoints_world
        self.pre_world=template.grasp_world
        self.plan=token([template.config,[p.tolist() for p in self.waypoints_world],
            SUPPORT_HALF.tolist(),'fixed_grasp_pair_rules_16_3_v1'])
        self.task=token([self.plan,self.scene,self.target]);self.ids=[self.task,self.plan,self.target,self.scene]


class FixedGraspProbe(NativeClient):
    def __init__(self,build,env,sources,template,folder,*,expected_pid):
        super().__init__(build,env,transport='unix',expected_pid=expected_pid)
        import sys
        scripts=Path('/home/pcz/super_ws/src/super_px4_bridge/scripts')
        sys.path.insert(0,str(scripts))
        from grasp_audit.geometry import fk_gripper,residual
        self.fk_gripper,self.residual=fk_gripper,residual
        self.config=yaml.safe_load((scripts.parent/'config/grasp_frame_audit.yaml').read_text())
        self.template=template;self.sources=sources;self.target=template.target
        self.plan=FixedBinding(template,sources.read(require_odometry=False)['truth'])
        self.log=(folder/'fixed_grasp_status.jsonl').open('x')
        self.events=[];self.complete=False;self.error=None;self.motion_started=False
        self.approach_request_count=0;self.close_requested=False
        self.stop_event=threading.Event();self.thread=threading.Thread(target=self.run,daemon=True)
        self.feedback_lock=threading.RLock()
        self.feedback_thread=threading.Thread(target=self.read_feedback,daemon=True)
        self.status_lock=threading.RLock();self.status_snapshot=None
        self.status_thread=threading.Thread(target=self.read_status,daemon=True)
        self.streaming=False;self.last_source=None
        self.stage='waiting_hover';self.latest={};self.native={};self.expected=0
        self.correction=np.zeros(3)
        self.pickup_height=None;self.grasp_reference=None;self.done_since=None;self.samples=[]
        self.route_index=0;self.route_stable_since=None
        self.grasp_base=None
        self.unfold_anchor=None;self.unfold_settled=False;self.unfold_window=StabilityWindow()
        self.progress_world=None;self.last_progress_sim=None
        self.last_center_sim=-math.inf
        self.baseline=self.status('arm_control_status')
        official=NativeClient(build,env).status('arm_control_status')
        if np.max(abs(np.asarray(official['joint_target'])-self.baseline['joint_target']))>1e-5:
            raise RuntimeError('Official CLI and local IPC baseline disagree; no flight')
        self.frame_verified=False;self.last_verified_stamp=-math.inf
        self.feedback={};self.arm={};self.checked_feedback=None
        self.frame_error=None;self.pair_skew=None
        self.pending_feedback=deque(maxlen=12);self.seen_feedback=None
        self.last_pair_failure=None
        self.last_geometry_valid=None

    def event(self,name,row,**extra):
        self.events.append(dict(event=name,sim_s=row['now'],**extra))
        print('FIXED GRASP: '+name,flush=True)

    def request(self,command):
        self.client('arm_control',command,*self.plan.ids)
        self.approach_request_count+=1

    def verify_feedback_frame(self,row):
        f=self.feedback;t=row['truth'];sequence=f.get('source_sequence')
        if f and sequence!=self.seen_feedback:
            if f.get('capabilities')!=15:
                raise RuntimeError('Physical grasp feedback capability mismatch: '+str(f.get('capabilities')))
            self.pending_feedback.append(f);self.seen_feedback=sequence
        pending=deque(maxlen=12)
        for f in self.pending_feedback:
            stamp=f.get('sample_time_us',0)*1e-6
            history=self.sources.truth_samples()
            try:
                paired,skew=pair_feedback(history,stamp,
                    (t.source_instance,t.world_epoch,t.vehicle_id,t.target_id))
            except ValueError as error:
                stamps=[h.header.stamp.to_sec() for h in history]
                self.last_pair_failure=dict(reason=str(error),feedback_stamp=stamp,
                    truth_first=min(stamps) if stamps else None,truth_last=max(stamps) if stamps else None,
                    max_truth_gap_s=max((b-a for a,b in zip(stamps,stamps[1:])),default=0))
                pending.append(f) # Preserve it while awaiting the right bracket.
            else:
                error=float(np.linalg.norm(np.asarray(f['object_center_gripper'])-paired))
                if error>.003:
                    raise RuntimeError('Physical grasp feedback frame mismatch: '+str(dict(error_m=error,skew_s=skew)))
                if stamp>=self.last_verified_stamp:
                    self.checked_feedback=f['source_sequence'];self.last_verified_stamp=stamp
                    self.frame_error=error;self.pair_skew=skew
        self.pending_feedback=pending
        self.frame_verified=row['now']-self.last_verified_stamp<=.15

    def read_feedback(self):
        """Keep the 10Hz physical evidence independent of slow status queries."""
        last_poll=-math.inf
        try:
            while not self.stop_event.wait(.002):
                row=self.sources.read(require_odometry=False)
                if row is None:continue
                if row['now']-last_poll>=.02:
                    last_poll=row['now'];feedback=self.status('arm_grasp_feedback')
                    with self.feedback_lock:self.feedback=feedback
        except Exception as exc:
            self.error=str(exc);self.stop_event.set()

    def read_status(self):
        """Slow diagnostics must never block the source-time context stream."""
        last_poll=-math.inf;serial=0
        try:
            while not self.stop_event.wait(.002):
                row=self.sources.read(require_odometry=False)
                if row is None or row['now']-last_poll<.2:continue
                last_poll=row['now']
                latest=self.status('arm_hover_status');native=self.status('arm_approach_status')
                arm=self.status('arm_control_status')
                if row['now']-native['timestamp']*1e-6>.5:
                    raise RuntimeError('Native scheduled status stopped; abort test instead of waiting for hover')
                mode=native.get('mode')
                if mode not in (0,1):raise RuntimeError('Unknown arm approach status mode')
                if self.status('arm_grasp_status').get('ownership_locked') or (mode==0 and native.get('ownership_locked')):
                    raise RuntimeError('Another arm owner entered')
                serial+=1
                with self.status_lock:self.status_snapshot=(serial,latest,native,arm)
        except Exception as exc:
            self.error=str(exc);self.stop_event.set()

    def run(self):
        cursor=ObservationCursor()
        try:
            while not self.stop_event.wait(.002):
                row=self.sources.read(require_odometry=False)
                if row is None:continue
                lease=row['known_status']
                # The fine-grasp frame contract starts at the matching task's
                # terminal hold, not during distant, rotating navigation.
                # Once streaming starts, stale/revoked context still fails.
                if not self.streaming and not (self.plan.matches(lease) and lease.terminal_hold):continue
                iteration_wall=time.monotonic()
                t=row['truth'];now=row['now'];source=(t.header.seq,t.header.stamp.to_nsec()//1000)
                # Do not consume a newly arrived status on an already used
                # truth frame. Its request ACK must survive until the next
                # fresh frame, even when both pollers have different cadences.
                if source==cursor.source:continue
                with self.status_lock:snapshot=self.status_snapshot
                if snapshot is None:continue
                with self.feedback_lock:
                    # Retry pending pairing on THIS new truth bracket before
                    # evaluating freshness, independently of the CLI poll.
                    self.verify_feedback_frame(row)
                    feedback=self.feedback
                    frame_verified=row['now']-self.last_verified_stamp<=.15
                    verified_stamp=self.last_verified_stamp;frame_error=self.frame_error;pair_skew=self.pair_skew
                    paired_sequence=self.checked_feedback;pair_failure=self.last_pair_failure
                if not feedback:continue
                # Truth is 50Hz. Recomputing FK/metrics on the SAME sample at
                # the 2ms polling cadence only competes with ROS callbacks and
                # delays the next genuinely fresh context; it adds no evidence.
                polled=cursor.take(source,snapshot)
                if polled is None:continue
                if polled:_,self.latest,self.native,self.arm=snapshot
                lease=row['known_status'];names=list(t.link_names)
                g=pose_values(t.link_poses[names.index('so101::gripper_link')])
                G=pose(g[:3],g[3:]);A=pose(row['arm'][:3],row['arm'][3:])
                O=pose_values(t.target_pose);local=(inverse(G)@np.r_[O[:3],1])[:3]
                pos,angle=self.residual(A@self.fk_gripper(self.config,row['joints']),G)
                measured=metrics(row,self.target,REFERENCE)
                phase=int(self.native.get('phase',0))
                geometry,clearance=context_geometry(t,bool(self.native.get('unfold_complete')) and self.unfold_settled,3<=phase<=7)
                valid=bool(geometry and clearance>.016 and pos<=.002 and angle<=.001 and frame_verified)
                if self.streaming and not valid and self.last_geometry_valid is not False:
                    self.event('context_geometry_rejected',row,raw_geometry=t.fixed_grasp_geometry_valid,
                        forbidden_contact=t.fixed_grasp_forbidden_contact,fk_position_error=pos,
                        fk_rotation_error=angle,frame_verified=frame_verified,
                        verified_age_s=now-verified_stamp,pair_failure=pair_failure,
                        latest_feedback_sequence=feedback.get('source_sequence'),
                        latest_feedback_sample_us=feedback.get('sample_time_us'),
                        paired_feedback_sequence=paired_sequence)
                self.last_geometry_valid=valid
                if not self.streaming and self.plan.matches(lease) and lease.terminal_hold and self.latest.get('native_heading_ready'):
                    self.streaming=True;self.event('fixed_task_bound',row,binding=self.plan.ids)
                # The two designated pads may contact the designated cube in
                # this fine-grasp profile. Do not add an observer-only abort on
                # that allowed pair; forbidden contacts still invalidate context
                # above, and native close admission still checks live alignment.
                tool=(G@np.r_[[-.0079,-.000218121,-.0981274],1])[:3]
                required_compensation=compensation_speed(row['body'][:3],row['actual_v'],row['actual_w'],tool)
                unfold_ready=None
                if phase==1 and self.native.get('unfold_complete') and not self.unfold_settled:
                    if self.unfold_anchor is None:
                        self.unfold_anchor=np.asarray(self.arm['ee_target'],dtype=float)
                        if self.unfold_anchor.shape!=(3,) or not np.isfinite(self.unfold_anchor).all():
                            raise RuntimeError('Missing native unfold-hold anchor')
                        self.event('unfold_base_hold_started',row,anchor_base=self.unfold_anchor.tolist())
                    # Let the vehicle settle with the arm held relative to its
                    # own base BEFORE enabling world-TCP compensation. This
                    # avoids chasing unfold transients with the arm itself.
                    ready=(valid and not self.native.get('paused') and self.latest.get('start_ready') and
                        unfold_motion_ready(required_compensation,self.arm.get('position_error',math.inf),
                            self.arm.get('max_joint_speed',math.inf),measured[3]))
                    unfold_ready=bool(ready)
                    if self.unfold_window.update(now,ready):
                        self.unfold_settled=True;self.progress_world=tool.copy();self.last_progress_sim=now
                        self.event('unfold_hover_settled',row,body_error_m=measured[3],body_speed_m_s=measured[0],
                            compensation_speed_m_s=required_compensation)
                motion_evidence=(required_compensation,
                    self.arm.get('position_error',math.inf),
                    float(np.linalg.norm(tool-self.progress_world)) if self.progress_world is not None else math.inf,
                    self.arm.get('max_joint_speed',math.inf),measured[3])
                precision_ready=precision_motion_ready(*motion_evidence)
                close_motion_ready=precision_motion_ready(*motion_evidence,body_limit=.010)
                if phase==1 and self.native.get('unfold_complete') and self.route_index<len(self.plan.waypoints_world)-1:
                    distance=float(np.linalg.norm(tool-self.plan.waypoints_world[self.route_index]))
                    ready=distance<=.003 and precision_ready and measured[5]<=.01 and not self.native.get('paused') and self.latest.get('start_ready') and (
                        self.progress_world is not None and np.linalg.norm(self.progress_world-self.plan.waypoints_world[self.route_index])<=.0002)
                    if not ready:self.route_stable_since=None
                    else:
                        if self.route_stable_since is None:self.route_stable_since=now
                        if now-self.route_stable_since>=waypoint_dwell_seconds(self.route_index,len(self.plan.waypoints_world)):
                            self.event('pregrasp_reached',row,waypoint=self.route_index,error_m=distance)
                            self.route_index+=1;self.route_stable_since=None
                final_waypoint=self.route_index==len(self.plan.waypoints_world)-1
                trim_allowed=may_trim(phase,self.stage in ('close_ack','grasp','done'),self.native.get('paused'),self.latest.get('start_ready'))
                if phase==1 and final_waypoint and trim_allowed and measured[4]<.08 and self.arm.get('position_error',math.inf)<=.002 and np.linalg.norm(tool-self.progress_world)<=.003 and measured[5]<=.01 and now-self.last_center_sim>=.1:
                    # Correct only the finger-gap direction during insertion.
                    # Do not turn the remaining insertion distance into a trim.
                    self.correction=lateral_center(self.correction,G[:3,:3],local[0])
                    self.last_center_sim=now
                desired_world=self.plan.waypoints_world[self.route_index]+self.correction
                corridor_gap=None;insertion_ready=False
                progress_goal=desired_world
                if self.progress_world is not None and phase==1 and final_waypoint:
                    progress_goal,corridor_gap,insertion_ready=insertion_goal(self.progress_world,desired_world,
                        G[:3,:3],local,G[:3,:3].T@pose(O[:3],O[3:])[:3,:3],row['joints'][5],measured[6])
                if self.progress_world is not None and now-self.last_progress_sim>=.1:
                    enabled=phase==1 and trim_allowed and (
                        np.linalg.norm(tool-self.progress_world)<=.005 and self.arm.get('position_error',math.inf)<=.005)
                    if final_waypoint:enabled=enabled and precision_ready
                    self.progress_world=world_progress(self.progress_world,progress_goal,now-self.last_progress_sim,
                        enabled,.002 if final_waypoint else .02)
                    self.last_progress_sim=now
                goal_world=self.progress_world if self.progress_world is not None else desired_world
                base_hold=phase==1 and self.unfold_anchor is not None and not self.unfold_settled
                if base_hold:goal_world=(A@np.r_[self.unfold_anchor,1])[:3]
                base=(inverse(A)@np.r_[goal_world,1])[:3]
                # Track WORLD until both pads grip. Once native Verify confirms
                # bilateral contact, stop pushing against the support through
                # world-position corrections and hold relative to the body.
                if 4<=phase<=7:
                    if self.grasp_base is None:self.grasp_base=base.copy()
                    base=self.grasp_base.copy()
                    goal_world=(A@np.r_[base,1])[:3]
                lift=(inverse(A)@np.r_[goal_world+[0,0,LIFT_DISTANCE],1])[:3]
                timing=None
                if self.streaming and source!=self.last_source:
                    self.last_source=source
                    fresh=(self.plan.matches(lease) and lease.terminal_hold and lease.lease_valid and
                        -.02<=now-lease.header.stamp.to_sec()<=.1 and -.02<=now-lease.command_stamp.to_sec()<=.3)
                    stamp=min(lease.header.stamp.to_nsec(),lease.command_stamp.to_nsec())//1000 if lease else 0
                    fields=[self.plan.task,source[0],source[1],t.world_epoch,1,*measured,
                        self.plan.plan,self.plan.target,self.plan.scene,stamp,int(fresh),int(valid),
                        *map(float,base),clearance,*map(float,lift),t.fixed_grasp_clearance]
                    import rospy
                    before_wall=time.monotonic();before_sim=rospy.Time.now().to_sec()
                    self.client('arm_control','fixed_grasp_context',*fields)
                    after_sim=rospy.Time.now().to_sec()
                    timing=dict(source_age_before_s=before_sim-source[1]*1e-6,
                        source_age_after_s=after_sim-source[1]*1e-6,
                        compute_wall_s=before_wall-iteration_wall,
                        command_wall_s=time.monotonic()-before_wall,
                        input_ages_s=row['ages'])
                # Dense physical truth during insertion; native snapshots keep
                # their original timestamp/fresh flag, never get restamped.
                if not polled and not timing:continue
                arm=self.arm
                state=self.native;phase=int(state.get('phase',0))
                record=dict(sim_s=now,stage=self.stage,phase=phase,native=state,hover=self.latest,arm=arm,
                    context_timing=timing,
                    physical_motion=dict(body_speed=measured[0],body_angular_speed=measured[1],
                        hover_position_error=measured[3]),unfold_stability_ready=unfold_ready,
                    native_status_fresh=polled,
                    compensation_mode=('BASE_HOLD_FOR_UNFOLD_SETTLE' if base_hold else
                        'BASE_HOLD_DURING_GRASP' if 4<=phase<=7 else 'WORLD_TCP'),
                    unfold_hover_settled=self.unfold_settled,
                    waypoint_index=self.route_index,
                    planned_waypoint_world=desired_world.tolist(),world_progress_error_m=float(np.linalg.norm(desired_world-goal_world)),
                    feedback=feedback,object_world=O,object_local=local.tolist(),body_world=row['body'],
                    joint_position_truth=row['joints'].tolist(),gripper_pose_world=g,arm_pose_world=row['arm'],
                    alignment_error=measured[4],relative_speed=measured[5],orientation_error=measured[6],
                    geometry_valid=valid,clearance=clearance,
                    compensation_speed_m_s=required_compensation,precision_motion_ready=precision_ready,
                    full_stroke_clearance=t.fixed_grasp_clearance,
                    open_corridor_gap_m=corridor_gap,insertion_ready=insertion_ready,
                    geometry_stage='fine' if self.unfold_settled else 'unfold_all_pairs_16mm',
                    forbidden_contact=t.fixed_grasp_forbidden_contact,
                    contact_frames=t.fixed_grasp_contact_frames,bilateral=t.fixed_grasp_bilateral_fraction,
                    support=t.fixed_grasp_support_fraction,grasp_goal_world=goal_world.tolist(),
                    trim_world=self.correction.tolist(),fk_position_error=pos,fk_rotation_error=angle,
                    feedback_frame_error=frame_error,feedback_pair_skew_s=pair_skew,
                    feedback_verified_age_s=now-verified_stamp if frame_error is not None else None,
                    raw_geometry_valid=t.fixed_grasp_geometry_valid,frame_verified=frame_verified)
                record.update(paired_feedback_sequence=paired_sequence,
                    paired_sample_time_us=verified_stamp*1e6 if frame_error is not None else None)
                self.samples.append(record);self.log.write(json.dumps(record,allow_nan=False)+'\n');self.log.flush()
                if not polled:continue
                if phase in (8,9):raise RuntimeError('Native fixed grasp exited: '+str(state))
                if self.stage=='waiting_hover' and self.streaming and self.latest.get('start_ready'):
                    self.expected=self.latest.get('last_request_id',0)
                    self.client('arm_control','hover_grasp_prepare',self.plan.task);self.stage='prepare'
                elif self.stage=='prepare' and self.latest.get('last_request_id',0)>self.expected:
                    if not self.latest.get('prepared'):self.stage='waiting_hover';continue
                    self.expected=state.get('last_request_id',0)
                    self.request('fixed_grasp_start');self.stage='start_ack'
                elif self.stage=='start_ack' and state.get('last_request_id',0)>self.expected:
                    if state.get('request_result')!=1:self.stage='waiting_hover';continue
                    self.motion_started=True;self.stage='approach';self.event('fixed_native_approach_started',row)
                elif state.get('paused') and self.latest.get('start_ready'):
                    self.request('fixed_grasp_resume')
                elif (self.stage=='approach' and final_waypoint and close_motion_ready and phase==1 and
                        state.get('unfold_complete') and state.get('close_entry_ready') and
                        measured[4]<=.003 and measured[5]<=.010 and measured[6]<=math.radians(5)):
                    # Use the live cube center, not the nominal endpoint or
                    # the lagging measured tool pose. Native slew/IK still
                    # bounds the <=3mm correction before actual closure.
                    self.progress_world=centered_world_target(tool,G[:3,:3],local);self.last_progress_sim=now
                    self.expected=state.get('last_request_id',0)
                    self.pickup_height=O[2];self.request('fixed_grasp_close');self.close_requested=True;self.stage='close_ack'
                    self.event('controlled_close_requested',row,alignment=measured[4],orientation=measured[6])
                elif self.stage=='close_ack' and state.get('last_request_id',0)>self.expected:
                    self.stage='grasp' if state.get('request_result')==1 else 'approach'
                if phase==5 and self.grasp_reference is None:
                    self.grasp_reference=local.copy();self.event('physical_lift_started',row)
                if phase==7:
                    good=(self.pickup_height is not None and O[2]-self.pickup_height>=.025 and
                        t.fixed_grasp_bilateral_fraction>=.9 and t.fixed_grasp_support_fraction<=.001 and
                        valid and self.grasp_reference is not None and np.linalg.norm(local-self.grasp_reference)<=.045)
                    if not good:self.done_since=None
                    elif self.done_since is None:self.done_since=now
                    elif now-self.done_since>=5:
                        self.complete=True;self.stage='done';self.event('fixed_object_lift_held',row,
                            actual_lift=O[2]-self.pickup_height);return
        except Exception as exc:self.error=str(exc)
        finally:self.log.close()

    def start(self):self.feedback_thread.start();self.status_thread.start();self.thread.start()

    def finish(self):
        self.stop_event.set();self.thread.join(timeout=8);self.feedback_thread.join(timeout=8);self.status_thread.join(timeout=8)
        if any(t.is_alive() for t in (self.thread,self.feedback_thread,self.status_thread)):
            raise RuntimeError('Fixed grasp observer did not stop')
        if self.error:raise RuntimeError(self.error)
        held=[r for r in self.samples if self.done_since is not None and r['sim_s']>=self.done_since]
        frames=sum(r['contact_frames'] for r in held)
        bilateral=sum(r['contact_frames']*r['bilateral'] for r in held)/max(1,frames)
        support=sum(r['contact_frames']*r['support'] for r in held)/max(1,frames)
        lift=min((r['object_world'][2]-self.pickup_height for r in held),default=-1)
        physical=bool(self.complete and frames and bilateral>=.9 and support<=.001 and lift>=.025 and
            all(r['geometry_valid'] and not r['forbidden_contact'] for r in held))
        return dict(grasp_validated=physical,events=self.events,binding=self.plan.ids,
            independent_contact_frames=frames,independent_bilateral_fraction=bilateral,
            independent_support_fraction=support,minimum_held_lift_m=lift,
            observed_hold_seconds=held[-1]['sim_s']-held[0]['sim_s'] if held else 0,
            final_native_status=self.native,grasp_requests=self.approach_request_count,
            close_requested=self.close_requested,vehicle_fixed=False,object_attached=False,
            general_pose_or_carry_flight_tested=False)
