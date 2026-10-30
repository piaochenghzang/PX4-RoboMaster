"""LIO/FK adapter for the existing no-close native approach, not a grasp FSM."""
from pathlib import Path
from types import SimpleNamespace
import json
import math
import threading
import time
import numpy as np
from lio_tool_observer import finite,inverse
from lio_control_sample import sample_control
from candidate_hover import angle_deg
from lio_box_gaps import box_gaps


def _limited_difference(previous,delta,elapsed,limit,rate):
    previous=finite(previous,(3,));delta=finite(delta,(3,)).copy()
    if not math.isfinite(elapsed) or elapsed<=0:raise ValueError('ALIGNMENT_TIME')
    length=float(np.linalg.norm(delta));delta*=min(1.,limit/max(length,1e-12))
    change=delta-previous;change*=min(1.,rate*min(elapsed,.1)/max(float(np.linalg.norm(change)),1e-12))
    return previous+change,change/elapsed


def alignment_bias(previous,ekf_position,lio_position,elapsed):
    """Bounded LIO-to-filter command offset; never moves either map origin."""
    return _limited_difference(previous,finite(ekf_position,(3,))-finite(lio_position,(3,)),elapsed,.06,.02)


def alignment_velocity(previous,ekf_velocity,lio_velocity,elapsed):
    """Velocity feedforward bias, capped at 50mm/s and slewed <=50mm/s²."""
    return _limited_difference(previous,finite(ekf_velocity,(3,))-finite(lio_velocity,(3,)),elapsed,.05,.05)[0]


def filter_tracking_errors(position,velocity,goal,position_bias,velocity_bias):
    """Compare the filter to its compensated references, not raw LIO values."""
    return (float(np.linalg.norm(finite(position,(3,))-finite(goal,(3,))-finite(position_bias,(3,)))),
        float(np.linalg.norm(finite(velocity,(3,))-finite(velocity_bias,(3,)))))


def transit_ready(index,phase,error,speed,hover_ready):
    # Keep the final native dwell. The far alignment is only an intermediate
    # waypoint: its instantaneous criteria need no second one-second dwell.
    return phase==2 or (index==1 and phase==1 and hover_ready and
        math.isfinite(error) and 0<=error<=.005 and math.isfinite(speed) and 0<=speed<=.010)


def cancellation_ack(state,arm,ids,jaw,after_request):
    return (state.get('mode')==0 and state.get('phase')==5 and
        [state.get(k) for k in ('task_id','plan_id','target_id','scene_id')]==ids and
        state.get('request_result')==1 and state.get('last_request_id',0)>after_request and
        state.get('close_authorized') is False and abs(arm['joint_target'][5]-jaw)<1e-6)


def approach_geometry(row,config,geometry,static_boxes,points):
    from fixed_demo_plan import FixedDemoPlan
    from perception.check_installation import transform
    body=finite(row['map_from_body'],(4,4));arm=finite(row['map_from_arm'],(4,4))
    q=finite(row['joint_position_rad'],(6,));points=finite(points)
    if not static_boxes or points.ndim!=2 or points.shape[1]!=3 or not len(points):raise ValueError('APPROACH_GEOMETRY_COVERAGE')
    boxes=list(FixedDemoPlan.boxes(SimpleNamespace(arm=arm),q[:5],float(q[5])))
    target=finite(row['map_from_target'],(4,4));cfg=geometry['target']
    obstacles=[(p['name'],target@transform(p['xyz_rpy_deg']),np.array(p['size_m'])*.5) for p in cfg['parts']]
    obstacles.append(('support',target@transform(cfg['object_from_support_xyz_rpy_deg']),np.array(cfg['support_size_m'])*.5))
    local=float(np.min(box_gaps(boxes,obstacles)))
    # Original LIO stamp remains explicit. Reserve the entire predicted rigid
    # displacement plus 4mm localization allowance; it is not a measurement.
    uncertainty=.004+row['prediction_translation_m']+.51*row['prediction_rotation_rad']
    observed=.5
    for _,T,h in boxes:
        p=(points-T[:3,3])@T[:3,:3]
        if len(p):observed=min(observed,float(np.min(np.linalg.norm(np.maximum(np.abs(p)-h,0),axis=1))))
    self_gap=float(np.min(box_gaps(boxes,[(n,body@B,h) for n,B,h in static_boxes])))
    # Self geometry is body-relative; common LIO pose error cancels. Native
    # swept/pending joint budgets still reserve motion from the measured q.
    clearance=min(local-uncertainty,observed-.004-uncertainty,self_gap)
    return dict(valid=bool(clearance>.016),clearance_m=clearance,
        target_support_lower_bound_m=local,observed_return_gap_m=observed,
        arm_body_lower_bound_m=self_gap,prediction_localization_margin_m=uncertainty,
        required_m=.016,contact_exemptions=[],scope='TARGET_SUPPORT_OBSERVED_RETURNS_AND_ARM_VEHICLE',
        unknown_space_certified=False)


def context_values(row,goal_map,candidate,lease,ids,geometry,now,goal_velocity_map=None):
    if row['frame']!='map':raise ValueError('APPROACH_FRAME')
    body=np.array(row['map_from_body']);arm=np.array(row['map_from_arm']);grip=np.array(row['map_from_gripper'])
    goal=(inverse(arm)@np.r_[goal_map[:3,3],1])[:3]
    v=np.array(row['linear_velocity_map_m_s']);w=np.array(row['angular_velocity_map_rad_s'])
    goal_velocity=np.zeros(3) if goal_velocity_map is None else finite(goal_velocity_map,(3,))
    speed=max(np.linalg.norm(np.array(row['tcp_velocity_map_m_s'])-goal_velocity),
        np.linalg.norm(np.array(row['reference_velocity_map_m_s'])-goal_velocity))
    metrics=[float(np.linalg.norm(v)),float(np.linalg.norm(w)),
        math.acos(float(np.clip(body[2,2],-1,1))),
        float(np.linalg.norm(body[:3,3]-candidate[:3,3])),
        float(np.linalg.norm(np.array(row['map_from_grasp_reference'])[:3,3]-np.array(row['map_from_target'])[:3,3])),
        float(speed),math.radians(angle_deg(grip[:3,:3],np.array(row['map_from_target'])[:3,:3]))]
    matching=lease is not None and [int(lease.task_id),int(lease.plan_id),int(lease.target_id),int(lease.scene_id)]==ids
    lease_good=matching and lease.header.frame_id=='map' and lease.terminal_hold and lease.lease_valid and -.02<=now-lease.header.stamp.to_sec()<=.1 and -.02<=now-lease.command_stamp.to_sec()<=.3
    stamp=int(round(row['stamp_s']*1e6))
    lease_stamp=min(lease.header.stamp.to_nsec(),lease.command_stamp.to_nsec())//1000 if lease is not None else 0
    return [ids[0],stamp,stamp,1,1,*metrics,*ids[1:],lease_stamp,int(lease_good),int(geometry['valid']),*map(float,goal),geometry['clearance_m']]


class LioApproachProbe:
    def __init__(self,client,observer,config,geometry,candidate,seed,folder,latest,scene,points):
        from hover_approach_probe import token
        from lio_route_clearance import body_collision_boxes
        self.client=client;self.observer=observer;self.config=config;self.geometry=geometry
        self.candidate=candidate;self.latest=latest;self.seed=np.array(seed)
        self.folder=Path(folder);self.folder.mkdir()
        self.ids=[1,token([candidate['map_from_target'].tolist(),'lio_no_close_center_then_pre_v6']),
            token(candidate['map_from_target'].tolist()),token(str(self.folder))]
        p=np.array(points,dtype=float)
        if p.ndim!=2 or p.shape[1]!=3 or not len(p) or not np.isfinite(p).all():raise ValueError('APPROACH_MAP_RETURNS')
        center=candidate['map_from_body_hover'][:3,3]
        self.points=p[np.linalg.norm(p-center,axis=1)<.9]
        # Registration is downsampled; point count alone is not a coverage
        # certificate. Require observed returns, retain the independently
        # validated target fit/shape, and never mark unknown space free.
        if not len(self.points):raise ValueError('APPROACH_MAP_COVERAGE')
        self.baseline=client.status('arm_control_status');self.jaw=self.baseline['joint_target'][5]
        self.static=[b for b in body_collision_boxes(scene,config,self.baseline['joint_target']) if '::' in b[0]]
        # A higher-clearance transit on the same unfolded IK branch. The old
        # 26.8mm transit margin was too small once LIO uncertainty was reserved.
        self.first_arm=np.eye(4);self.first_arm[:3,3]=[.15,.015,.325]
        first=candidate['map_from_arm']@self.first_arm
        pre=np.array(candidate['pre_goal']['map_from_desired_tcp'])
        # 70mm put shoulder_lift on its lower limit even at nominal hover.
        # Align at 55mm instead, then retain the unchanged 40mm endpoint.
        far=pre.copy();far[:3,3]+=np.array(candidate['map_from_target'])[:3,2]*.015
        self.goals=[first,far,pre]
        self.segment=None
        self.index=0;self.stage='prepare';self.expected=0;self.last_stamp=0.;self.last_status=0.;self.poll_index=0
        self.native={};self.hover={};self.arm={};self.error=None;self.complete=False;self.motion_requested=False
        self.stop_event=threading.Event();self.thread=threading.Thread(target=self.run,daemon=True)
        self.holding=None;self.last_preview=0.;self.preview_valid=False;self.resume_id=0
        self.events=[];self.last_record=None;self.cancel_after=0

    def event(self,name,now,**extra):
        self.events.append(dict(event=name,sim_s=now,**extra));print('LIO APPROACH: '+name,flush=True)

    def bound_lease(self,lease):
        return lease is not None and [int(lease.task_id),int(lease.plan_id),int(lease.target_id),int(lease.scene_id)]==self.ids and lease.terminal_hold

    def goal_at(self,row,now):
        from known_target import quintic_segment
        goal=self.goals[self.index].copy();velocity=None;finished=True
        if self.index==0:
            goal=np.array(row['map_from_arm'])@self.first_arm
            offset=goal[:3,3]-np.array(row['map_from_body'])[:3,3]
            velocity=np.array(row['linear_velocity_map_m_s'])+np.cross(row['angular_velocity_map_rad_s'],offset)
        elif self.segment is not None:
            began,start,duration=self.segment
            if not math.isfinite(now) or now<began:raise ValueError('APPROACH_SEGMENT_TIME')
            goal[:3,3],velocity=quintic_segment(start,goal[:3,3],now-began,duration)
            finished=now-began>=duration
        return goal,velocity,finished

    def run(self):
        import rospy
        from fixed_grasp_target import parse_native
        from grasp_audit.geometry import pose_transform
        try:
            with (self.folder/'control.jsonl').open('w') as log:
                while not self.stop_event.wait(.002):
                    now=rospy.Time.now().to_sec();lease=self.latest.get('known')
                    if not self.bound_lease(lease):continue
                    row=sample_control(self.observer,now)
                    if row is None or row['stamp_s']==self.last_stamp:continue
                    self.last_stamp=row['stamp_s']
                    geometry_started=time.perf_counter()
                    geo=approach_geometry(row,self.config,self.geometry,self.static,self.points)
                    geometry_wall=time.perf_counter()-geometry_started
                    goal,goal_velocity,segment_finished=self.goal_at(row,now)
                    fields=context_values(row,goal,self.candidate['map_from_body_hover'],lease,self.ids,geo,now,goal_velocity)
                    self.client.client('arm_control','hover_task_context',*fields)
                    publish_age=rospy.Time.now().to_sec()-row['stamp_s']
                    if now-self.last_status<.02:continue
                    self.last_status=now
                    # Read one diagnostic topic between context publications;
                    # three blocking CLI reads must never form a heartbeat gap.
                    attr,topic=(('hover','arm_hover_status'),('native','arm_approach_status'),('arm','arm_control_status'))[self.poll_index]
                    setattr(self,attr,self.client.status(topic));self.poll_index=(self.poll_index+1)%3
                    if self.poll_index or not self.hover or not self.native or not self.arm:continue
                    tcp=np.array(row['map_from_tcp']);error=float(np.linalg.norm(tcp[:3,3]-goal[:3,3]))
                    angle=angle_deg(tcp[:3,:3],goal[:3,:3]) if self.index else None
                    record=dict(sim_s=now,stage=self.stage,index=self.index,geometry=geo,hover=self.hover,
                        approach=self.native,arm=self.arm,control_sample=row,goal_map=goal.tolist(),tool_error_m=error,orientation_error_deg=angle,
                        goal_policy='ARM_RELATIVE_TRANSIT' if self.index==0 else 'LIO_TARGET_QUINTIC_10MM_S',segment_finished=segment_finished,
                        geometry_compute_wall_s=geometry_wall,context_publish_age_s=publish_age)
                    log.write(json.dumps(record,allow_nan=False)+'\n');log.flush();self.last_record=record
                    if self.native.get('mode')!=0 or self.native.get('close_authorized') or abs(self.arm['joint_target'][5]-self.jaw)>1e-6:
                        raise RuntimeError('No-close arm ownership/jaw contract violated')
                    if self.native.get('phase') in (4,5):raise RuntimeError('Native no-close exit: '+str(self.native))
                    if self.stage=='prepare' and self.hover.get('start_ready') and geo['valid']:
                        if now-self.last_preview<.5:continue
                        self.last_preview=now
                        # Validate the actual body pose, not the nominal hover.
                        actual_pre=inverse(np.array(row['map_from_arm']))@self.goals[-1]
                        output=self.client.client('arm_control','ik_check',*map(float,actual_pre[:3,3]),*map(float,self.seed))
                        native=parse_native(output)
                        with (self.folder/'actual_ik_precheck.log').open('a') as out:out.write(output+'\n')
                        self.preview_valid=bool(native.get('ik_success'))
                        if self.preview_valid:
                            T=pose_transform([*native['fk'],*native['quat']])
                            self.preview_valid=angle_deg(T[:3,:3],actual_pre[:3,:3])<=5.
                        if not self.preview_valid:continue
                        self.expected=self.hover.get('last_request_id',0)
                        self.client.client('arm_control','hover_grasp_prepare',self.ids[0]);self.stage='prepare_ack'
                    elif self.stage=='prepare_ack' and self.hover.get('last_request_id',0)>self.expected:
                        if not self.hover.get('prepared'):self.stage='prepare';continue
                        self.expected=self.native.get('last_request_id',0)
                        self.client.client('arm_control','hover_approach',*self.ids)
                        self.motion_requested=True;self.stage='start_ack'
                    elif self.stage=='start_ack' and self.native.get('last_request_id',0)>self.expected:
                        if self.native.get('request_result')!=1 or not self.native.get('ownership_locked'):
                            self.stage='prepare';continue
                        self.stage='approach';self.event('native_no_close_started',now)
                    elif self.stage=='approach':
                        if self.native.get('phase')==3:
                            self.holding=None
                            if self.native.get('resume_ready') and self.hover.get('start_ready'):
                                request_id=self.native.get('last_request_id',0)
                                if request_id!=self.resume_id:
                                    self.resume_id=request_id;self.client.client('arm_control','hover_resume',*self.ids)
                                    self.event('same_task_resume_requested',now)
                        elif segment_finished and self.native.get('unfold_complete') and transit_ready(
                                self.index,self.native.get('phase'),error,fields[10],self.hover.get('start_ready')) and error<=.015:
                            if self.index<2:
                                if self.index==1:
                                    lateral=np.array(row['map_from_target'])[:3,:3].T@(tcp[:3,3]-goal[:3,3])
                                    if error>.005 or np.linalg.norm(lateral[:2])>.003:continue
                                # Recheck from the actual unfolded/transit seed;
                                # the pre-start check used a different posture
                                # and an earlier body pose. Refusal holds here.
                                if now-self.last_preview<.5:continue
                                self.last_preview=now
                                actual_pre=inverse(np.array(row['map_from_arm']))@self.goals[self.index+1]
                                output=self.client.client('arm_control','ik_check',*map(float,actual_pre[:3,3]),
                                    *map(float,self.arm['joint_target'][:5]))
                                with (self.folder/'transit_ik_precheck.log').open('a') as out:out.write(output+'\n')
                                preview=parse_native(output)
                                if not preview.get('ik_success'):continue
                                checked=pose_transform([*preview['fk'],*preview['quat']])
                                if self.index==1 and angle_deg(checked[:3,:3],actual_pre[:3,:3])>5:continue
                                event='existing_transit_reached' if self.index==0 else 'far_alignment_reached'
                                self.index+=1;self.holding=None
                                end=self.goals[self.index][:3,3]
                                # Smooth only forward approach; native retains
                                # its 30mm/s budget for body-motion compensation.
                                duration=max(4.,1.875*float(np.linalg.norm(end-tcp[:3,3]))/.010)
                                self.segment=(now,tcp[:3,3].copy(),duration)
                                self.event(event,now,error_m=error,transition_duration_s=duration)
                            elif error<=.005 and angle<=5 and fields[10]<=.010:
                                if self.holding is None:self.holding=now
                                elif now-self.holding>=2:
                                    self.complete=True;self.event('lio_fk_preapproach_held',now,error_m=error,orientation_deg=angle)
                                    self.cancel_after=self.native.get('last_request_id',0)
                                    self.client.client('arm_control','hover_cancel',*self.ids)
                                    self.event('no_close_motion_canceled_after_validation',now);return
                            else:self.holding=None
                        else:self.holding=None
        except Exception as exc:self.error=str(exc)

    def start(self):self.thread.start()

    def finish(self):
        self.stop_event.set();self.thread.join(timeout=8)
        if self.thread.is_alive():raise RuntimeError('LIO approach worker did not stop')
        if self.motion_requested and not self.complete:
            self.cancel_after=self.native.get('last_request_id',0)
            self.client.client('arm_control','hover_cancel',*self.ids)
        canceled=None
        if self.motion_requested:
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline:
                state=self.client.status('arm_approach_status')
                arm=self.client.status('arm_control_status')
                if cancellation_ack(state,arm,self.ids,self.jaw,self.cancel_after):
                    canceled=dict(acknowledged=True,status=state,jaw_unchanged=True);break
                time.sleep(.02)
            if canceled is None:raise RuntimeError('No-close cancellation not acknowledged')
        result=dict(completed=self.complete and self.error is None,error=self.error,events=self.events,binding=self.ids,
            motion_requested=self.motion_requested,closing_requested=False,truth_control_inputs=False,
            final_record=self.last_record,cancellation=canceled)
        (self.folder/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        return result
