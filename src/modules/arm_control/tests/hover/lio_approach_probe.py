"""LIO/FK adapter for existing native approach/grasp; no joint actuation."""
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
from sim_scene import LIO_CLOCK_HZ


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
    """Velocity feedforward bias, capped at 50mm/s and slewed <=200mm/s²."""
    return _limited_difference(previous,finite(ekf_velocity,(3,))-finite(lio_velocity,(3,)),elapsed,.05,.20)[0]


def filter_tracking_errors(position,velocity,goal,position_bias,velocity_bias):
    """Compare the filter to its compensated references, not raw LIO values."""
    return (float(np.linalg.norm(finite(position,(3,))-finite(goal,(3,))-finite(position_bias,(3,)))),
        float(np.linalg.norm(finite(velocity,(3,))-finite(velocity_bias,(3,)))))


def preview_pose_ready(preview,desired):
    from grasp_audit.geometry import pose_transform
    if not preview.get('ik_success'):return False
    actual=pose_transform([*preview['fk'],*preview['quat']])
    return angle_deg(actual[:3,:3],finite(desired,(4,4))[:3,:3])<=5.


def transit_ready(index,phase,error,speed,hover_ready):
    # Keep the final native dwell. The far alignment is only an intermediate
    # waypoint: its instantaneous criteria need no second one-second dwell.
    return phase==2 or (index==1 and phase==1 and hover_ready and
        math.isfinite(error) and 0<=error<=.005 and math.isfinite(speed) and 0<=speed<=.010)


def cancellation_ack(state,arm,ids,jaw,after_request,*,grasp=False):
    return (state.get('mode')==(1 if grasp else 0) and state.get('phase')==(9 if grasp else 5) and
        [state.get(k) for k in ('task_id','plan_id','target_id','scene_id')]==ids and
        state.get('request_result')==1 and state.get('last_request_id',0)>after_request and
        state.get('close_authorized') is False and abs(arm['joint_target'][5]-jaw)<1e-6)


def approach_geometry(row,config,geometry,static_boxes,points,*,precision=False,jaw=None,boxes_override=None):
    from fixed_demo_plan import FixedDemoPlan,PAD_NAMES
    from perception.check_installation import transform
    body=finite(row['map_from_body'],(4,4));arm=finite(row['map_from_arm'],(4,4))
    q=finite(row['joint_position_rad'],(6,)).copy();points=finite(points)
    if jaw is not None:q[5]=float(jaw)
    if not static_boxes or points.ndim!=2 or points.shape[1]!=3 or not len(points):raise ValueError('APPROACH_GEOMETRY_COVERAGE')
    boxes=list(FixedDemoPlan.boxes(SimpleNamespace(arm=arm),q[:5],float(q[5]))) if boxes_override is None else list(boxes_override)
    target=finite(row['map_from_target'],(4,4));cfg=geometry['target']
    obstacles=[(p['name'],target@transform(p['xyz_rpy_deg']),np.array(p['size_m'])*.5) for p in cfg['parts']]
    obstacles.append(('support',target@transform(cfg['object_from_support_xyz_rpy_deg']),np.array(cfg['support_size_m'])*.5))
    gaps=box_gaps(boxes,obstacles);effective=gaps.copy()
    # Express the approved 3mm finger/support condition in the existing 16mm
    # native budget. Only the designated pads/handle pair is contact-exempt.
    exemptions=[]
    if precision:
        for i,(name,_,_) in enumerate(boxes):
            if name not in PAD_NAMES:continue
            for j,(part,_,_) in enumerate(obstacles):
                if part=='handle':effective[i,j]=math.inf;exemptions.append([name,'handle'])
                elif part=='support':effective[i,j]+=.013
    local=float(np.min(effective))
    uncertainty=.004+row['prediction_translation_m']+.51*row['prediction_rotation_rad']
    point_handle=np.zeros(len(points),dtype=bool);point_support=point_handle.copy()
    if precision:
        for name,T,h in obstacles:
            inside=np.all(np.abs((points-T[:3,3])@T[:3,:3])<=h+.004,axis=1)
            if name=='handle':point_handle|=inside
            elif name=='support':point_support|=inside
    observed=.5
    for name,T,h in boxes:
        p=(points-T[:3,3])@T[:3,:3]
        distances=np.linalg.norm(np.maximum(np.abs(p)-h,0),axis=1)
        if precision and name in PAD_NAMES:
            distances[point_support]+=.013
            distances[point_handle]=math.inf
        observed=min(observed,float(np.min(distances)))
    self_gap=float(np.min(box_gaps(boxes,[(n,body@B,h) for n,B,h in static_boxes])))
    external=min(local-uncertainty,observed-.004-uncertainty)
    clearance=min(external,self_gap)
    return dict(valid=bool(clearance>.016),clearance_m=clearance,
        target_support_lower_bound_m=local,observed_return_gap_m=observed,
        arm_body_lower_bound_m=self_gap,external_clearance_m=external,prediction_localization_margin_m=uncertainty,
        required_m=.016,pad_support_required_m=.003 if precision else .016,
        contact_exemptions=exemptions,scope='TARGET_SUPPORT_OBSERVED_RETURNS_AND_ARM_VEHICLE',
        target_support_policy='LOCKED_LIO_TARGET_WITH_KNOWN_FIXTURE',unknown_space_certified=False)


def jaw_sweep_geometry(row,config,geometry,static_boxes,points,*,precision=False):
    from fixed_demo_plan import FixedDemoPlan,joints,links
    from known_target import mount_transform
    q=finite(row['joint_position_rad'],(6,));arm=finite(row['map_from_arm'],(4,4))
    all_boxes=list(FixedDemoPlan.boxes(SimpleNamespace(arm=arm),q[:5],float(q[5])))
    fixed=[b for b in all_boxes if b[0]!='moving_jaw_collision']
    joint=next(j for j in joints if j.get('name')=='gripper')
    child=joint.findtext('child')
    collisions=links[child].findall('collision')
    if len(collisions)!=1 or collisions[0].get('name')!='moving_jaw_collision' or any(j.findtext('parent')==child for j in joints):
        raise ValueError('UNSUPPORTED_JAW_COLLISION_TREE')
    origin=mount_transform(np.fromstring(joint.findtext('pose','0 0 0 0 0 0'),sep=' '))
    axis=np.fromstring(joint.findtext('axis/xyz'),sep=' ');axis=axis/np.linalg.norm(axis)
    K=np.array([[0,-axis[2],axis[1]],[axis[2],0,-axis[0]],[-axis[1],axis[0],0]])
    collision=links[joint.findtext('child')].find("collision[@name='moving_jaw_collision']")
    local=mount_transform(np.fromstring(collision.findtext('pose','0 0 0 0 0 0'),sep=' '))
    half=np.fromstring(collision.findtext('geometry/box/size'),sep=' ')*.5
    # Recover the parent from the SAME production-SDF chain as the scalar
    # boxes, avoiding even the URDF/SDF sub-micrometre rounding difference.
    fixed_pose=next(T for name,T,_ in fixed if name=='gripper_fixed_finger_collision')
    fixed_collision=links[joint.findtext('parent')].find("collision[@name='gripper_fixed_finger_collision']")
    fixed_local=mount_transform(np.fromstring(fixed_collision.findtext('pose','0 0 0 0 0 0'),sep=' '))
    parent=fixed_pose@inverse(fixed_local)@origin
    moving=[]
    for angle in np.linspace(.04,.3,53):
        rotate=np.eye(4);rotate[:3,:3]=np.eye(3)+math.sin(angle)*K+(1-math.cos(angle))*(K@K)
        moving.append(('moving_jaw_collision',parent@rotate@local,half))
    # Equivalent to 53 complete-scene evaluations, but all fixed links are
    # evaluated only once. Keep the same between-sample descendant bound.
    measured=approach_geometry(row,config,geometry,static_boxes,points,precision=precision,boxes_override=fixed+moving)
    return dict(external_clearance_m=measured['external_clearance_m']-.11*.005,
        arm_body_lower_bound_m=measured['arm_body_lower_bound_m']-.11*.005)


def context_values(row,goal_map,candidate,lease,ids,geometry,now,goal_velocity_map=None,*,alignment_target_map=None):
    if row['frame']!='map':raise ValueError('APPROACH_FRAME')
    body=np.array(row['map_from_body']);arm=np.array(row['map_from_arm']);grip=np.array(row['map_from_gripper'])
    goal=(inverse(arm)@np.r_[goal_map[:3,3],1])[:3]
    v=np.array(row['linear_velocity_map_m_s']);w=np.array(row['angular_velocity_map_rad_s'])
    goal_velocity=np.zeros(3) if goal_velocity_map is None else finite(goal_velocity_map,(3,))
    speed=max(np.linalg.norm(np.array(row['tcp_velocity_map_m_s'])-goal_velocity),
        np.linalg.norm(np.array(row['reference_velocity_map_m_s'])-goal_velocity))
    alignment_target=np.array(row['map_from_target'])[:3,3] if alignment_target_map is None else finite(alignment_target_map,(3,))
    metrics=[float(np.linalg.norm(v)),float(np.linalg.norm(w)),
        math.acos(float(np.clip(body[2,2],-1,1))),
        float(np.linalg.norm(body[:3,3]-candidate[:3,3])),
        float(np.linalg.norm(np.array(row['map_from_grasp_reference'])[:3,3]-alignment_target)),
        float(speed),math.radians(angle_deg(grip[:3,:3],np.array(row['map_from_target'])[:3,:3]))]
    matching=lease is not None and [int(lease.task_id),int(lease.plan_id),int(lease.target_id),int(lease.scene_id)]==ids
    lease_good=matching and lease.header.frame_id=='map' and lease.terminal_hold and lease.lease_valid and -.02<=now-lease.header.stamp.to_sec()<=.1 and -.02<=now-lease.command_stamp.to_sec()<=.3
    stamp=int(round(row['stamp_s']*1e6))
    lease_stamp=min(lease.header.stamp.to_nsec(),lease.command_stamp.to_nsec())//1000 if lease is not None else 0
    return [ids[0],stamp,stamp,1,1,*metrics,*ids[1:],lease_stamp,int(lease_good),int(geometry['valid']),*map(float,goal),geometry['clearance_m']]


class LioApproachProbe:
    def __init__(self,client,observer,config,geometry,candidate,seed,folder,latest,scene,points,*,grasp=False):
        from hover_approach_probe import token
        from lio_route_clearance import body_collision_boxes
        self.client=client;self.observer=observer;self.config=config;self.geometry=geometry
        self.grasp=bool(grasp);self.closing_requested=False
        self.candidate=candidate;self.latest=latest;self.seed=np.array(seed)
        self.folder=Path(folder);self.folder.mkdir()
        self.ids=[1,token([candidate['map_from_target'].tolist(),'lio_contact_grasp_v1' if self.grasp else 'lio_no_close_center_then_pre_v6']),
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
        # Keep the SAME first waypoint as the existing native unfold precheck.
        # A substituted Cartesian point can select a joint-limit IK branch.
        from fixed_demo_plan import ROUTE
        self.first_arm=np.eye(4);self.first_arm[:3,3]=ROUTE[0]
        first=candidate['map_from_arm']@self.first_arm
        pre=np.array(candidate['pre_goal']['map_from_desired_tcp'])
        # 70mm put shoulder_lift on its lower limit even at nominal hover.
        # Align at 55mm instead, then retain the unchanged 40mm endpoint.
        far=pre.copy();far[:3,3]+=np.array(candidate['map_from_target'])[:3,2]*.015
        self.goals=[first,far,pre]
        self.alignment_target=None
        if self.grasp:
            final=np.array(candidate['final_goal']['map_from_desired_tcp']).copy()
            # Reuse the tested 2.3mm fixed-pad seating correction. This is a
            # target derived from the observed object, not its simulator pose.
            final[:3,3]+=np.array(candidate['map_from_target'])[:3,0]*.0023
            self.goals.append(final)
            # Compare the measured OPEN reference to its one fixed seated
            # destination. Never use a moving far/pre waypoint as close target.
            self.alignment_target=np.array(candidate['map_from_target'])[:3,3]+np.array(candidate['map_from_target'])[:3,0]*.0023
        self.segment=None
        self.index=0;self.stage='prepare';self.expected=0;self.last_stamp=0.;self.last_status=0.;self.poll_index=0
        self.native={};self.hover={};self.arm={};self.error=None;self.complete=False;self.motion_requested=False
        self.stop_event=threading.Event();self.thread=threading.Thread(target=self.run,daemon=True)
        self.holding=None;self.last_preview=0.;self.preview_valid=False;self.resume_id=0
        self.events=[];self.last_record=None;self.cancel_after=0;self.prepare_since=None

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
        try:
            with (self.folder/'control.jsonl').open('w') as log, (self.folder/'timing.jsonl').open('w') as timing:
                while not self.stop_event.wait(.002):
                    now=rospy.Time.now().to_sec();lease=self.latest.get('known')
                    if not self.bound_lease(lease):continue
                    if self.prepare_since is None:self.prepare_since=now
                    if self.stage in ('prepare','prepare_ack','start_ack') and now-self.prepare_since>30:
                        raise RuntimeError('LIO grasp preparation did not pass within 30 simulation seconds')
                    sample_started=time.perf_counter()
                    row=sample_control(self.observer,now)
                    sample_wall=time.perf_counter()-sample_started
                    if row is None:
                        self.holding=None
                        continue
                    if row['stamp_s']==self.last_stamp:continue
                    self.last_stamp=row['stamp_s']
                    geometry_started=time.perf_counter()
                    # Precision starts at the 55mm alignment in grasp mode.
                    # Only the already approved pad/handle contact pair changes;
                    # NoClose and every other 16mm pair remain independent.
                    fine=self.grasp and self.index>=1
                    geo=approach_geometry(row,self.config,self.geometry,self.static,self.points,precision=fine)
                    sweep=jaw_sweep_geometry(row,self.config,self.geometry,self.static,self.points,precision=True) if fine else None
                    jaw_clearance=0. if self.grasp else None
                    geometry_wall=time.perf_counter()-geometry_started
                    compute_age=max(0.,rospy.Time.now().to_sec()-row['stamp_s'])+1./LIO_CLOCK_HZ
                    if compute_age>.1:
                        self.holding=None
                        continue
                    transport=(np.linalg.norm(row['linear_velocity_map_m_s'])+.51*np.linalg.norm(row['angular_velocity_map_rad_s']))*compute_age
                    geo['clearance_m']=min(geo['external_clearance_m']-float(transport),geo['arm_body_lower_bound_m'])
                    geo['valid']=geo['clearance_m']>.016
                    geo['computation_age_s']=compute_age;geo['computation_transport_reserve_m']=float(transport)
                    if sweep is not None:jaw_clearance=min(sweep['external_clearance_m']-float(transport),sweep['arm_body_lower_bound_m'])
                    goal,goal_velocity,segment_finished=self.goal_at(row,now)
                    fields=context_values(row,goal,self.candidate['map_from_body_hover'],lease,self.ids,geo,now,goal_velocity,
                        alignment_target_map=self.alignment_target)
                    if self.grasp:
                        lift=(inverse(np.array(row['map_from_arm']))@np.r_[goal[:3,3]+[0.,0.,.04],1])[:3]
                        fields.extend([*map(float,lift),jaw_clearance])
                    sent=rospy.Time.now().to_sec();ipc_started=time.perf_counter()
                    self.client.client('arm_control','fixed_grasp_context' if self.grasp else 'hover_task_context',*fields)
                    ack=rospy.Time.now().to_sec();publish_age=ack-row['stamp_s']
                    timing.write(json.dumps(dict(kind='context',sample_sim_s=now,stamp_s=row['stamp_s'],sample_wall_monotonic_s=sample_started,ack_wall_monotonic_s=time.perf_counter(),
                        lio_stamp_s=row['last_lio_stamp_s'],sample_wall_s=sample_wall,geometry_wall_s=geometry_wall,
                        sent_sim_s=sent,ack_sim_s=ack,prepare_wall_s=ipc_started-sample_started,ipc_wall_s=time.perf_counter()-ipc_started))+'\n');timing.flush()
                    if now-self.last_status<.02:continue
                    self.last_status=now
                    # Read one diagnostic topic between context publications;
                    # three blocking CLI reads must never form a heartbeat gap.
                    attr,topic=(('hover','arm_hover_status'),('native','arm_approach_status'),('arm','arm_control_status'))[self.poll_index]
                    began=rospy.Time.now().to_sec();ipc_started=time.perf_counter()
                    setattr(self,attr,self.client.status(topic));self.poll_index=(self.poll_index+1)%3
                    timing.write(json.dumps(dict(kind='diagnostic',topic=topic,start_sim_s=began,
                        end_sim_s=rospy.Time.now().to_sec(),ipc_wall_s=time.perf_counter()-ipc_started))+'\n');timing.flush()
                    if self.poll_index or not self.hover or not self.native or not self.arm:continue
                    tcp=np.array(row['map_from_tcp']);error=float(np.linalg.norm(tcp[:3,3]-goal[:3,3]))
                    angle=angle_deg(tcp[:3,:3],goal[:3,:3]) if self.index else None
                    record=dict(sim_s=now,stage=self.stage,index=self.index,geometry=geo,hover=self.hover,
                        approach=self.native,arm=self.arm,control_sample=row,goal_map=goal.tolist(),tool_error_m=error,orientation_error_deg=angle,
                        goal_policy='ARM_RELATIVE_TRANSIT' if self.index==0 else ('LIO_TARGET_QUINTIC_2MM_S' if self.grasp and self.index==3 else 'LIO_TARGET_QUINTIC_10MM_S'),segment_finished=segment_finished,
                        geometry_compute_wall_s=geometry_wall,context_publish_age_s=publish_age,
                        precision_contact_profile=fine,jaw_sweep_clearance_m=jaw_clearance,closing_requested=self.closing_requested)
                    from check_fixed_target_native import evidence_json
                    log.write(json.dumps(evidence_json(record),allow_nan=False)+'\n');log.flush();self.last_record=record
                    if (self.motion_requested and self.native.get('ownership_locked') and self.native.get('mode')!=(1 if self.grasp else 0)):
                        raise RuntimeError('Unexpected native motion owner')
                    if not self.closing_requested and (self.native.get('close_authorized') or abs(self.arm['joint_target'][5]-self.jaw)>1e-6):
                        raise RuntimeError('Open-jaw motion contract violated')
                    if self.native.get('mode')==(1 if self.grasp else 0) and self.native.get('phase') in ((8,9) if self.grasp else (4,5)):
                        raise RuntimeError('Native motion exit: '+str(self.native))
                    if self.stage=='prepare' and self.hover.get('start_ready') and geo['valid']:
                        if now-self.last_preview<.5:continue
                        self.last_preview=now
                        # Validate the actual body pose, not the nominal hover.
                        actual_pre=inverse(np.array(row['map_from_arm']))@self.goals[-1]
                        output=self.client.client('arm_control','ik_check',*map(float,actual_pre[:3,3]),*map(float,self.seed))
                        native=parse_native(output)
                        with (self.folder/'actual_ik_precheck.log').open('a') as out:out.write(output+'\n')
                        self.preview_valid=preview_pose_ready(native,actual_pre)
                        if not self.preview_valid:continue
                        self.expected=self.hover.get('last_request_id',0)
                        self.client.client('arm_control','hover_grasp_prepare',self.ids[0]);self.stage='prepare_ack'
                    elif self.stage=='prepare_ack' and self.hover.get('last_request_id',0)>self.expected:
                        if not self.hover.get('prepared'):self.stage='prepare';continue
                        self.expected=self.native.get('last_request_id',0)
                        self.client.client('arm_control','fixed_grasp_start' if self.grasp else 'hover_approach',*self.ids)
                        self.motion_requested=True;self.stage='start_ack'
                    elif self.stage=='start_ack' and self.native.get('last_request_id',0)>self.expected:
                        if self.native.get('request_result')!=1 or not self.native.get('ownership_locked'):
                            self.stage='prepare';continue
                        self.stage='approach';self.event('native_contact_grasp_started' if self.grasp else 'native_no_close_started',now)
                    elif self.stage=='close_ack' and self.native.get('last_request_id',0)>self.expected:
                        if self.native.get('request_result')!=1:
                            self.closing_requested=False;self.stage='approach'
                        else:self.stage='grasp'
                    elif self.stage=='grasp':
                        if self.native.get('paused') and self.hover.get('start_ready'):
                            self.client.client('arm_control','fixed_grasp_resume',*self.ids)
                        if self.native.get('phase')==7 and self.native.get('completed_success'):
                            self.complete=True;self.event('contact_hold_and_lio_fk_extraction_completed',now)
                            self.cancel_after=self.native.get('last_request_id',0)
                            self.client.client('arm_control','fixed_grasp_cancel',*self.ids);return
                    elif self.stage=='approach':
                        if (self.native.get('paused') if self.grasp else self.native.get('phase')==3):
                            self.holding=None
                            if (self.grasp or self.native.get('resume_ready')) and self.hover.get('start_ready'):
                                request_id=self.native.get('last_request_id',0)
                                if request_id!=self.resume_id:
                                    self.resume_id=request_id;self.client.client('arm_control','fixed_grasp_resume' if self.grasp else 'hover_resume',*self.ids)
                                    self.event('same_task_resume_requested',now)
                        elif segment_finished and self.native.get('unfold_complete') and (
                                (self.grasp and self.native.get('phase')==1 and self.hover.get('start_ready') and fields[10]<=.010)
                                or (not self.grasp and transit_ready(self.index,self.native.get('phase'),error,fields[10],self.hover.get('start_ready')))) and error<=.015:
                            if self.index<len(self.goals)-1:
                                if self.index>=1:
                                    if self.index==2 and (error>.005 or angle>5):continue
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
                                if not preview_pose_ready(preview,actual_pre):continue
                                event=('existing_transit_reached','far_alignment_reached','preapproach_reached')[self.index]
                                self.index+=1;self.holding=None
                                end=self.goals[self.index][:3,3]
                                # Smooth only forward approach; native retains
                                # its 30mm/s budget for body-motion compensation.
                                duration=max(4.,1.875*float(np.linalg.norm(end-tcp[:3,3]))/(.002 if self.grasp and self.index==3 else .010))
                                self.segment=(now,tcp[:3,3].copy(),duration)
                                self.event(event,now,error_m=error,transition_duration_s=duration)
                            elif self.grasp:
                                if error<=.003 and angle<=5 and fields[10]<=.010 and self.native.get('close_entry_ready'):
                                    self.expected=self.native.get('last_request_id',0)
                                    self.closing_requested=True
                                    self.client.client('arm_control','fixed_grasp_close',*self.ids)
                                    self.stage='close_ack';self.event('lio_controlled_close_requested',now,error_m=error,orientation_deg=angle)
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
        grasp=getattr(self,'grasp',False)
        jaw=self.jaw
        if self.motion_requested and not self.complete:
            self.cancel_after=self.native.get('last_request_id',0)
            self.client.client('arm_control','fixed_grasp_cancel' if grasp else 'hover_cancel',*self.ids)
        canceled=None
        if self.motion_requested:
            deadline=time.monotonic()+2.
            while time.monotonic()<deadline:
                state=self.client.status('arm_approach_status')
                arm=self.client.status('arm_control_status')
                expected_jaw=state.get('jaw_target',float('nan')) if grasp else jaw
                before=self.arm['joint_target'][5] if grasp else jaw
                retained=(not grasp or (math.isfinite(expected_jaw) and .039999<=expected_jaw<=before+1e-6))
                # If start was rejected, there is no owned motion to cancel.
                # Still require the fresh response to THIS cancel and unchanged
                # targets, so a missing acknowledgment cannot masquerade as idle.
                if (grasp and state.get('mode')==1 and state.get('phase')==0 and
                        state.get('ownership_locked') is False and state.get('request_result')==3 and
                        state.get('last_request_id',0)>self.cancel_after and
                        np.max(np.abs(np.array(arm['joint_target'])-self.baseline['joint_target']))<1e-6):
                    canceled=dict(acknowledged=True,status=state,motion_not_acquired=True,
                        jaw_held_rad=arm['joint_target'][5],jaw_unchanged=True,jaw_bias_retained=True);break
                if retained and cancellation_ack(state,arm,self.ids,expected_jaw,self.cancel_after,grasp=grasp):
                    if grasp:
                        time.sleep(.02)
                        second=self.client.status('arm_approach_status')
                        second_arm=self.client.status('arm_control_status')
                        if second.get('timestamp',0)<=state.get('timestamp',0) or not cancellation_ack(
                                second,second_arm,self.ids,expected_jaw,self.cancel_after,grasp=True):continue
                    canceled=dict(acknowledged=True,status=state,jaw_held_rad=expected_jaw,
                        jaw_unchanged=not grasp,jaw_bias_retained=retained);break
                time.sleep(.02)
            if canceled is None:raise RuntimeError('No-close cancellation not acknowledged')
        result=dict(completed=self.complete and self.error is None,error=self.error,events=self.events,binding=self.ids,
            motion_requested=self.motion_requested,closing_requested=getattr(self,'closing_requested',False),truth_control_inputs=False,
            object_pose_after_grasp_observed=False,confirmation='CONTACT_SUPPORT_RELEASE_AND_LIO_FK_TOOL_EXTRACTION' if grasp else 'LIO_FK_PREAPPROACH',
            final_record=self.last_record,cancellation=canceled)
        from check_fixed_target_native import evidence_json
        result=evidence_json(result)
        (self.folder/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        return result
