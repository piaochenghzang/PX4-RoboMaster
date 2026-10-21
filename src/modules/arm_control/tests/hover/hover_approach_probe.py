"""Bounded no-close task coordinator. Only native ArmControl may command joints."""
import hashlib
import json
import math
import threading
import re
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from known_target import pose, inverse
from hover_permit_probe import HoverPermitProbe, metrics, CUBE_REFERENCE


def validate_swept_bounds(root):
    """Triangle-chain bound for every descendant box corner, for ALL angles."""
    from pathlib import Path
    root=Path(root)
    model=ET.parse(root/'Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/so101/so101.sdf').find('model')
    links={e.get('name'):e for e in model.findall('link')}; joints=model.findall('joint')
    from known_target import mount_transform
    def radius(name):
        result=0.
        for c in links[name].findall('collision'):
            size=np.fromstring(c.findtext('geometry/box/size',''),sep=' ')
            if size.shape!=(3,) or not np.isfinite(size).all() or np.any(size<=0):
                raise RuntimeError('Unsupported collision proxy')
            if c.find('pose') is not None and c.find('pose').get('relative_to',name)!=name:
                raise RuntimeError('Unsupported collision frame')
            T=mount_transform(np.fromstring(c.findtext('pose','0 0 0 0 0 0'),sep=' '))
            for k in range(8):
                p=size*.5*np.array([1 if k&(1<<a) else -1 for a in range(3)])
                result=max(result,float(np.linalg.norm((T@np.r_[p,1])[:3])))
        for j in joints:
            if j.findtext('parent')!=name: continue
            p=np.fromstring(j.findtext('pose','0 0 0 0 0 0'),sep=' ')
            child=j.findtext('child'); cp=links[child].find('pose')
            if (p.shape!=(6,) or (j.find('pose') is not None and j.find('pose').get('relative_to',name)!=name)
                    or (cp is not None and (cp.get('relative_to')!=j.get('name') or
                        not np.allclose(np.fromstring(cp.text,sep=' '),0)))):
                raise RuntimeError('Unsupported child/joint frame')
            result=max(result,float(np.linalg.norm(p[:3]))+radius(child))
        return result
    names=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll']
    derived=[radius(next(j for j in joints if j.get('name')==n).findtext('child')) for n in names]
    text=(root/'src/modules/arm_control/grasp/HoverApproachAdapter.cpp').read_text()
    matches=re.findall(r'constexpr float radii\[ARM_DOF\]\{([^}]+)\}',text)
    variants=[[float(x.strip().rstrip('f')) for x in m.split(',')] for m in matches]
    compiled=variants[0] if variants else []
    if len(variants)!=2 or any(v!=compiled for v in variants):
        raise RuntimeError('Native command and resume swept radii differ')
    if len(compiled)!=5 or any(not math.isfinite(x) or x<r for x,r in zip(compiled,derived)):
        raise RuntimeError('Native swept radius does not bound production SDF')
    return dict(derived_m=derived,native_upper_bounds_m=compiled)


def token(value):
    return int(hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()[:12], 16) or 1


def pose_values(p):
    return [p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]


class ApproachPlan:
    def __init__(self, target, truth):
        # First unfold sideways, away from cube/support, then advance to an
        # offset approach point. This is NOT a jaw-aligned grasp pose. Offline
        # IK/production-box checks reject the old direct bench pregrasp route.
        self.waypoints_world=[(target.hover @ target.mount @ np.r_[p,1])[:3]
            for p in ([.10,-.10,.03],[.19,-.06,.10])]
        self.pre_world=self.waypoints_world[-1]
        self.plan = token([target.object.tolist(),target.hover.tolist(),
            [p.tolist() for p in self.waypoints_world],'no_close_offset_approach_v2'])
        self.scene = token([truth.source_instance,truth.world_epoch,truth.vehicle_id,truth.target_id])
        self.target = int(truth.target_id)
        self.task = token([self.plan,self.scene,self.target])
        self.ids = [self.task,self.plan,self.target,self.scene]

    def base_goal(self, row, index=-1):
        return (inverse(pose(row['arm'][:3],row['arm'][3:])) @ np.r_[self.waypoints_world[index],1])[:3]

    def matches(self, status):
        return status is not None and [status.task_id,status.plan_id,status.target_id,status.scene_id] == self.ids


class HoverApproachProbe(HoverPermitProbe):
    def __init__(self, build, env, sources, target, folder):
        import sys
        from pathlib import Path
        scripts=Path('/home/pcz/super_ws/src/super_px4_bridge/scripts')
        sys.path.insert(0,str(scripts))
        from grasp_audit.geometry import fk_gripper, residual
        self.fk_gripper,self.residual=fk_gripper,residual
        self.config=yaml.safe_load((scripts.parent/'config/grasp_frame_audit.yaml').read_text())
        self.swept_bounds=validate_swept_bounds(build.parents[1])
        self.build,self.env,self.sources,self.target=build,env,sources,target
        row=sources.read(); self.plan=ApproachPlan(target,row['truth'])
        self.log=(folder/'approach_status.jsonl').open('x')
        self.events=[]; self.complete=False; self.error=None; self.latest={}
        self.stage='waiting_hover'; self.streaming=False; self.motion_started=False
        self.approach_request_count=0
        self.stop_event=threading.Event(); self.thread=threading.Thread(target=self.run,daemon=True)
        self.last_source=None; self.last_status=-math.inf; self.expected_request=0; self.holding_since=None
        self.baseline=self.status('arm_control_status'); self.jaw=self.baseline['joint_target'][5]
        self.before_error=None; self.final_geometry=None
        self.route_index=0

    def physical(self, row):
        truth=row['truth']; names=list(truth.link_names)
        g=truth.link_poses[names.index('so101::gripper_link')]
        gripper=pose(pose_values(g)[:3],pose_values(g)[3:])
        arm=pose(row['arm'][:3],row['arm'][3:])
        fk=arm @ self.fk_gripper(self.config,row['joints'])
        pos,angle=self.residual(fk,gripper)
        tool=gripper[:3,3]+gripper[:3,:3] @ CUBE_REFERENCE
        goal=self.plan.base_goal(row,self.route_index)
        object_pose=pose_values(truth.target_pose)
        object_error=float(np.linalg.norm(np.asarray(object_pose[:3])-self.target.object[:3,3]))
        geometry=bool(truth.approach_geometry_valid and not truth.arm_external_contact and pos<=.002 and angle<=.001
            and object_error<=.004)
        return dict(goal=goal,tool=tool,error=float(np.linalg.norm(tool-self.plan.pre_world)),
            waypoint_error=float(np.linalg.norm(tool-self.plan.waypoints_world[self.route_index])),
            clearance=float(truth.arm_obstacle_clearance),geometry=geometry,
            fk_position_error=pos,fk_rotation_error=angle,external_contact=truth.arm_external_contact,
            object_pose=object_pose,object_position_error=object_error)

    def run(self):
        try:
            while not self.stop_event.wait(.002):
                row=self.sources.read()
                if row is None: continue
                now=row['now']; t=row['truth']; lease=row['known_status']
                source=(t.header.seq,t.header.stamp.to_nsec()//1000)
                if not self.streaming and self.plan.matches(lease) and lease.terminal_hold and self.latest.get('native_heading_ready'):
                    self.streaming=True; self.event('task_bound_after_terminal_hover',row,self.latest,ids=self.plan.ids)
                geo=self.physical(row); self.final_geometry=geo
                if self.streaming and source!=self.last_source:
                    self.last_source=source
                    valid_lease=(self.plan.matches(lease) and lease.terminal_hold and lease.lease_valid
                        and -.02<=now-lease.header.stamp.to_sec()<=.1
                        and -.02<=now-lease.command_stamp.to_sec()<=.3)
                    stamp=min(lease.header.stamp.to_nsec(),lease.command_stamp.to_nsec())//1000 if lease else 0
                    fields=[self.plan.task,source[0],source[1],t.world_epoch,1,*metrics(row,self.target),
                        self.plan.plan,self.plan.target,self.plan.scene,stamp,int(valid_lease),
                        int(geo['geometry']),*map(float,geo['goal']),geo['clearance']]
                    self.client('arm_control','hover_task_context',*fields)
                if now-self.last_status<.1: continue
                self.last_status=now
                hover=self.status('arm_hover_status'); self.latest=hover
                state=self.status('arm_approach_status'); arm=self.status('arm_control_status')
                if state.get('mode')!=0:
                    raise RuntimeError('No-close observer refuses another approach mode')
                grasp=self.status('arm_grasp_status')
                record=dict(sim_s=now,stage=self.stage,hover=hover,approach=state,arm=arm,
                    measured_joint_position=row['joints'].tolist(),
                    truth_sample_s=t.header.stamp.to_sec(),
                    goal_world=self.plan.pre_world.tolist(),actual_tool_world=geo['tool'].tolist(),
                    actual_goal_error=geo['error'],clearance=geo['clearance'],
                    route_index=self.route_index,actual_waypoint_error=geo['waypoint_error'],
                    waypoint_world=self.plan.waypoints_world[self.route_index].tolist(),
                    fk_position_error=geo['fk_position_error'],fk_rotation_error=geo['fk_rotation_error'],
                    external_contact=geo['external_contact'],geometry_valid=geo['geometry'],
                    object_world=geo['object_pose'],object_position_error=geo['object_position_error'])
                self.log.write(json.dumps(record,allow_nan=False)+'\n'); self.log.flush()
                if state.get('close_authorized') or abs(arm['joint_target'][5]-self.jaw)>1e-6:
                    raise RuntimeError('Jaw changed during no-close approach')
                if grasp.get('phase')!=0 or grasp.get('ownership_locked'):
                    raise RuntimeError('Bench grasp unexpectedly active')
                if state.get('phase') in (4,5): raise RuntimeError('Native approach exited: '+str(state))
                if self.stage=='waiting_hover' and hover.get('start_ready') and self.streaming:
                    self.expected_request=hover.get('last_request_id',0)
                    self.client('arm_control','hover_grasp_prepare',self.plan.task); self.stage='prepare'
                elif self.stage=='prepare' and hover.get('last_request_id',0)>self.expected_request:
                    if not hover.get('prepared'):
                        self.stage='waiting_hover'; continue
                    self.expected_request=state.get('last_request_id',0)
                    self.approach_request_count+=1
                    self.client('arm_control','hover_approach',self.plan.task,self.plan.plan+1,self.plan.target,self.plan.scene)
                    self.stage='wrong_binding'
                elif self.stage=='wrong_binding' and state.get('last_request_id',0)>self.expected_request:
                    if state.get('phase')!=0 or state.get('request_result')!=3:
                        raise RuntimeError('Wrong plan was not rejected')
                    if arm['target_sequence']!=self.baseline['target_sequence'] or np.max(np.abs(
                            np.asarray(arm['joint_target'])-self.baseline['joint_target']))>1e-6:
                        raise RuntimeError('Wrong-plan request changed arm targets')
                    self.event('wrong_plan_rejected_no_motion',row,state)
                    self.stage='start'
                elif self.stage=='start' and hover.get('prepared') and hover.get('start_ready'):
                    self.before_error=geo['error']; self.expected_request=state.get('last_request_id',0)
                    self.approach_request_count+=1
                    self.client('arm_control','hover_approach',*self.plan.ids); self.stage='start_ack'
                elif self.stage=='start' and not hover.get('prepared'):
                    self.stage='waiting_hover'
                elif self.stage=='start_ack' and state.get('last_request_id',0)>self.expected_request:
                    if state.get('request_result')!=1 or not state.get('ownership_locked'):
                        self.stage='waiting_hover'; continue
                    self.motion_started=True; self.stage='approach'
                    self.event('no_close_approach_started',row,state,initial_world_error=self.before_error)
                elif self.stage=='approach':
                    if state.get('phase')==3:
                        self.holding_since=None
                        if state.get('resume_ready') and hover.get('start_ready'):
                            self.expected_request=state.get('last_request_id',0)
                            self.client('arm_control','hover_resume',*self.plan.ids)
                            self.stage='resume_ack'; self.event('explicit_same_task_resume_requested',row,state)
                    elif state.get('phase')==2 and geo['waypoint_error']<=.015:
                        if self.route_index<len(self.plan.waypoints_world)-1:
                            self.event('side_unfold_waypoint_reached',row,state,
                                route_index=self.route_index,waypoint_error=geo['waypoint_error'])
                            self.route_index+=1; self.holding_since=None
                        elif self.holding_since is None: self.holding_since=now
                        elif now-self.holding_since>=3:
                            self.complete=True; self.stage='done'
                            self.event('no_close_pregrasp_held',row,state,actual_world_error=geo['error'])
                    else: self.holding_since=None
                elif self.stage=='resume_ack' and state.get('last_request_id',0)>self.expected_request:
                    self.stage='approach'
        except Exception as exc:
            self.error=str(exc)
        finally:
            self.log.close()

    def finish(self):
        self.stop_event.set(); self.thread.join(timeout=8)
        if self.thread.is_alive(): raise RuntimeError('Approach observer did not stop')
        if self.error: raise RuntimeError(self.error)
        final=self.status('arm_control_status'); state=self.status('arm_approach_status')
        geo=self.final_geometry
        if abs(final['joint_target'][5]-self.jaw)>1e-6: raise RuntimeError('Jaw target changed')
        return dict(approach_validated=self.complete,events=self.events,binding=self.plan.ids,
            swept_bounds=self.swept_bounds,
            approach_request_count=self.approach_request_count,
            pregrasp_world=self.plan.pre_world.tolist(),initial_error_m=self.before_error,
            waypoints_world=[p.tolist() for p in self.plan.waypoints_world],
            final_error_m=geo['error'],final_native_status=state,
            initial_joint_target=self.baseline['joint_target'],final_joint_target=final['joint_target'],
            initial_target_sequence=self.baseline['target_sequence'],final_target_sequence=final['target_sequence'],
            jaw_target_unchanged=True,close_authorized=False,grasp_executed=False,
            grasp_alignment_validated=False,
            approach_standoff_m=float(np.linalg.norm(self.plan.pre_world-self.target.object[:3,3])))
