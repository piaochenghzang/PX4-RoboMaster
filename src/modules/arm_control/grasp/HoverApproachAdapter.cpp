#include "../arm_control.hpp"
#include "WristUnfold.hpp"
#include <algorithm>

void ArmControl::stopHoverMotion()
{
    _approach_stop_pending=true;
    float frozen[ARM_DOF]{};
    if (!_command_initialized || !so101_hover::freezeCommand(_command_position,_joint_limits,frozen)) return;
    for (int j=0;j<ARM_DOF;++j) {
        _joint_target[j]=frozen[j]; _command_position[j]=frozen[j]; _command_velocity[j]=0.f;
    }
    _ee_target_position=extractPosition(computeURDFFK(_joint_target));
    _has_last_ik_solution=false; _arrival.reset(); _approach_stop_pending=false;
}

float ArmControl::hoverMotionClearance() const
{
    using so101_hover::FixedPhase;
    if (_fixed_grasp.owns() && _fixed_grasp.phase>=FixedPhase::Close && _fixed_grasp.phase<=FixedPhase::Done)
        return _hover_context.jaw_clearance;
    return _hover_context.clearance;
}

so101_hover::TargetResult ArmControl::applyHoverTarget(const so101_hover::Point &p,float max_joint_step,float unused_fraction)
{
    using so101_hover::TargetResult;
    if (!PX4_ISFINITE(max_joint_step) || max_joint_step<=0 || max_joint_step>.02f) return TargetResult::Rejected;
    const matrix::Vector3f raw{p.x,p.y,p.z}; matrix::Vector3f safe{};
    if (!sanitizeCartesianTarget(raw,safe) || (safe-raw).norm()>.0002f) {
        PX4_ERR("hover raw workspace rejected %.6f %.6f %.6f",(double)raw(0),(double)raw(1),(double)raw(2)); return TargetResult::Rejected;
    }
    float candidate[ARM_DOF]{},seed[ARM_DOF]{};
    for (int j=0;j<ARM_DOF;++j) seed[j]=_joint_target[j];
    _last_ik_result=solvePositionIK(seed,raw,candidate);
    if (_last_ik_result.status!=IKStatus::Success) {
        PX4_ERR("hover IK rejected status=%u error=%.6f",unsigned(_last_ik_result.status),(double)_last_ik_result.final_error); return TargetResult::Rejected;
    }
    float largest=0;
    for (int j=0;j<ARM_DOF;++j) largest=std::max(largest,fabsf(candidate[j]-seed[j]));
    const float scale=largest>max_joint_step ? max_joint_step/largest : 1.f;
    float desired[ARM_DOF]{};
    for (int j=0;j<ARM_DOF;++j) desired[j]=seed[j]+scale*(candidate[j]-seed[j]);
    return applyHoverJoints(desired,unused_fraction);
}

so101_hover::TargetResult ArmControl::applyHoverJoints(const float (&desired)[ARM_DOF],float unused_fraction)
{
    using so101_hover::TargetResult;
    if (!PX4_ISFINITE(unused_fraction) || unused_fraction<=0 || unused_fraction>1) return TargetResult::Rejected;
    float seed[ARM_DOF]{},candidate[ARM_DOF]{};
    for (int j=0;j<ARM_DOF;++j) seed[j]=_joint_target[j];
    // Triangle-chain upper bounds from production SDF, including all collision
    // corners and the jaw subtree; regression checks these constants against SDF.
    constexpr float radii[ARM_DOF]{.51f,.45f,.33f,.20f,.13f};
    // Reserve 10 mm clearance plus 6 mm for the next bounded body sample.
    const float budget=hoverMotionClearance()-.016f;
    float measured[ARM_DOF]{};
    for (int j=0;j<ARM_DOF;++j) measured[j]=_latest_joint_status.position[j];
    const float reserve=so101_hover::pendingDisplacement(seed,measured,radii);
    if (!PX4_ISFINITE(budget) || !PX4_ISFINITE(reserve)) return TargetResult::Rejected;
    if (budget<=.001f || reserve>=budget) {
        PX4_WARN("hover clearance pause budget=%.5f pending=%.5f",(double)budget,(double)reserve);
        return TargetResult::WaitClearance;
    }
    const float allowed=unused_fraction>=1.f ? budget : reserve+unused_fraction*(budget-reserve);
    if (!so101_hover::boundJointBudget(desired,seed,measured,_joint_limits,radii,allowed,candidate)) {
        PX4_ERR("hover invalid joint budget candidate"); return TargetResult::Rejected;
    }
    const auto next=extractPosition(computeURDFFK(candidate));
    matrix::Vector3f checked{};
    if (!sanitizeCartesianTarget(next,checked) || (checked-next).norm()>.0002f) {
        PX4_ERR("hover bounded FK workspace rejected next=%.6f %.6f %.6f old_z=%.6f",
            (double)next(0),(double)next(1),(double)next(2),(double)_ee_target_position(2)); return TargetResult::Rejected;
    }
    for (int j=0;j<ARM_DOF;++j) {
        if (!PX4_ISFINITE(candidate[j]) || fabsf(constrainJointPosition(j,candidate[j])-candidate[j])>1e-6f) {
            PX4_ERR("hover joint bound rejected joint=%d target=%.6f measured=%.6f",
                j,(double)candidate[j],(double)_latest_joint_status.position[j]); return TargetResult::Rejected;
        }
    }
    for (int j=0;j<ARM_DOF;++j) _joint_target[j]=candidate[j];
    _ee_target_position=next; _ee_target_valid=true; _has_last_ik_solution=false;
    ++_target_sequence; _arrival.reset();
    return TargetResult::Applied;
}

so101_hover::TargetResult ArmControl::applyHoverUnfold(float max_joint_step,float joint_speed,bool &complete,float unused_fraction)
{
    // .4f * .05f can round one float ULP above .02f. Accept only that
    // numerical neighbourhood and cap the actual command at exactly .02f.
    if (!PX4_ISFINITE(max_joint_step) || max_joint_step<=0 || max_joint_step>.020001f)
        return so101_hover::TargetResult::Rejected;
    max_joint_step=std::min(max_joint_step,.02f);
    float anchor[ARM_DOF]{},measured[ARM_DOF]{},desired[ARM_DOF]{};
    for (int j=0;j<ARM_DOF;++j) { anchor[j]=_joint_target[j]; measured[j]=_latest_joint_status.position[j]; }
    const bool recovery=_hover_approach.moving() && _approach_unfold_selected &&
        _ee_target_position(2)<_cartesian_limit.min_z+.002f;
    const auto result=so101_hover::wristUnfoldStep(anchor,measured,joint_speed,max_joint_step,desired,recovery);
    if (result==so101_hover::UnfoldResult::Invalid) return so101_hover::TargetResult::Rejected;
    if (result==so101_hover::UnfoldResult::Settled) {
        complete=true;
        // Generic no-close compensation runs every 20 ms after unfolding;
        // the existing fixed-grasp cadence remains 4 ms.
        ScheduleOnInterval(_hover_approach.moving() ? 20_ms : 4_ms);
    } else if (result==so101_hover::UnfoldResult::Move) return applyHoverJoints(desired,unused_fraction);
    return so101_hover::TargetResult::Applied;
}

void ArmControl::updateHoverApproach()
{
    const uint64_t now=hrt_absolute_time(); const auto &c=_hover_context;
    auto fresh=[now](uint64_t s,uint64_t age) { return s && s<=now+20000 && (s>=now || now-s<=age); };
    so101_hover::ApproachInput in{};
    in.now=now; in.binding={c.task_id,c.plan_id,c.target_id,c.scene_id};
    in.context_fresh=fresh(c.sample_time_us,100000) && fresh(c.timestamp,300000);
    in.lease_fresh=c.lease_valid && fresh(c.lease_sample_us,300000);
    in.arm_ready=_command_initialized && _joint_feedback_valid && fresh(_last_feedback_time,200000);
    in.geometry_valid=c.geometry_valid; in.clearance=hoverMotionClearance();
    in.tool_relative_speed=c.relative_speed;
    in.goal={c.approach_base[0],c.approach_base[1],c.approach_base[2]};
    in.accepted={_ee_target_position(0),_ee_target_position(1),_ee_target_position(2)};
    in.measured={_measured_ee_pose.position(0),_measured_ee_pose.position(1),_measured_ee_pose.position(2)};
    in.joint_speed=0;
    for (int j=0;j<ARM_DOF;++j) in.joint_speed=std::max(in.joint_speed,fabsf(_latest_joint_status.velocity[j]));
    in.hover_ready=_hover_gate.start_ready;
    constexpr float radii[ARM_DOF]{.51f,.45f,.33f,.20f,.13f};
    float anchor[ARM_DOF]{},measured[ARM_DOF]{};
    for (int j=0;j<ARM_DOF;++j) { anchor[j]=_joint_target[j]; measured[j]=_latest_joint_status.position[j]; }
    const float pending=so101_hover::pendingDisplacement(anchor,measured,radii);
    // Hysteresis only strengthens resumption: >=32 mm total clearance and
    // 8 mm unused beyond the unchanged 16 mm minimum/pending-motion reserve.
    in.clearance_ready=PX4_ISFINITE(pending) && c.clearance>=.032f && pending<=c.clearance-.024f;
    in.hard_hover_ok=!(_hover_gate.denial & (so101_hover::Missing|so101_hover::Stale|so101_hover::Invalid|
        so101_hover::Flight|so101_hover::Allocation|so101_hover::Reset));
    in.prepared=_hover_preparation.prepared_task==c.task_id;
    in.geometry_valid=in.geometry_valid && in.goal.x>=.05f && in.goal.x<=.4f &&
        fabsf(in.goal.y)<=.25f && in.goal.z>=.02f && in.goal.z<=.4f;
    arm_approach_request_s r{};
    const bool requested=_approach_request_sub.update(&r);
    if (requested && r.action<=3) {
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
        if (!_grasp.owns() && !_fixed_grasp.owns() && !c.fixed_grasp && fresh(r.timestamp,300000)) {
            const bool was_idle=!_hover_approach.owns();
            _hover_approach.request(r.request_id,r.action,{r.task_id,r.plan_id,r.target_id,r.scene_id},in);
            if (was_idle && _hover_approach.owns()) {
                _approach_jaw_target=_joint_target[GRIPPER_INDEX];
                _approach_unfold_selected=so101_hover::knownFoldedArm(anchor,measured);
                _approach_unfold_complete=false; _approach_unfold_last_send=0;
                ScheduleOnInterval(_approach_unfold_selected ? 50_ms : 20_ms);
            }
        }
#endif
    }
    in.unfold_complete=!_approach_unfold_selected || _approach_unfold_complete;
    bool send=false,stop=false; const auto p=_hover_approach.step(in,send,stop);
    if (stop || _approach_stop_pending) stopHoverMotion();
    const bool unfold=_approach_unfold_selected && !_approach_unfold_complete;
    if (_hover_approach.moving() && in.hover_ready && unfold &&
        (!_approach_unfold_last_send || now-_approach_unfold_last_send>=50000)) {
        // No catch-up increment: at most the existing .4 rad/s, .02 rad
        // wrist-unfold budget per 50 ms, independently of callback cadence.
        const float seconds=float(_approach_unfold_last_send ? std::min(uint64_t(50000),now-_approach_unfold_last_send) : 50000)*1e-6f;
        _approach_unfold_last_send=now;
        // Preserve half the free travel while leaving the known close-to-body
        // folded posture; do not deliberately exhaust the pending-motion budget.
        const auto outcome=applyHoverUnfold(.4f*seconds,in.joint_speed,_approach_unfold_complete,.5f);
        if (outcome==so101_hover::TargetResult::WaitClearance) {
            _hover_approach.pause(so101_hover::PauseReason::Clearance); stopHoverMotion();
        } else if (outcome==so101_hover::TargetResult::Rejected) {
            _hover_approach.fail(so101_hover::ApproachFault::IK); stopHoverMotion();
        }
    } else if (send) {
        const auto outcome=applyHoverTarget(p,_hover_approach.max_joint_step,.5f);
        if (outcome==so101_hover::TargetResult::WaitClearance) {
            _hover_approach.pause(so101_hover::PauseReason::Clearance); stopHoverMotion();
        } else if (outcome==so101_hover::TargetResult::Rejected) {
            _hover_approach.fail(so101_hover::ApproachFault::IK); stopHoverMotion();
        }
    }
    // No branch ever writes a jaw target; verify it did not change via another owner.
    if (_hover_approach.owns() && fabsf(_joint_target[GRIPPER_INDEX]-_approach_jaw_target)>1e-6f) {
        _hover_approach.fail(so101_hover::ApproachFault::BindingChanged); stopHoverMotion();
    }
    // Exactly one motion owner publishes the shared status. The explicit mode
    // preserves the old no-close phase/authorization contract.
    if (!_hover_approach.owns()) {
        updateFixedGrasp(requested && r.action>=4 ? &r : nullptr,in);
        if (_fixed_grasp.reportsStatus(c.fixed_grasp)) return;
    }
    arm_approach_status_s s{}; s.timestamp=now;
    s.mode=arm_approach_status_s::MODE_APPROACH;
    const auto &b=_hover_approach.binding;
    s.task_id=b.task; s.plan_id=b.plan; s.target_id=b.target; s.scene_id=b.scene;
    s.last_request_id=_hover_approach.last_id; s.request_result=_hover_approach.request_result;
    s.phase=uint8_t(_hover_approach.phase); s.fault=uint8_t(_hover_approach.fault);
    s.pause_reason=uint8_t(_hover_approach.pause_reason); s.resume_ready=_hover_approach.resumeReady(in);
    s.ownership_locked=_hover_approach.owns(); s.motion_authorized=_hover_approach.moving() && in.hover_ready;
    s.close_authorized=false; s.lease_valid=in.lease_fresh;
    s.unfold_complete=_approach_unfold_selected && _approach_unfold_complete;
    s.goal_base[0]=in.goal.x; s.goal_base[1]=in.goal.y; s.goal_base[2]=in.goal.z;
    s.goal_error=(in.goal-in.measured).norm(); s.clearance=in.clearance;
    s.jaw_target=_joint_target[GRIPPER_INDEX]; _approach_status_pub.publish(s);
}
