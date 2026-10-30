#include "../arm_control.hpp"
#include <algorithm>

void ArmControl::updateFixedGrasp(const arm_approach_request_s *request,const so101_hover::ApproachInput &motion)
{
    using namespace so101_hover;
    FixedInput input{}; input.motion=motion; input.feedback=graspInput();
#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
    input.contact_only=_grasp_feedback.source==arm_grasp_feedback_s::SOURCE_SIM_CONTACT;
    input.profile=_hover_context.fixed_grasp &&
        (_grasp_feedback.source==arm_grasp_feedback_s::SOURCE_SIM_GROUND_TRUTH || input.contact_only);
#endif
    constexpr float radii[ARM_DOF]{.51f,.45f,.33f,.20f,.13f};
    float anchor[ARM_DOF]{},measured[ARM_DOF]{};
    for (int j=0;j<ARM_DOF;++j) { anchor[j]=_joint_target[j]; measured[j]=_latest_joint_status.position[j]; }
    const float pending=pendingDisplacement(anchor,measured,radii);
    input.jaw_sweep_valid=PX4_ISFINITE(_hover_context.jaw_clearance) && _hover_context.jaw_clearance>.016f;
    input.close_ready=input.profile && _hover_gate.start_ready &&
        fixedCloseGeometryReady(_hover_context.alignment_error,_hover_context.relative_speed,_hover_context.orientation_error,
            _fixed_grasp.phase==FixedPhase::Close) &&
        precisionPositionReady(_hover_context.hover_position_error) &&
        _fixed_unfold_complete && input.jaw_sweep_valid &&
        PX4_ISFINITE(pending) && _hover_context.jaw_clearance>.016f+pending+.0001f;
    input.unfold_complete=_fixed_unfold_complete;
    input.lift_goal={_hover_context.lift_base[0],_hover_context.lift_base[1],_hover_context.lift_base[2]};
    const uint64_t now=motion.now;
    if (request && !_grasp.owns() && !_hover_approach.owns() &&
        request->timestamp && request->timestamp<=now && now-request->timestamp<=300000) {
        const bool idle=!_fixed_grasp.owns();
        _fixed_grasp.request(request->request_id,request->action,
            {request->task_id,request->plan_id,request->target_id,request->scene_id},input);
        if (idle && _fixed_grasp.owns()) {
            _fixed_unfold_complete=false;
            // The fixed demo alone uses a 50 ms unfold command budget. The
            // normal controller, joint limits and collision reserve are unchanged.
            ScheduleOnInterval(50_ms);
        }
    }
    const auto before=_fixed_grasp.phase;
    const auto action=_fixed_grasp.step(input);
    if (before!=FixedPhase::Fault && _fixed_grasp.phase==FixedPhase::Fault) {
        PX4_ERR("fixed fault=%u geometry=%u clearance=%.5f lift_delta=%.5f",unsigned(_fixed_grasp.fault),
            unsigned(motion.geometry_valid),(double)motion.clearance,(double)(input.lift_goal-motion.goal).norm());
        PX4_ERR("fixed finite=%u age=%.4f other=%.4f support_arm=%.4f joint_speed=%.4f",unsigned(input.feedback.finite),
            (double)input.feedback.grasp_age,(double)input.feedback.other,(double)input.feedback.support_arm,(double)motion.joint_speed);
        PX4_ERR("fixed contract context=%u lease=%u hard_hover=%u binding=%u",unsigned(motion.context_fresh),
            unsigned(motion.lease_fresh),unsigned(motion.hard_hover_ok),unsigned(motion.binding==_fixed_grasp.binding));
        PX4_ERR("fixed timing sample_age=%.4f receipt_age=%.4f denial=%u",
            (double(now)-double(_hover_context.sample_time_us))*1e-6,
            (double(now)-double(_hover_context.timestamp))*1e-6,unsigned(_hover_gate.denial));
    }
    if (action.stop) stopHoverMotion();
    if (action.send && !_fixed_unfold_complete) {
        // Select the existing downward-facing IK branch by a fixed wrist
        // unfold. The SAME joint, workspace recovery and swept budget apply.
        const auto result=applyHoverUnfold(action.max_joint_step,motion.joint_speed,_fixed_unfold_complete);
        if (result==TargetResult::Rejected) { _fixed_grasp.fail(FixedFault::IK); stopHoverMotion(); }
        else if (result==TargetResult::WaitClearance) { _fixed_grasp.paused=true; stopHoverMotion(); }
    } else if (action.send) {
        const auto result=applyHoverTarget(action.goal,action.max_joint_step);
        if (result==TargetResult::Rejected) { _fixed_grasp.fail(FixedFault::IK); stopHoverMotion(); }
        else if (result==TargetResult::WaitClearance) { _fixed_grasp.paused=true; stopHoverMotion(); }
    }
    if (action.jaw && !_fixed_grasp.paused && _fixed_grasp.phase!=FixedPhase::Fault &&
        _fixed_grasp.phase!=FixedPhase::Canceled) {
        // Actual pad/support geometry includes the COMPLETE jaw stroke. The
        // normal arm budget still reserves pending arm tracking displacement.
        const float jaw_clearance=_fixed_grasp.phase==FixedPhase::Approach ? _hover_context.clearance : _hover_context.jaw_clearance;
        // applyHoverTarget above may have changed the arm target THIS cycle.
        // Reserve that new pending displacement before adding a jaw increment.
        for (int j=0;j<ARM_DOF;++j) anchor[j]=_joint_target[j];
        const float free=jaw_clearance-.016f-pendingDisplacement(anchor,measured,radii);
        if (PX4_ISFINITE(free) && free>.0001f) {
            const float step=std::min(action.max_jaw_step,free/.11f);
            const float delta=math::constrain(action.jaw_target-_joint_target[GRIPPER_INDEX],-step,step);
            _joint_target[GRIPPER_INDEX]=constrainJointPosition(GRIPPER_INDEX,_joint_target[GRIPPER_INDEX]+delta);
        }
    }
    if (!_fixed_grasp.owns()) return;
    arm_approach_status_s out{};out.timestamp=now;
    out.mode=arm_approach_status_s::MODE_FIXED_GRASP;
    const auto &b=_fixed_grasp.binding;
    out.task_id=b.task;out.plan_id=b.plan;out.target_id=b.target;out.scene_id=b.scene;
    out.phase=uint8_t(_fixed_grasp.phase);out.fault=uint8_t(_fixed_grasp.fault);
    out.last_request_id=_fixed_grasp.last_id;out.request_result=_fixed_grasp.request_result;
    out.ownership_locked=_fixed_grasp.owns();out.paused=_fixed_grasp.paused;
    out.unfold_complete=_fixed_unfold_complete;
    out.close_authorized=_fixed_grasp.phase==FixedPhase::Close && input.close_ready &&
        motion.hover_ready && !_fixed_grasp.paused && _fixed_grasp.safe(input);
    out.close_entry_ready=_fixed_grasp.phase==FixedPhase::Approach && input.close_ready &&
        motion.hover_ready && !_fixed_grasp.paused && _fixed_grasp.safe(input);
    out.completed_success=_fixed_grasp.phase==FixedPhase::Done;
    out.actual_lift=_fixed_grasp.actual_lift;out.feedback_age=input.feedback.grasp_age;
    out.bilateral_fraction=input.feedback.bilateral;
    out.fixed_normal_n=input.feedback.fixed_force;out.moving_normal_n=input.feedback.moving_force;
    out.motion_authorized=motion.hover_ready && !_fixed_grasp.paused && _fixed_grasp.safe(input) &&
        _fixed_grasp.phase!=FixedPhase::Fault && _fixed_grasp.phase!=FixedPhase::Canceled;
    out.lease_valid=motion.lease_fresh;
    out.goal_base[0]=motion.goal.x;out.goal_base[1]=motion.goal.y;out.goal_base[2]=motion.goal.z;
    out.goal_error=(motion.goal-motion.measured).norm();
    out.jaw_target=_joint_target[GRIPPER_INDEX];
    out.clearance=motion.clearance;_approach_status_pub.publish(out);
}
