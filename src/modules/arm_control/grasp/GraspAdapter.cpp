#include "../arm_control.hpp"
#include <algorithm>

void ArmControl::pollGraspFeedback()
{
    arm_grasp_feedback_s feedback{};
    if (_arm_grasp_feedback.update(&feedback)) _grasp_feedback=feedback;
}
so101_grasp::Input ArmControl::graspInput() const
{
    so101_grasp::Input in{};
    in.now=hrt_absolute_time(); in.initialized=_command_initialized;
    in.joint_age=_last_feedback_time ? float(in.now-_last_feedback_time)*1e-6f : INFINITY;
    const auto &f=_grasp_feedback;
    in.grasp_age=INFINITY;
    if (f.timestamp && f.timestamp_sample && in.now>=f.timestamp && f.timestamp_sample<=in.now+100000) {
        const float receive=float(in.now-f.timestamp)*1e-6f;
        const float sampled=in.now>f.timestamp_sample ? float(in.now-f.timestamp_sample)*1e-6f : 0.f;
        in.grasp_age=std::max(receive,sampled);
    }
    in.accepted_target={_ee_target_position(0),_ee_target_position(1),_ee_target_position(2)};
    in.position_error=(_ee_target_position-_measured_ee_pose.position).norm();
    in.joint_error=0; in.joint_speed=0;
    for (int i=0; i<ARM_DOF; ++i) {
        in.joint_error=std::max(in.joint_error,fabsf(_joint_target[i]-_latest_joint_status.position[i]));
        in.joint_speed=std::max(in.joint_speed,fabsf(_latest_joint_status.velocity[i]));
    }
    in.jaw_speed=_latest_joint_status.velocity[GRIPPER_INDEX];
    in.target_sequence=_target_sequence; in.sample_stamp=f.timestamp_sample;
    in.sequence=f.source_sequence; in.epoch=f.world_epoch; in.target_id=f.target_id; in.profile=f.profile_id;
    in.source_identity=(uint16_t(f.source_system)<<8)|f.source_component;
    in.capabilities=f.capabilities; in.frames=f.contact_frames;
    in.bilateral=f.bilateral_fraction; in.support=f.support_fraction;
    in.other=f.other_fraction; in.support_arm=f.support_arm_fraction;
    in.fixed_force=f.fixed_normal_n; in.moving_force=f.moving_normal_n;
    in.object_height=f.object_height_world; in.clearance=f.body_support_clearance;
    in.object_local={f.object_center_gripper[0],f.object_center_gripper[1],f.object_center_gripper[2]};
    // FK tool frame differs from actual gripper_link by the fixed tool transform.
    const float error[]{in.object_local.x-.0025f,in.object_local.y+.000218f,in.object_local.z+.088f};
    float base_error[3]{};
    for (int row=0; row<3; ++row) {
        for (int col=0; col<3; ++col) {
            for (int k=0; k<3; ++k) base_error[row]+=_measured_ee_pose.rotation(row,k)*_tool_tf(col,k)*error[col];
        }
    }
    in.alignment_delta_base={base_error[0],base_error[1],base_error[2]};
    in.finite=in.accepted_target.finite() && in.object_local.finite() && in.alignment_delta_base.finite()
        && PX4_ISFINITE(in.position_error) && PX4_ISFINITE(in.jaw_speed) && PX4_ISFINITE(in.object_height);
    in.strict_arrived=_arrival.arrived;
    return in;
}
void ArmControl::processGraspRequests(const so101_grasp::Input &input)
{
    arm_grasp_request_s request{};
    for (int count=0; count<4 && _arm_grasp_request.update(&request); ++count) {
#if !defined(CONFIG_ARCH_BOARD_PX4_SITL)
        // This v1 profile relies on Gazebo truth and a fixed bench anchor.
        // It is not a hardware/flight grasp interface.
        if (request.action==arm_grasp_request_s::START) request.profile_id=0;
#endif
        _grasp.request(request.request_id,request.action,request.profile_id,request.target_id,request.hold_duration_s,input);
    }
}
void ArmControl::executeGraspAction(const so101_grasp::Action &action, const so101_grasp::Input &input)
{
    using so101_grasp::ActionType;
    if (action.type==ActionType::Stop) _grasp_hold_pending=true;
    if (_grasp_hold_pending && input.joint_age<.2f && _joint_feedback_valid) {
        // Emergency cancellation clears residual trajectories; keep the jaw
        // bias, rather than unloading contact by targeting its measured angle.
        for (int i=0; i<ARM_DOF; ++i) {
            _joint_target[i]=constrainJointPosition(i,_latest_joint_status.position[i]);
            _command_position[i]=_joint_target[i]; _command_velocity[i]=0.f;
        }
        _ee_target_position=extractPosition(computeURDFFK(_joint_target));
        _has_last_ik_solution=false; _arrival.reset(); _grasp_hold_pending=false;
    }
    if (action.type==ActionType::None || action.type==ActionType::Stop) return;
    if (action.type==ActionType::JawOnly) {
        _joint_target[GRIPPER_INDEX]=constrainJointPosition(GRIPPER_INDEX,action.jaw); return;
    }
    const matrix::Vector3f target{action.target.x,action.target.y,action.target.z};
    matrix::Vector3f safe{};
    const bool accepted=sanitizeCartesianTarget(target,safe) && (safe-target).norm()<=.0002f && updateIKTarget(target);
    _grasp.acknowledge(accepted);
    if (accepted) _joint_target[GRIPPER_INDEX]=constrainJointPosition(GRIPPER_INDEX,action.jaw);
    else _grasp_hold_pending=true;
}
void ArmControl::publishGraspStatus()
{
    const auto in=graspInput(); arm_grasp_status_s out{};
    out.timestamp=in.now; out.active_request_id=_grasp.active_id; out.last_request_id=_grasp.last_id;
    out.last_request_result=uint8_t(_grasp.last_request_result);
    out.phase=uint8_t(_grasp.phase); out.result=uint8_t(_grasp.result);
    out.fault_reason=uint8_t(_grasp.fault); out.payload_state=uint8_t(_grasp.payload);
    out.failed_phase=uint8_t(_grasp.failed_phase);
    out.completed_success=_grasp.completed_success; out.ownership_locked=_grasp.owns();
    out.strict_position_reached=in.strict_arrived; out.bench_near=_grasp.benchNear(in);
    out.target_sequence=in.target_sequence;
    out.phase_elapsed_s=_grasp.entered && in.now>=_grasp.entered ? float(in.now-_grasp.entered)*1e-6f : 0.f;
    out.feedback_age=in.grasp_age; out.alignment_error=_grasp.alignment_error;
    out.bilateral_fraction=in.bilateral; out.fixed_normal_n=in.fixed_force; out.moving_normal_n=in.moving_force;
    out.vertical_force_n=_grasp_feedback.vertical_force_n; out.initial_motion_shift=_grasp.motion_shift;
    out.hold_drift=_grasp.hold_drift; out.actual_lift=_grasp.actual_lift;
    _arm_grasp_status_pub.publish(out);
}
