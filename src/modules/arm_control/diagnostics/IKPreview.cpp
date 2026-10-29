#if defined(CONFIG_ARCH_BOARD_PX4_SITL)
#include "../arm_control.hpp"

int ArmControl::previewCartesianIK(const matrix::Vector3f &target,const float *explicit_seed)
{
    // Snapshot the published state, not mutable trajectory/IK caches. This
    // command never calls sanitizeCartesianTarget/updateIKTarget, publishes a
    // setpoint, writes _last_ik_result or changes arrival/ownership state.
    uORB::Subscription sub{ORB_ID(arm_control_status)};
    arm_control_status_s state{};
    if (!sub.copy(&state) || !state.timestamp || hrt_elapsed_time(&state.timestamp)>300000 ||
        !state.command_initialized || !state.feedback_valid || !PX4_ISFINITE(state.feedback_age) || state.feedback_age>.2f) {
        PX4_ERR("fresh initialized arm state required for IK inspection"); return PX4_ERROR;
    }
    // Explicit seeds remain local diagnostic inputs; validate them before the
    // workspace refusal so an invalid seed never becomes a successful query.
    float seed[ARM_DOF]{},q[ARM_DOF]{};
    for (int i=0;i<ARM_DOF;++i) {
        seed[i]=explicit_seed ? explicit_seed[i] : state.joint_target[i];
        if (!PX4_ISFINITE(seed[i]) || seed[i]<_joint_limits[i].min_rad || seed[i]>_joint_limits[i].max_rad) return PX4_ERROR;
    }
    const bool workspace=target.isAllFinite() && target(0)>=_cartesian_limit.min_x && target(0)<=_cartesian_limit.max_x &&
        target(1)>=_cartesian_limit.min_y && target(1)<=_cartesian_limit.max_y &&
        target(2)>=_cartesian_limit.min_z && target(2)<=_cartesian_limit.max_z;
    if (!workspace) {
        PX4_INFO("IK_CHECK_META {\"read_only\":true,\"workspace_valid\":false,\"ik_evaluated\":false}");
        PX4_INFO("IK_CHECK_RESULT {\"ik_success\":false,\"joint_valid\":false}");
        return PX4_OK;
    }
    const auto result=solvePositionIK(seed,target,q);
    bool legal=PX4_ISFINITE(result.final_error);
    for (int i=0;i<ARM_DOF;++i) legal=legal && PX4_ISFINITE(q[i]) &&
        q[i]>=_joint_limits[i].min_rad && q[i]<=_joint_limits[i].max_rad;
    if (!legal) {
        PX4_INFO("IK_CHECK_META {\"read_only\":true,\"workspace_valid\":true,\"ik_evaluated\":true}");
        PX4_INFO("IK_CHECK_RESULT {\"ik_success\":false,\"joint_valid\":false}");
        return PX4_OK;
    }
    const auto fk=extractPose(computeURDFFK(q));
    // Keep each record below PX4's bounded log text size; a truncated JSON
    // record must never be interpreted as a valid inspection result.
    PX4_INFO("IK_CHECK_META {\"read_only\":true,\"workspace_valid\":true,\"ik_evaluated\":true}");
    PX4_INFO("IK_CHECK_RESULT {\"ik_success\":%s,\"joint_valid\":true}",
        result.status==IKStatus::Success ? "true" : "false");
    // seed_sequence always identifies the published control-state snapshot;
    // explicit-seed provenance is recorded by the read-only caller.
    PX4_INFO("IK_CHECK_ERROR {\"error_m\":%.9g,\"seed_sequence\":%u}",
        (double)result.final_error,unsigned(state.target_sequence));
    PX4_INFO("IK_CHECK_Q {\"q\":[%.9g,%.9g,%.9g,%.9g,%.9g]}",
        (double)q[0],(double)q[1],(double)q[2],(double)q[3],(double)q[4]);
    PX4_INFO("IK_CHECK_FK {\"fk\":[%.9g,%.9g,%.9g]}",
        (double)fk.position(0),(double)fk.position(1),(double)fk.position(2));
    PX4_INFO("IK_CHECK_QUAT {\"quat\":[%.9g,%.9g,%.9g,%.9g]}",
        (double)fk.quaternion(0),(double)fk.quaternion(1),(double)fk.quaternion(2),(double)fk.quaternion(3));
    return PX4_OK;
}
#endif // CONFIG_ARCH_BOARD_PX4_SITL
