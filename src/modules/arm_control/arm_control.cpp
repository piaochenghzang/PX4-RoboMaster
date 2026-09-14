/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "arm_control.hpp"

#include <cmath>
#include <cstring>

ArmControl::ArmControl() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
    for (int i = 0; i < ARM_DOF; ++i) {
        const JointOrigin &origin = _joint_origins[i];
        _joint_origin_tf[i] = makeRPYTransform(origin.x, origin.y, origin.z, origin.roll, origin.pitch, origin.yaw);
    }

    _tool_tf = makeRPYTransform(-0.0079f, -0.000218121f, -0.0981274f, 0.0f, PI, 0.0f);
}

ArmControl::~ArmControl() = default;

bool ArmControl::init()
{
    ScheduleOnInterval(100_ms);
    return true;
}

void ArmControl::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    const float dt = updateDt();
    ++_run_count;

    pollJointFeedback();

    if (!_command_initialized) {
        updateControlState();
        publishControlStatus();
        return;
    }

    processCartesianSetpoint();
    updateControlState();
    generateAndPublishJointCommand(dt);

    _command_ee_pose = extractPose(computeURDFFK(_command_position));
    publishControlStatus();
}

float ArmControl::updateDt()
{
    const hrt_abstime now = hrt_absolute_time();
    float dt = 0.1f;

    if (_last_run != 0) {
        dt = math::constrain((now - _last_run) * 1e-6f, 0.001f, 0.2f);
    }

    _last_run = now;
    return dt;
}

void ArmControl::pollJointFeedback()
{
    arm_joint_status_s status{};

    if (_arm_joint_status.update(&status) && updateJointFeedback(status)) {
        _last_feedback_time = hrt_absolute_time();
        _joint_feedback_valid = true;
    }
}

void ArmControl::processCartesianSetpoint()
{
    arm_cartesian_setpoint_s setpoint{};

    if (!_arm_cartesian_setpoint.update(&setpoint)) {
        return;
    }

    if (!setpoint.valid) {
        PX4_WARN("Cartesian target rejected");
        return;
    }

    const matrix::Vector3f target_position{setpoint.position[0], setpoint.position[1], setpoint.position[2]};

    if (!updateIKTarget(target_position)) {
        PX4_WARN("Cartesian IK target rejected");
        return;
    }

    _joint_target[GRIPPER_INDEX] = constrainJointPosition(GRIPPER_INDEX, setpoint.gripper);
}

void ArmControl::generateAndPublishJointCommand(float dt)
{
    arm_joint_command_s msg{};
    msg.timestamp = hrt_absolute_time();
    msg.joint_count = MOTOR_COUNT;

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        const float target = constrainJointPosition(i, _joint_target[i]);
        updateJointTrajectory(i, target, dt);
        msg.position[i] = _command_position[i];
        msg.velocity[i] = _command_velocity[i];
    }

    _arm_joint_command_pub.publish(msg);
}

bool ArmControl::updateJointFeedback(const arm_joint_status_s &status)
{
    const int count = status.joint_count > MOTOR_COUNT ? MOTOR_COUNT : status.joint_count;

    if (count < MOTOR_COUNT) {
        return false;
    }

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        if (!PX4_ISFINITE(status.position[i]) || !PX4_ISFINITE(status.velocity[i])) {
            return false;
        }
    }

    _latest_joint_status = status;

    float measured_q[ARM_DOF]{};

    for (int i = 0; i < ARM_DOF; ++i) {
        measured_q[i] = _latest_joint_status.position[i];
    }

    _measured_ee_pose = extractPose(computeURDFFK(measured_q));

    if (!_command_initialized) {
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            _command_position[i] = _latest_joint_status.position[i];
            _command_velocity[i] = 0.0f;
            _joint_target[i] = _latest_joint_status.position[i];
        }

        _ee_target_position = _measured_ee_pose.position;
        _ee_target_valid = true;
        _command_ee_pose = _measured_ee_pose;
        _command_initialized = true;
    }

    return true;
}

int ArmControl::task_spawn(int argc, char *argv[])
{
    ArmControl *instance = new ArmControl();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }
    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
}

int ArmControl::print_status()
{
    PX4_INFO("running");
    PX4_INFO("run count: %lu", static_cast<unsigned long>(_run_count));
    PX4_INFO("command initialized: %s", _command_initialized ? "yes" : "no");
    PX4_INFO("joint feedback: %s", _joint_feedback_valid ? "valid" : "invalid");

    if (_command_initialized) {
        PX4_INFO("Command EE: x=%.4f y=%.4f z=%.4f", (double)_command_ee_pose.position(0), (double)_command_ee_pose.position(1), (double)_command_ee_pose.position(2));
        PX4_INFO("Command quat: w=%.4f x=%.4f y=%.4f z=%.4f", (double)_command_ee_pose.quaternion(0), (double)_command_ee_pose.quaternion(1), (double)_command_ee_pose.quaternion(2), (double)_command_ee_pose.quaternion(3));
    }

    if (_joint_feedback_valid) {
        PX4_INFO("Measured EE: x=%.4f y=%.4f z=%.4f", (double)_measured_ee_pose.position(0), (double)_measured_ee_pose.position(1), (double)_measured_ee_pose.position(2));
    }

    return 0;
}

int ArmControl::custom_command(int argc, char *argv[])
{
    if (argc > 0 && !strcmp(argv[0], "test_fk")) {
        ArmControl *instance = get_instance();

        if (!instance) {
            return PX4_ERROR;
        }

        instance->testForwardKinematics();
        return PX4_OK;
    }

    if (argc > 0 && !strcmp(argv[0], "test_jacobian")) {
        ArmControl *instance = get_instance();

        if (!instance) {
            return PX4_ERROR;
        }

        instance->testJacobian();
        return PX4_OK;
    }

    if (argc > 0 && !strcmp(argv[0], "test_position_ik")) {
        ArmControl *instance = get_instance();

        if (!instance) {
            return PX4_ERROR;
        }

        instance->testPositionIK();
        return PX4_OK;
    }

    if (argc > 0 && !strcmp(argv[0], "test_warm_start")) {
        ArmControl *instance = get_instance();

        if (!instance) {
            return PX4_ERROR;
        }

        instance->testWarmStart();
        return PX4_OK;
    }

    if (argc > 0 && !strcmp(argv[0], "test_cartesian_ik")) {
        ArmControl *instance = get_instance();

        if (!instance) {
            return PX4_ERROR;
        }

        instance->testCartesianIKPipeline();
        return PX4_OK;
    }

    return print_usage("unknown command");
}

int ArmControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
SO101 arm control module running on a PX4 work queue.

### Example
$ arm_control test_fk
$ arm_control test_jacobian
$ arm_control test_position_ik
$ arm_control test_warm_start
$ arm_control test_cartesian_ik
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("arm_control", "module");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    PRINT_MODULE_USAGE_COMMAND("test_fk");
    PRINT_MODULE_USAGE_COMMAND("test_jacobian");
    PRINT_MODULE_USAGE_COMMAND("test_position_ik");
    PRINT_MODULE_USAGE_COMMAND("test_warm_start");
    PRINT_MODULE_USAGE_COMMAND("test_cartesian_ik");
    return 0;
}

float ArmControl::constrainJointPosition(int joint, float target_rad)
{
    const JointLimit &limit = _joint_limits[joint];
    return math::constrain(target_rad, limit.min_rad, limit.max_rad);
}

void ArmControl::updateJointTrajectory(int joint, float target, float dt)
{
    const JointLimit &limit = _joint_limits[joint];
    float &q = _command_position[joint];
    float &v = _command_velocity[joint];

    const float error = target - q;

    if (fabsf(error) < 1e-4f && fabsf(v) < 1e-3f) {
        q = target;
        v = 0.0f;
        return;
    }

    const float direction = error >= 0.0f ? 1.0f : -1.0f;
    const float distance = fabsf(error);
    const float speed_along_target = v * direction;
    const float stopping_distance = speed_along_target > 0.0f ? speed_along_target * speed_along_target / (2.0f * limit.max_acceleration) : 0.0f;

    float desired_acceleration = 0.0f;

    if (distance <= stopping_distance) {
        desired_acceleration = -direction * limit.max_acceleration;
    } else if (speed_along_target < limit.max_velocity) {
        desired_acceleration = direction * limit.max_acceleration;
    }

    v += desired_acceleration * dt;
    v = math::constrain(v, -limit.max_velocity, limit.max_velocity);

    const float old_q = q;
    q += v * dt;

    if ((target - old_q) * (target - q) <= 0.0f) {
        q = target;
        v = 0.0f;
    }
}

Matrix4f ArmControl::makeIdentityTransform()
{
    Matrix4f T{};
    T(0, 0) = 1.0f;
    T(1, 1) = 1.0f;
    T(2, 2) = 1.0f;
    T(3, 3) = 1.0f;
    return T;
}

Matrix4f ArmControl::makeRPYTransform(float x, float y, float z, float roll, float pitch, float yaw)
{
    return makeTranslation(x, y, z) * makeRotationZ(yaw) * makeRotationY(pitch) * makeRotationX(roll);
}

Matrix4f ArmControl::makeTranslation(float x, float y, float z)
{
    Matrix4f T = makeIdentityTransform();
    T(0, 3) = x;
    T(1, 3) = y;
    T(2, 3) = z;
    return T;
}

Matrix4f ArmControl::makeRotationX(float roll)
{
    const float c = cosf(roll);
    const float s = sinf(roll);

    Matrix4f R = makeIdentityTransform();
    R(1, 1) = c;
    R(1, 2) = -s;
    R(2, 1) = s;
    R(2, 2) = c;
    return R;
}

Matrix4f ArmControl::makeRotationY(float pitch)
{
    const float c = cosf(pitch);
    const float s = sinf(pitch);

    Matrix4f R = makeIdentityTransform();
    R(0, 0) = c;
    R(0, 2) = s;
    R(2, 0) = -s;
    R(2, 2) = c;
    return R;
}

Matrix4f ArmControl::makeRotationZ(float yaw)
{
    const float c = cosf(yaw);
    const float s = sinf(yaw);

    Matrix4f R = makeIdentityTransform();
    R(0, 0) = c;
    R(0, 1) = -s;
    R(1, 0) = s;
    R(1, 1) = c;
    return R;
}

Matrix4f ArmControl::computeURDFFK(const float q[ARM_DOF]) const
{
    Matrix4f T = makeIdentityTransform();

    for (int i = 0; i < ARM_DOF; ++i) {
        T = T * _joint_origin_tf[i] * makeRotationZ(q[i]);
    }

    return T * _tool_tf;
}

EndEffectorPose ArmControl::extractPose(const Matrix4f &T)
{
    EndEffectorPose pose{};
    pose.position = matrix::Vector3f(T(0, 3), T(1, 3), T(2, 3));

    matrix::Dcmf rotation{};

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            rotation(row, col) = T(row, col);
        }
    }

    pose.rotation = rotation;
    pose.quaternion = matrix::Quatf(rotation);
    return pose;
}

matrix::Vector3f ArmControl::extractPosition(const Matrix4f &T)
{
    return matrix::Vector3f(T(0, 3), T(1, 3), T(2, 3));
}

JacobianPos ArmControl::computePositionJacobian(const float q[ARM_DOF]) const
{
    Matrix4f T = makeIdentityTransform();
    matrix::Vector3f joint_position[ARM_DOF]{};
    matrix::Vector3f joint_axis[ARM_DOF]{};

    for (int i = 0; i < ARM_DOF; ++i) {
        T = T * _joint_origin_tf[i];
        joint_position[i] = matrix::Vector3f(T(0, 3), T(1, 3), T(2, 3));
        joint_axis[i] = matrix::Vector3f(T(0, 2), T(1, 2), T(2, 2));
        T = T * makeRotationZ(q[i]);
    }

    T = T * _tool_tf;
    const matrix::Vector3f pe(T(0, 3), T(1, 3), T(2, 3));

    JacobianPos J{};

    for (int i = 0; i < ARM_DOF; ++i) {
        const matrix::Vector3f jv = joint_axis[i].cross(pe - joint_position[i]);
        J(0, i) = jv(0);
        J(1, i) = jv(1);
        J(2, i) = jv(2);
    }

    return J;
}

bool ArmControl::computeDLSJointStep(const JacobianPos &J, const matrix::Vector3f &error, float dq[ARM_DOF])
{
    if (!PX4_ISFINITE(error.norm())) {
        return false;
    }

    constexpr float lambda = 0.01f;

    matrix::Matrix3f I{};
    I.setIdentity();

    const matrix::Matrix3f A = J * J.transpose() + lambda * lambda * I;
    const auto delta_q = J.transpose() * A.I() * error;

    for (int i = 0; i < ARM_DOF; ++i) {
        if (!PX4_ISFINITE(delta_q(i, 0))) {
            return false;
        }

        dq[i] = delta_q(i, 0);
    }

    return true;
}

IKResult ArmControl::solvePositionIK(const float q_initial[ARM_DOF], const matrix::Vector3f &target_position, float q_solution[ARM_DOF])
{
    constexpr int max_iterations = 80;
    constexpr int max_stagnant_iterations = 8;
    constexpr int max_no_motion_iterations = 5;

    constexpr float tolerance = 1e-4f;
    constexpr float alpha = 0.3f;
    constexpr float max_dq_step = 0.15f;
    constexpr float min_improvement = 1e-5f;
    constexpr float min_joint_motion = 1e-5f;

    IKResult result{IKStatus::MaxIterations, 0, INFINITY};

    float q[ARM_DOF]{};

    for (int i = 0; i < ARM_DOF; ++i) {
        q[i] = q_initial[i];
    }

    float previous_error = INFINITY;
    int stagnant_count = 0;
    int no_motion_count = 0;

    for (int iter = 0; iter < max_iterations; ++iter) {
        const matrix::Vector3f current_position = extractPosition(computeURDFFK(q));
        const matrix::Vector3f error = target_position - current_position;
        const float error_norm = error.norm();

        result.iterations = iter + 1;
        result.final_error = error_norm;

        if (!PX4_ISFINITE(error_norm)) {
            for (int i = 0; i < ARM_DOF; ++i) {
                q_solution[i] = q[i];
            }

            result.status = IKStatus::NumericalFailure;
            result.final_error = INFINITY;
            return result;
        }

        if (error_norm < tolerance) {
            for (int i = 0; i < ARM_DOF; ++i) {
                q_solution[i] = q[i];
            }

            result.status = IKStatus::Success;
            return result;
        }

        if (PX4_ISFINITE(previous_error)) {
            const float improvement = previous_error - error_norm;
            stagnant_count = improvement < min_improvement ? stagnant_count + 1 : 0;
        }

        previous_error = error_norm;

        if (stagnant_count >= max_stagnant_iterations) {
            for (int i = 0; i < ARM_DOF; ++i) {
                q_solution[i] = q[i];
            }

            result.status = IKStatus::Stagnated;
            return result;
        }

        const JacobianPos J = computePositionJacobian(q);
        float dq[ARM_DOF]{};

        if (!computeDLSJointStep(J, error, dq)) {
            for (int i = 0; i < ARM_DOF; ++i) {
                q_solution[i] = q[i];
            }

            result.status = IKStatus::NumericalFailure;
            return result;
        }

        float q_before[ARM_DOF]{};

        for (int i = 0; i < ARM_DOF; ++i) {
            q_before[i] = q[i];
        }

        for (int i = 0; i < ARM_DOF; ++i) {
            const float step = math::constrain(alpha * dq[i], -max_dq_step, max_dq_step);
            q[i] = constrainJointPosition(i, q[i] + step);
        }

        float actual_motion_sq = 0.0f;

        for (int i = 0; i < ARM_DOF; ++i) {
            const float delta = q[i] - q_before[i];
            actual_motion_sq += delta * delta;
        }

        const float actual_motion = sqrtf(actual_motion_sq);
        no_motion_count = actual_motion <= min_joint_motion ? no_motion_count + 1 : 0;

        if (no_motion_count >= max_no_motion_iterations) {
            for (int i = 0; i < ARM_DOF; ++i) {
                q_solution[i] = q[i];
            }

            result.final_error = (target_position - extractPosition(computeURDFFK(q))).norm();
            result.status = IKStatus::Stagnated;
            return result;
        }
    }

    result.final_error = (target_position - extractPosition(computeURDFFK(q))).norm();

    for (int i = 0; i < ARM_DOF; ++i) {
        q_solution[i] = q[i];
    }

    return result;
}

void ArmControl::testForwardKinematics()
{
    float q[ARM_DOF]{};
    const Matrix4f T = computeURDFFK(q);
    const EndEffectorPose pose = extractPose(T);

    PX4_INFO("SO101 zero FK:");
    PX4_INFO("x=%.6f y=%.6f z=%.6f", (double)pose.position(0), (double)pose.position(1), (double)pose.position(2));

    for (int row = 0; row < 4; ++row) {
        PX4_INFO("%.5f %.5f %.5f %.5f", (double)T(row, 0), (double)T(row, 1), (double)T(row, 2), (double)T(row, 3));
    }

    PX4_INFO("quat: w=%.5f x=%.5f y=%.5f z=%.5f", (double)pose.quaternion(0), (double)pose.quaternion(1), (double)pose.quaternion(2), (double)pose.quaternion(3));
}

void ArmControl::testJacobian()
{
    float q[ARM_DOF]{0.3f, -0.4f, 0.5f, -0.2f, 0.6f};
    constexpr float eps = 1e-4f;

    const JacobianPos J = computePositionJacobian(q);
    const matrix::Vector3f pose0 = extractPosition(computeURDFFK(q));

    for (int i = 0; i < ARM_DOF; ++i) {
        float q_eps[ARM_DOF]{};

        for (int j = 0; j < ARM_DOF; ++j) {
            q_eps[j] = q[j];
        }

        q_eps[i] += eps;

        const matrix::Vector3f pose1 = extractPosition(computeURDFFK(q_eps));
        const matrix::Vector3f numerical = (pose1 - pose0) / eps;

        PX4_INFO("joint %d analytic: %.6f %.6f %.6f", i, (double)J(0, i), (double)J(1, i), (double)J(2, i));
        PX4_INFO("joint %d numeric : %.6f %.6f %.6f", i, (double)numerical(0), (double)numerical(1), (double)numerical(2));
    }
}

void ArmControl::testPositionIK()
{
    float q0[ARM_DOF]{};
    matrix::Vector3f target = extractPosition(computeURDFFK(q0));
    target(0) -= 0.1f;
    target(2) += 0.1f;

    float q_solution[ARM_DOF]{};
    const IKResult result = solvePositionIK(q0, target, q_solution);

    PX4_INFO("IK status: %s", ikStatusString(result.status));
    PX4_INFO("iterations: %d", result.iterations);
    PX4_INFO("final error: %.6f", (double)result.final_error);

    for (int i = 0; i < ARM_DOF; ++i) {
        PX4_INFO("q_solution[%d] = %.6f", i, (double)q_solution[i]);
    }

    const matrix::Vector3f start = extractPosition(computeURDFFK(q0));
    const matrix::Vector3f final_pose = extractPosition(computeURDFFK(q_solution));

    PX4_INFO("start : %.6f %.6f %.6f", (double)start(0), (double)start(1), (double)start(2));
    PX4_INFO("target: %.6f %.6f %.6f", (double)target(0), (double)target(1), (double)target(2));
    PX4_INFO("final : %.6f %.6f %.6f", (double)final_pose(0), (double)final_pose(1), (double)final_pose(2));
}

const char *ArmControl::ikStatusString(IKStatus status)
{
    switch (status) {
    case IKStatus::Success:
        return "success";

    case IKStatus::MaxIterations:
        return "max_iterations";

    case IKStatus::Stagnated:
        return "stagnated";

    case IKStatus::NumericalFailure:
        return "numerical_failure";

    default:
        return "unknown";
    }
}

uint8_t ArmControl::convertIkStatus(IKStatus status)
{
    switch (status) {
    case IKStatus::Success:
        return arm_control_status_s::IK_SUCCESS;

    case IKStatus::MaxIterations:
        return arm_control_status_s::IK_MAX_ITERATIONS;

    case IKStatus::Stagnated:
        return arm_control_status_s::IK_STAGNATED;

    case IKStatus::NumericalFailure:
        return arm_control_status_s::IK_NUMERICAL_FAILURE;

    default:
        return arm_control_status_s::IK_NUMERICAL_FAILURE;
    }
}

bool ArmControl::updateIKTarget(const matrix::Vector3f &target_position)
{
    matrix::Vector3f safe_target{};

    if (!sanitizeCartesianTarget(target_position, safe_target)) {
        return false;
    }

    float q_initial[ARM_DOF]{};

    if (_has_last_ik_solution) {
        for (int i = 0; i < ARM_DOF; ++i) {
            q_initial[i] = _last_ik_solution[i];
        }
    } else if (_joint_feedback_valid) {
        for (int i = 0; i < ARM_DOF; ++i) {
            q_initial[i] = _latest_joint_status.position[i];
        }
    } else {
        for (int i = 0; i < ARM_DOF; ++i) {
            q_initial[i] = _command_position[i];
        }
    }

    float q_solution[ARM_DOF]{};
    const IKResult result = solvePositionIK(q_initial, safe_target, q_solution);
    _last_ik_result = result;

    if (result.status != IKStatus::Success) {
        return false;
    }

    for (int i = 0; i < ARM_DOF; ++i) {
        _last_ik_solution[i] = q_solution[i];
        _joint_target[i] = q_solution[i];
    }

    _has_last_ik_solution = true;
    _ee_target_position = safe_target;
    _ee_target_valid = true;
    return true;
}

void ArmControl::testWarmStart()
{
    // Development-only test: modifies the module's internal IK state.
    _has_last_ik_solution = false;
    _ee_target_valid = false;

    for (int i = 0; i < ARM_DOF; ++i) {
        _command_position[i] = 0.0f;
    }

    float q0[ARM_DOF]{};
    matrix::Vector3f target1 = extractPosition(computeURDFFK(q0));
    target1(0) -= 0.02f;
    target1(2) += 0.02f;

    if (!updateIKTarget(target1)) {
        PX4_ERR("warm start test: target1 failed");
        return;
    }

    const int iter1 = _last_ik_result.iterations;

    matrix::Vector3f target2 = target1;
    target2(0) -= 0.01f;

    if (!updateIKTarget(target2)) {
        PX4_ERR("warm start test: target2 failed");
        return;
    }

    const int iter2 = _last_ik_result.iterations;

    PX4_INFO("warm start test: iter1=%d iter2=%d", iter1, iter2);

    for (int i = 0; i < ARM_DOF; ++i) {
        PX4_INFO("q[%d]=%.6f", i, (double)_last_ik_solution[i]);
    }
}

bool ArmControl::sanitizeCartesianTarget(const matrix::Vector3f &raw_target, matrix::Vector3f &safe_target)
{
    if (!PX4_ISFINITE(raw_target(0)) || !PX4_ISFINITE(raw_target(1)) || !PX4_ISFINITE(raw_target(2))) {
        return false;
    }

    matrix::Vector3f target_bounded{};
    target_bounded(0) = math::constrain(raw_target(0), _cartesian_limit.min_x, _cartesian_limit.max_x);
    target_bounded(1) = math::constrain(raw_target(1), _cartesian_limit.min_y, _cartesian_limit.max_y);
    target_bounded(2) = math::constrain(raw_target(2), _cartesian_limit.min_z, _cartesian_limit.max_z);

    if (!_ee_target_valid) {
        safe_target = target_bounded;
        return true;
    }

    matrix::Vector3f delta = target_bounded - _ee_target_position;
    const float delta_norm = delta.norm();

    if (!PX4_ISFINITE(delta_norm)) {
        return false;
    }

    if (delta_norm > _cartesian_limit.max_step && delta_norm > 1e-6f) {
        delta *= _cartesian_limit.max_step / delta_norm;
    }

    safe_target = _ee_target_position + delta;
    return true;
}

void ArmControl::testCartesianIKPipeline()
{
    // Development-only test: modifies the module's internal IK state.
    _has_last_ik_solution = false;
    _ee_target_valid = false;

    for (int i = 0; i < ARM_DOF; ++i) {
        _command_position[i] = 0.0f;
        _last_ik_solution[i] = 0.0f;
    }

    float q0[ARM_DOF]{};
    matrix::Vector3f target1 = extractPosition(computeURDFFK(q0));
    target1(0) -= 0.02f;
    target1(2) += 0.02f;

    const bool ok1 = updateIKTarget(target1);

    PX4_INFO("target1: ok=%d iter=%d err=%.6f", ok1, _last_ik_result.iterations, (double)_last_ik_result.final_error);

    if (!ok1) {
        PX4_ERR("target1 failed");
        return;
    }

    PX4_INFO("safe1: %.6f %.6f %.6f", (double)_ee_target_position(0), (double)_ee_target_position(1), (double)_ee_target_position(2));

    matrix::Vector3f target2 = target1;
    target2(0) += 0.20f;

    const bool ok2 = updateIKTarget(target2);

    PX4_INFO("target2: ok=%d iter=%d err=%.6f", ok2, _last_ik_result.iterations, (double)_last_ik_result.final_error);
    PX4_INFO("raw2 : %.6f %.6f %.6f", (double)target2(0), (double)target2(1), (double)target2(2));
    PX4_INFO("safe2: %.6f %.6f %.6f", (double)_ee_target_position(0), (double)_ee_target_position(1), (double)_ee_target_position(2));

    for (int i = 0; i < ARM_DOF; ++i) {
        PX4_INFO("joint_target[%d] = %.6f", i, (double)_joint_target[i]);
    }
}

void ArmControl::updateControlState()
{
    if (!_command_initialized) {
        _control_state = ArmControlState::Uninitialized;
        return;
    }

    constexpr hrt_abstime feedback_timeout = 1000_ms;

    if (_last_feedback_time != 0 && hrt_elapsed_time(&_last_feedback_time) > feedback_timeout) {
        _joint_feedback_valid = false;
        _control_state = ArmControlState::Fault;
        return;
    }

    _control_state = _ee_target_valid ? ArmControlState::Active : ArmControlState::Ready;
}

void ArmControl::publishControlStatus()
{
    arm_control_status_s msg{};
    msg.timestamp = hrt_absolute_time();

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        msg.joint_target[i] = _joint_target[i];
        msg.joint_command[i] = _command_position[i];
    }

    for (int i = 0; i < 3; ++i) {
        msg.ee_target[i] = _ee_target_position(i);
        msg.ee_command[i] = _command_ee_pose.position(i);
        msg.ee_measured[i] = _measured_ee_pose.position(i);
    }

    msg.ik_status = convertIkStatus(_last_ik_result.status);
    msg.ik_error = _last_ik_result.final_error;
    msg.feedback_valid = _joint_feedback_valid;
    msg.command_initialized = _command_initialized;
    msg.state = static_cast<uint8_t>(_control_state);

    _arm_control_status_pub.publish(msg);
}

extern "C" __EXPORT int arm_control_main(int argc, char *argv[])
{
    return ArmControl::main(argc, argv);
}
