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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/arm_cartesian_setpoint.h>
#include <uORB/topics/arm_control_status.h>
#include <uORB/topics/arm_joint_command.h>
#include <uORB/topics/arm_joint_status.h>

#include <matrix/matrix/math.hpp>

using namespace time_literals;

using Matrix4f = matrix::Matrix<float, 4, 4>;

static constexpr int ARM_DOF = 5;
static constexpr int MOTOR_COUNT = 6;
static constexpr int GRIPPER_INDEX = ARM_DOF;

using JacobianPos = matrix::Matrix<float, 3, ARM_DOF>;

struct JointLimit {
    float min_rad;
    float max_rad;
    float max_velocity;
    float max_acceleration;
};

struct JointOrigin {
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};

struct EndEffectorPose {
    matrix::Vector3f position{};
    matrix::Dcmf rotation{};
    matrix::Quatf quaternion{};
};

enum class IKStatus {
    Success,
    MaxIterations,
    Stagnated,
    NumericalFailure,
};

struct IKResult {
    IKStatus status;
    int iterations;
    float final_error;
};

struct CartesianLimit {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    float max_step;
};

enum class ArmControlState : uint8_t {
    Uninitialized = 0,
    Ready,
    Active,
    Fault
};

class ArmControl : public ModuleBase<ArmControl>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    ArmControl();
    ~ArmControl() override;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();
    int print_status() override;

private:
    // Main loop
    void Run() override;
    float updateDt();
    void pollJointFeedback();
    void processCartesianSetpoint();
    void generateAndPublishJointCommand(float dt);

    // Feedback
    bool updateJointFeedback(const arm_joint_status_s &status);

    // Joint trajectory
    float constrainJointPosition(int joint, float target_rad);
    void updateJointTrajectory(int joint, float target, float dt);

    // Forward kinematics
    static Matrix4f makeIdentityTransform();
    static Matrix4f makeRPYTransform(float x, float y, float z, float roll, float pitch, float yaw);
    static Matrix4f makeTranslation(float x, float y, float z);
    static Matrix4f makeRotationX(float roll);
    static Matrix4f makeRotationY(float pitch);
    static Matrix4f makeRotationZ(float yaw);
    Matrix4f computeURDFFK(const float q[ARM_DOF]) const;
    static EndEffectorPose extractPose(const Matrix4f &T);
    static matrix::Vector3f extractPosition(const Matrix4f &T);

    // Jacobian / IK
    JacobianPos computePositionJacobian(const float q[ARM_DOF]) const;
    static bool computeDLSJointStep(const JacobianPos &J, const matrix::Vector3f &error, float dq[ARM_DOF]);
    IKResult solvePositionIK(const float q_initial[ARM_DOF], const matrix::Vector3f &target_position, float q_solution[ARM_DOF]);
    bool updateIKTarget(const matrix::Vector3f &target_position);
    static const char *ikStatusString(IKStatus status);
    static uint8_t convertIkStatus(IKStatus status);

    // Cartesian safety
    bool sanitizeCartesianTarget(const matrix::Vector3f &raw_target, matrix::Vector3f &safe_target);

    // State / status
    void updateControlState();
    void publishControlStatus();

    // Development tests
    void testForwardKinematics();
    void testJacobian();
    void testPositionIK();
    void testWarmStart();
    void testCartesianIKPipeline();

private:
    // Publications / subscriptions
    uORB::Publication<arm_joint_command_s> _arm_joint_command_pub{ORB_ID(arm_joint_command)};
    uORB::Publication<arm_control_status_s> _arm_control_status_pub{ORB_ID(arm_control_status)};
    uORB::Subscription _arm_joint_status{ORB_ID(arm_joint_status)};
    uORB::Subscription _arm_cartesian_setpoint{ORB_ID(arm_cartesian_setpoint)};

    // Runtime
    uint32_t _run_count{0};
    hrt_abstime _last_run{0};
    hrt_abstime _last_feedback_time{0};

    // Feedback state
    arm_joint_status_s _latest_joint_status{};
    bool _joint_feedback_valid{false};
    EndEffectorPose _measured_ee_pose{};

    // Command state
    float _command_position[MOTOR_COUNT]{};
    float _command_velocity[MOTOR_COUNT]{};
    float _joint_target[MOTOR_COUNT]{};
    bool _command_initialized{false};
    EndEffectorPose _command_ee_pose{};

    // Joint limits
    JointLimit _joint_limits[MOTOR_COUNT] {
        {-1.57f, 1.57f, 1.0f, 2.0f},
        {-1.20f, 1.20f, 1.0f, 2.0f},
        {-1.50f, 1.50f, 1.0f, 2.0f},
        {-1.57f, 1.57f, 1.5f, 3.0f},
        {-1.57f, 1.57f, 1.5f, 3.0f},
        {-1.00f, 1.00f, 1.0f, 2.0f},
    };

    // SO101 kinematic model
    static constexpr float PI = 3.14159265358979323846f;
    static constexpr float HALF_PI = 1.57079632679489661923f;

    JointOrigin _joint_origins[ARM_DOF] {
        {0.0388353f, 0.0f, 0.0624f, PI, 0.0f, -PI},
        {-0.0303992f, -0.0182778f, -0.0542f, -HALF_PI, -HALF_PI, 0.0f},
        {-0.11257f, -0.028f, 0.0f, 0.0f, 0.0f, HALF_PI},
        {-0.1349f, 0.0052f, 0.0f, 0.0f, 0.0f, -HALF_PI},
        {0.0f, -0.0611f, 0.0181f, HALF_PI, 0.0486795f, PI},
    };

    Matrix4f _joint_origin_tf[ARM_DOF]{};
    Matrix4f _tool_tf{};

    // IK state
    float _last_ik_solution[ARM_DOF]{};
    bool _has_last_ik_solution{false};
    IKResult _last_ik_result{};

    // Cartesian target
    matrix::Vector3f _ee_target_position{};
    bool _ee_target_valid{false};

    CartesianLimit _cartesian_limit {
        0.05f, 0.40f,
        -0.25f, 0.25f,
        0.02f, 0.40f,
        0.02f
    };

    // Control state
    ArmControlState _control_state{ArmControlState::Uninitialized};
};
