#pragma once

#include <uORB/Subscription.hpp>
#include <uORB/topics/arm_joint_command.h>

class MavlinkStreamArmJointCommand : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamArmJointCommand(mavlink);
    }

    const char *get_name() const override
    {
        return get_name_static();
    }

    static constexpr const char *get_name_static()
    {
        return "ARM_JOINT_COMMAND";
    }

    static constexpr uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_ARM_JOINT_COMMAND;
    }

    uint16_t get_id() override
    {
        return get_id_static();
    }

    unsigned get_size() override
    {
        return MAVLINK_MSG_ID_ARM_JOINT_COMMAND_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    explicit MavlinkStreamArmJointCommand(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _arm_joint_command_sub{ORB_ID(arm_joint_command)};

    bool send() override
    {
        arm_joint_command_s command{};

        if (!_arm_joint_command_sub.update(&command)) {
            return false;
        }

        mavlink_arm_joint_command_t message{};

        for (int i = 0; i < 6; ++i) {
            message.position[i] = command.position[i];
            message.velocity[i] = command.velocity[i];
        }

        mavlink_msg_arm_joint_command_send_struct(_mavlink->get_channel(), &message);
        return true;
    }
};
