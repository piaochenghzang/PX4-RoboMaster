/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "SMS_STS.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <uORB/Subscription.hpp>
#include <uORB/topics/arm_joint_command.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/arm_joint_status.h>
using namespace time_literals;
enum class PendingCommand : uint8_t {
    None = 0,
    Ping,
    Info
};
//校准定义结构体
struct JointCalibration {
    uint8_t servo_id;
    int32_t range_min;
    int32_t range_max;
    int32_t homing_offset;
    bool reversed;
};

class FTServo final : public ModuleBase<FTServo>
{
public:
	FTServo(const char *device, uint32_t baudrate);
	~FTServo() override = default;

	static int task_spawn(int argc, char *argv[]);
	static FTServo *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int32_t unnormalizePosition(int joint_index, float rad);
	float normalizePosition(int joint_index, int32_t raw);

	int16_t unnormalizeVelocity(int joint_index, float rad_s);
	float normalizeVelocity(int joint_index, int16_t raw_speed);
	int print_status() override;
	void run() override;

private:
	static constexpr const char *DEFAULT_DEVICE{"/dev/ttyS5"};
	static constexpr uint32_t DEFAULT_BAUDRATE{1'000'000};
	static constexpr size_t DEVICE_PATH_LENGTH{32};
	static constexpr int JOINT_COUNT = 6;

	JointCalibration _calibration[JOINT_COUNT] {
		{1, 0, 4095, 0, false},
		{2, 0, 4095, 0, false},
		{3, 0, 4095, 0, false},
		{4, 0, 4095, 0, false},
		{5, 0, 4095, 0, false},
		{6, 0, 4095, 0, false},
	};

	// Subscriptions
	uORB::Subscription                 _arm_joint_command_sub{ORB_ID(arm_joint_command)};

	//Publication
	uORB::Publication<arm_joint_status_s> _arm_joint_status_pub{ORB_ID(arm_joint_status)};

	SMS_STS _servo{};
	char _device[DEVICE_PATH_LENGTH]{};
	uint32_t _baudrate{DEFAULT_BAUDRATE};
	bool _port_open{false};
	PendingCommand _pending_command{PendingCommand::None};
	int _pending_id{-1};

	arm_joint_command_s _latest_command{};
	bool _has_command{false};

	int32_t _raw_position[JOINT_COUNT]{};
};

FTServo::FTServo(const char *device, uint32_t baudrate) :
	_baudrate(baudrate)
{
	strncpy(_device, device, sizeof(_device) - 1);
	_device[sizeof(_device) - 1] = '\0';
}

int FTServo::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("ftservo",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return PX4_OK;
}

FTServo *FTServo::instantiate(int argc, char *argv[])
{
	const char *device = DEFAULT_DEVICE;
	uint32_t baudrate = DEFAULT_BAUDRATE;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch;

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'b': {
			char *end = nullptr;
			const unsigned long parsed = strtoul(myoptarg, &end, 10);

			if (end == myoptarg || *end != '\0' || parsed == 0 || parsed > 4'000'000UL) {
				PX4_ERR("invalid baudrate: %s", myoptarg);
				return nullptr;
			}

			baudrate = static_cast<uint32_t>(parsed);
			break;
		}

		default:
			return nullptr;
		}
	}

	if (device == nullptr || strlen(device) >= DEVICE_PATH_LENGTH) {
		PX4_ERR("invalid serial device path");
		return nullptr;
	}

	return new FTServo(device, baudrate);
}

int FTServo::custom_command(int argc, char *argv[])
{
	if(!strcmp(argv[0], "ping"))
	{
		if (argc < 2)
		{
			PX4_ERR("missing servo ID");
        	return PX4_ERROR;
		}
		
		int id = atoi(argv[1]);//获取id
		if(id < 0 || id > 255)
		{
			PX4_ERR("invalid servo ID: %d", id);
			return PX4_ERROR;
		}

		FTServo *instance = get_instance();
		if (instance == nullptr)
		{
			PX4_ERR("device not running");
			return PX4_ERROR;
		}

		instance->_pending_id = id;
		instance->_pending_command = PendingCommand::Ping;
		PX4_INFO("ping request: servo %d", id);
        return PX4_OK;
	}

	if (argc > 0 && !strcmp(argv[0], "read")) {

        if (argc < 2) {
            PX4_ERR("missing servo ID");
            return PX4_ERROR;
        }

        int id = atoi(argv[1]);

        if (id < 0 || id > 253) {
            PX4_ERR("invalid servo ID: %d", id);
            return PX4_ERROR;
        }

        FTServo *instance = get_instance();

        if (instance == nullptr) {
            PX4_ERR("device not running");
            return PX4_ERROR;
        }

        instance->_pending_id = id;
        instance->_pending_command = PendingCommand::Info;

        PX4_INFO("info request: servo %d", id);
        return PX4_OK;
    }
	return print_usage("unknown command");
}

int FTServo::print_status()
{
	PX4_INFO("%s", _port_open ? "running" : "starting");
	PX4_INFO("device: %s", _device);
	PX4_INFO("baudrate: %lu", static_cast<unsigned long>(_baudrate));
	PX4_INFO("servo commands: disabled");

	if (_has_command) {
    PX4_INFO("latest arm command:");
	const int count = _latest_command.joint_count < JOINT_COUNT ? _latest_command.joint_count : JOINT_COUNT;
    for (int i = 0; i < count; ++i) {
			PX4_INFO("joint[%d]: rad=%.3f raw=%ld",
					i,
					static_cast<double>(_latest_command.position[i]),
            		static_cast<long>(_raw_position[i]));
			const float recovered_rad = normalizePosition(i, _raw_position[i]);

			PX4_INFO(
				"joint[%d]: cmd=%.3f raw=%ld recovered=%.3f",
				i,
				static_cast<double>(_latest_command.position[i]),
				static_cast<long>(_raw_position[i]),
				static_cast<double>(recovered_rad));
		}
	}
	return PX4_OK;
}

void FTServo::run()
{
	if (!_servo.begin(_baudrate, _device)) {
		PX4_ERR("failed to initialize FT servo serial port");
		return;
	}

	_port_open = true;
	PX4_INFO("serial transport ready");

	while (!should_exit()) {
		if(_pending_command != PendingCommand::None)
		{
			const PendingCommand command = _pending_command;
			const int id = _pending_id;

			_pending_command = PendingCommand::None;
			_pending_id = -1;

			switch (command)
			{
			case PendingCommand::Ping:{
				const int ping_result = _servo.Ping(id);
				if(ping_result == id)
				{
					PX4_INFO("servo %d ping OK", id);
				} else {
					PX4_ERR("servo %d ping failed, error=%u",
                        id,
                        static_cast<unsigned>(_servo.getLastError()));
				}
				break;
			}
			case PendingCommand::Info:{
				const int result = _servo.FeedBack(id);
				if (result < 0) {
                	PX4_ERR("servo %d feedback failed, error=%u",
                        id,
                        static_cast<unsigned>(_servo.getLastError()));
					break;
				}

				const int position    = _servo.ReadPos(-1);
				const int speed       = _servo.ReadSpeed(-1);
				const int load        = _servo.ReadLoad(-1);
				const int voltage     = _servo.ReadVoltage(-1);
				const int temperature = _servo.ReadTemper(-1);
				const int moving      = _servo.ReadMove(-1);
				const int current     = _servo.ReadCurrent(-1);

				PX4_INFO("servo %d:", id);
				PX4_INFO("  position:    %d", position);
				PX4_INFO("  speed:       %d", speed);
				PX4_INFO("  load:        %d", load);
				PX4_INFO("  voltage:     %d", voltage);
				PX4_INFO("  temperature: %d", temperature);
				PX4_INFO("  moving:      %d", moving);
				PX4_INFO("  current:     %d", current);

				break;
			}
			case PendingCommand::None:
			default:
				break;
			}
		}

		arm_joint_command_s cmd{};

		if (_arm_joint_command_sub.update(&cmd)) {
			_latest_command = cmd;
    		_has_command = true;

			arm_joint_status_s status{};
			status.timestamp = hrt_absolute_time();
			const int count = cmd.joint_count < JOINT_COUNT ? cmd.joint_count : JOINT_COUNT;
			status.joint_count = count;

			for (int i = 0; i < count; ++i) {
				_raw_position[i] = unnormalizePosition(i, cmd.position[i]);
				status.position[i] = normalizePosition(i, _raw_position[i]);
				status.velocity[i] = 0.0f;
				status.current[i] = 0.0f;
			}
			_arm_joint_status_pub.publish(status);
		}
		
		px4_usleep(20'000);
	}
	_port_open = false;
	_servo.end();
}

int FTServo::print_usage(const char *reason)
{
	if (reason != nullptr) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Minimal serial transport module for FEETECH SMS/STS servos.
The module opens the configured UART and performs a one-shot
ping test to servo ID 1. Motion commands and torque control
are not enabled yet.

### Example
$ ftservo start -d /dev/ttyS5 -b 1000000
$ ftservo ping id
$ ftservo info id
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ftservo", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', DEFAULT_DEVICE, "<file:dev>", "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', DEFAULT_BAUDRATE, 1, 4'000'000, "Baudrate", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND("ping");
	PRINT_MODULE_USAGE_COMMAND("read");
	return PX4_OK;
}
//(raw =q\frac{4095}{2\pi}+mid\)
int32_t FTServo::unnormalizePosition(int joint_index, float rad)
{
	const JointCalibration &cal = _calibration[joint_index];
	float joint_rad = rad;

	if (cal.reversed)
	{
		joint_rad = -joint_rad;
	}
	const float mid = 0.5f * static_cast<float>(cal.range_min + cal.range_max);
	constexpr float TWO_PI = 6.28318530717958647692f;
    constexpr float MAX_RES = 4095.0f;

    float raw = joint_rad * MAX_RES / TWO_PI + mid;

    if (raw < cal.range_min) {
        raw = static_cast<float>(cal.range_min);
    }

    if (raw > cal.range_max) {
        raw = static_cast<float>(cal.range_max);
    }

    return static_cast<int32_t>(raw);
	
}

float FTServo::normalizePosition(int joint_index, int32_t raw)
{
	const JointCalibration &cal = _calibration[joint_index];
	const float mid = 0.5f * static_cast<float>(cal.range_min + cal.range_max);
	constexpr float TWO_PI = 6.28318530717958647692f;
    constexpr float MAX_RES = 4095.0f;

	float rad = (static_cast<float>(raw) - mid) * TWO_PI / MAX_RES;
    if (cal.reversed) {
        rad = -rad;
    }
    return rad;
}

int16_t FTServo::unnormalizeVelocity(int joint_index, float rad_s)
{
	constexpr float TWO_PI = 6.28318530717958647692f;
    constexpr float RESOLUTION = 4096.0f;

    float velocity = rad_s;

    if (_calibration[joint_index].reversed) {
        velocity = -velocity;
    }

    float raw =
        velocity * RESOLUTION / TWO_PI;

    return static_cast<int16_t>(raw);
}

float FTServo::normalizeVelocity(int joint_index, int16_t raw_speed)
{
	constexpr float TWO_PI = 6.28318530717958647692f;
    constexpr float RESOLUTION = 4096.0f;

    float velocity =
        static_cast<float>(raw_speed)
        * TWO_PI / RESOLUTION;

    if (_calibration[joint_index].reversed) {
        velocity = -velocity;
    }

    return velocity;
}

extern "C" __EXPORT int ftservo_main(int argc, char *argv[])
{
	return FTServo::main(argc, argv);
}
