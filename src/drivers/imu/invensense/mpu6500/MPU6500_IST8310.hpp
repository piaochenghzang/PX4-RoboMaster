/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

/**
 * IST8310 connected to the MPU6500 auxiliary I2C master.
 */

#pragma once

#include "IST8310_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class MPU6500;

namespace IST8310
{

class MPU6500_IST8310 : public px4::ScheduledWorkItem
{
public:
	MPU6500_IST8310(MPU6500 &mpu6500, enum Rotation rotation = ROTATION_NONE);
	~MPU6500_IST8310() override;

	bool Reset();
	void PrintInfo();

private:
	struct TransferBuffer {
		uint8_t HXL;
		uint8_t HXH;
		uint8_t HYL;
		uint8_t HYH;
		uint8_t HZL;
		uint8_t HZH;
	};

	struct register_config_t {
		IST8310::Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	void Run() override;

	MPU6500 &_mpu6500;

	PX4Magnetometer _px4_mag;
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME"_ist8310: bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME"_ist8310: bad transfer")};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	int _failure_count{0};

	enum class STATE : uint8_t {
		RESET,
		SETUP_WHO_AM_I,
		READ_WHO_AM_I,
		CONFIGURE_CNTL1,
		CONFIGURE_CNTL2,
		CONFIGURE_CNTL3,
		CONFIGURE_AVGCNTL,
		CONFIGURE_PDCNTL,
		MEASURE,
		SETUP_READ,
		READ,
	} _state{STATE::RESET};
};

} // namespace IST8310
