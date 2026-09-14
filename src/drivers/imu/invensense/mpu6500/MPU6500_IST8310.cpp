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

#include "MPU6500_IST8310.hpp"

#include "MPU6500.hpp"

using namespace time_literals;

namespace IST8310
{

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (static_cast<uint16_t>(msb) << 8u) | lsb;
}

MPU6500_IST8310::MPU6500_IST8310(MPU6500 &mpu6500, enum Rotation rotation) :
	ScheduledWorkItem("mpu6500_ist8310", px4::device_bus_to_wq(mpu6500.get_device_id())),
	_mpu6500(mpu6500),
	_px4_mag(mpu6500.get_device_id(), rotation)
{
	_px4_mag.set_device_type(DRV_MAG_DEVTYPE_IST8310);
	_px4_mag.set_scale(1.f / 1320.f);
}

MPU6500_IST8310::~MPU6500_IST8310()
{
	ScheduleClear();
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

bool MPU6500_IST8310::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void MPU6500_IST8310::PrintInfo()
{
	perf_print_counter(_bad_transfer_perf);
}

void MPU6500_IST8310::Run()
{
	switch (_state) {
	case STATE::RESET:
		// CNTL2 SRST: Soft reset
		_mpu6500.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT, static_cast<uint8_t>(Register::CNTL2), CNTL2_BIT::SRST);
		_reset_timestamp = hrt_absolute_time();
		_failure_count = 0;
		_state = STATE::SETUP_WHO_AM_I;
		ScheduleDelayed(50_ms);
		break;

	case STATE::SETUP_WHO_AM_I:
		_mpu6500.I2CSlaveExternalSensorDataEnable(I2C_ADDRESS_DEFAULT,
				static_cast<uint8_t>(Register::WIA), 1);
		_state = STATE::READ_WHO_AM_I;
		ScheduleDelayed(10_ms);
		break;

	case STATE::READ_WHO_AM_I: {
		uint8_t who_am_i{0};
		const bool transfer_ok = _mpu6500.I2CSlaveExternalSensorDataRead(&who_am_i, 1);

		if (transfer_ok && who_am_i == Device_ID) {
			_state = STATE::CONFIGURE_CNTL1;
			ScheduleNow();

		} else if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
			PX4_WARN("IST8310 WIA failed (0x%02x), resetting", who_am_i);
			_state = STATE::RESET;
			ScheduleDelayed(100_ms);

		} else {
			_state = STATE::SETUP_WHO_AM_I;
			ScheduleDelayed(20_ms);
		}
		break;
	}

	case STATE::CONFIGURE_CNTL1:
		// Standby mode is required while changing configuration registers.
		_mpu6500.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT,
				static_cast<uint8_t>(Register::CNTL1), CNTL1_BIT::STANDBY_MODE);
		_state = STATE::CONFIGURE_CNTL2;
		ScheduleDelayed(10_ms);
		break;

	case STATE::CONFIGURE_CNTL2:
		_mpu6500.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT,
				static_cast<uint8_t>(Register::CNTL2), 0);
		_state = STATE::CONFIGURE_CNTL3;
		ScheduleDelayed(10_ms);
		break;

	case STATE::CONFIGURE_CNTL3:
		_mpu6500.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT,
				static_cast<uint8_t>(Register::CNTL3), CNTL3_BIT::XYZ_16BIT);
		_state = STATE::CONFIGURE_AVGCNTL;
		ScheduleDelayed(10_ms);
		break;

	case STATE::CONFIGURE_AVGCNTL:
		_mpu6500.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT,
				static_cast<uint8_t>(Register::AVGCNTL), AVGCNTL_BIT::AVERAGE_16_ALL);
		_state = STATE::CONFIGURE_PDCNTL;
		ScheduleDelayed(10_ms);
		break;

	case STATE::CONFIGURE_PDCNTL:
		_mpu6500.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT,
				static_cast<uint8_t>(Register::PDCNTL), PDCNTL_BIT::PULSE_DURATION_NORMAL);
		_state = STATE::MEASURE;
		ScheduleDelayed(10_ms);
		break;

	case STATE::MEASURE:
		_mpu6500.I2CSlaveExternalSensorDataDisable();
		_mpu6500.I2CSlaveRegisterWrite(I2C_ADDRESS_DEFAULT,
				static_cast<uint8_t>(Register::CNTL1), CNTL1_BIT::SINGLE_MEASUREMENT_MODE);
		_state = STATE::SETUP_READ;
		ScheduleDelayed(10_ms);
		break;

	case STATE::SETUP_READ:
		_mpu6500.I2CSlaveExternalSensorDataEnable(I2C_ADDRESS_DEFAULT,
				static_cast<uint8_t>(Register::HXL), sizeof(TransferBuffer));
		_state = STATE::READ;
		ScheduleDelayed(2_ms);
		break;

	case STATE::READ: {
		TransferBuffer buffer{};
		const hrt_abstime timestamp_sample = hrt_absolute_time();
		const bool transfer_ok = _mpu6500.I2CSlaveExternalSensorDataRead(
				reinterpret_cast<uint8_t *>(&buffer), sizeof(buffer));

		_mpu6500.I2CSlaveExternalSensorDataDisable();

		if (transfer_ok) {
			const int16_t x = combine(buffer.HXH, buffer.HXL);
			const int16_t y = combine(buffer.HYH, buffer.HYL);
			int16_t z = combine(buffer.HZH, buffer.HZL);

			if ((x == 0) && (y == 0) && (z == 0)) {
				perf_count(_bad_transfer_perf);
				_failure_count++;
				_state = STATE::MEASURE;
				ScheduleDelayed(20_ms);
				return;
			}

			// IST8310 frame is +X forward, +Y right, +Z up.
			// PX4 sensor convention uses +Z down.
			z = (z == INT16_MIN) ? INT16_MAX : -z;

			_px4_mag.set_error_count(perf_event_count(_bad_transfer_perf));
			_px4_mag.update(timestamp_sample, x, y, z);

			if (_failure_count > 0) {
				_failure_count--;
			}

		} else {
			perf_count(_bad_transfer_perf);
			_failure_count++;

			if (_failure_count > 10) {
				Reset();
				return;
			}

			_state = STATE::MEASURE;
			ScheduleDelayed(20_ms);
			return;
		}

		_state = STATE::MEASURE;
		ScheduleDelayed(8_ms);
		break;
	}
	}
}

} // namespace IST8310
