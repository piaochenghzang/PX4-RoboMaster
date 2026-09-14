/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

/**
 * @file IST8310_registers.hpp
 *
 * IST8310 registers.
 *
 */

#pragma once

#include <cstdint>

namespace IST8310
{
// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint32_t I2C_SPEED = 400 * 1000; // 400 kHz I2C serial interface
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x0E;

static constexpr uint8_t Device_ID = 0x10; // Device ID of IST8310

enum class Register : uint8_t {
	WIA   = 0x00, // Device ID

	ST1   = 0x02, // Status 1
	HXL   = 0x03,
	HXH   = 0x04,
	HYL   = 0x05,
	HYH   = 0x06,
	HZL   = 0x07,
	HZH   = 0x08,
	ST2   = 0x09, // Status 2
	CNTL1 = 0x0A, // Control 1
	CNTL2 = 0x0B, // Control 2
	STR   = 0x0C, // Self-test register
	CNTL3 = 0x0D, // Output resolution

	TEMPL   = 0x1C, // Temperature output, low byte
	TEMPH   = 0x1D, // Temperature output, high byte

	AVGCNTL = 0x41, // Internal averaging control
	PDCNTL  = 0x42, // Set/reset pulse-duration control
};

// ST1
enum ST1_BIT : uint8_t {
	DRDY = Bit0, // Measurement data ready
	DOR  = Bit1, // Data overrun
};

// ST2
enum ST2_BIT : uint8_t {
	INT = Bit3, // Magnetic-field interrupt flag
};

// CNTL1
enum CNTL1_BIT : uint8_t {
	// MODE[3:0]: Operation mode setting
	MODE_MASK = Bit3 | Bit2 | Bit1 | Bit0,

	STANDBY_MODE           = 0,
	SINGLE_MEASUREMENT_MODE = Bit0,
};

// CNTL2
enum CNTL2_BIT : uint8_t {
	SRST = Bit0, // Reset

	// DRDY pin polarity:
	// 0: active low
	// 1: active high
	DRP = Bit2,

	// DRDY output enable
	DREN = Bit3,
};

// CNTL3
enum CNTL3_BIT : uint8_t {
	X_16BIT = Bit4,
	Y_16BIT = Bit5,
	Z_16BIT = Bit6,
	XYZ_16BIT = X_16BIT | Y_16BIT | Z_16BIT,
};
//STR
enum STR_BIT : uint8_t {
	SELF_TEST = Bit6,
};

/**
 * AVGCNTL register fields.
 *
 * Bits [5:3]: Y-axis averaging
 * Bits [2:0]: X-axis and Z-axis averaging
 */
enum AVGCNTL_BIT : uint8_t {
	XZ_AVERAGE_NONE = 0,
	XZ_AVERAGE_2    = Bit0,
	XZ_AVERAGE_4    = Bit1,
	XZ_AVERAGE_8    = Bit1 | Bit0,
	XZ_AVERAGE_16   = Bit2,

	Y_AVERAGE_NONE  = 0,
	Y_AVERAGE_2     = Bit3,
	Y_AVERAGE_4     = Bit4,
	Y_AVERAGE_8     = Bit4 | Bit3,
	Y_AVERAGE_16    = Bit5,

	// Y = 16 times, X/Z = 16 times
	// 0x20 | 0x04 = 0x24
	AVERAGE_16_ALL  = Y_AVERAGE_16 | XZ_AVERAGE_16,
};

/**
 * PDCNTL register fields.
 *
 * Bits [7:6] select the set/reset pulse duration.
 */
enum PDCNTL_BIT : uint8_t {
	PULSE_DURATION_LONG   = Bit6,        // 01b, 0x40
	PULSE_DURATION_NORMAL = Bit7 | Bit6, // 11b, 0xC0, recommended
};

} // namespace IST8310
