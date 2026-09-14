/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
 * @file InvenSense_MPU6500_registers.hpp
 *
 * Invensense MPU6500 registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace InvenSense_MPU6500
{
static constexpr uint32_t SPI_SPEED = 1 * 1000 * 1000;
static constexpr uint32_t SPI_SPEED_SENSOR = 10 * 1000 * 1000; // 20MHz for reading sensor and interrupt registers
static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHOAMI = 0x70;

static constexpr float TEMPERATURE_SENSITIVITY = 333.87f; // LSB/C
static constexpr float TEMPERATURE_OFFSET = 21.f; // C

enum class Register : uint8_t {

	CONFIG             = 0x1A,
	GYRO_CONFIG        = 0x1B,
	ACCEL_CONFIG       = 0x1C,
	ACCEL_CONFIG2      = 0x1D,

	FIFO_EN            = 0x23,
	I2C_MST_CTRL       = 0x24,
	I2C_SLV0_ADDR      = 0x25,
	I2C_SLV0_REG       = 0x26,
	I2C_SLV0_CTRL      = 0x27,
	I2C_SLV1_ADDR      = 0x28,
	I2C_SLV1_REG       = 0x29,
	I2C_SLV1_CTRL      = 0x2A,
	I2C_SLV2_ADDR      = 0x2B,
	I2C_SLV2_REG       = 0x2C,
	I2C_SLV2_CTRL      = 0x2D,
	I2C_SLV3_ADDR      = 0x2E,
	I2C_SLV3_REG       = 0x2F,
	I2C_SLV3_CTRL      = 0x30,
	I2C_SLV4_ADDR      = 0x31,
	I2C_SLV4_REG       = 0x32,
	I2C_SLV4_DO        = 0x33,
	I2C_SLV4_CTRL      = 0x34,
	I2C_MST_STATUS     = 0x36,

	INT_PIN_CFG        = 0x37,
	INT_ENABLE         = 0x38,

	TEMP_OUT_H         = 0x41,
	TEMP_OUT_L         = 0x42,
	EXT_SENS_DATA_00   = 0x49,
	I2C_SLV0_DO        = 0x63,
	I2C_SLV1_DO        = 0x64,
	I2C_SLV2_DO        = 0x65,
	I2C_SLV3_DO        = 0x66,
	I2C_MST_DELAY_CTRL = 0x67,

	SIGNAL_PATH_RESET  = 0x68,

	USER_CTRL          = 0x6A,
	PWR_MGMT_1         = 0x6B,

	FIFO_COUNTH        = 0x72,
	FIFO_COUNTL        = 0x73,
	FIFO_R_W           = 0x74,
	WHO_AM_I           = 0x75,

	XA_OFFSET_H        = 0x77,
	XA_OFFSET_L        = 0x78,

	YA_OFFSET_H        = 0x7A,
	YA_OFFSET_L        = 0x7B,

	ZA_OFFSET_H        = 0x7D,
	ZA_OFFSET_L        = 0x7E,
};

// CONFIG
enum CONFIG_BIT : uint8_t {
	FIFO_MODE = Bit6, // when the FIFO is full, additional writes will not be written to FIFO

	DLPF_CFG_BW_41HZ = Bit1 | Bit0, // 1 kHz internal sample rate
	DLPF_CFG_MASK = Bit2 | Bit1 | Bit0,
	DLPF_CFG_BYPASS_DLPF_8KHZ = 7, // Rate 8 kHz [2:0]
};

// GYRO_CONFIG
enum GYRO_CONFIG_BIT : uint8_t {
	// GYRO_FS_SEL [4:3]
	GYRO_FS_SEL_250_DPS	= 0,           // 0b00000
	GYRO_FS_SEL_500_DPS	= Bit3,        // 0b01000
	GYRO_FS_SEL_1000_DPS	= Bit4,        // 0b10000
	GYRO_FS_SEL_2000_DPS	= Bit4 | Bit3, // 0b11000

	// FCHOICE_B [1:0]
	FCHOICE_B_8KHZ_BYPASS_DLPF = Bit1 | Bit0, // 0b00 - 3-dB BW: 3281 Noise BW (Hz): 3451.0   8 kHz
};

// ACCEL_CONFIG
enum ACCEL_CONFIG_BIT : uint8_t {
	// ACCEL_FS_SEL [4:3]
	ACCEL_FS_SEL_2G  = 0,           // 0b00000
	ACCEL_FS_SEL_4G  = Bit3,        // 0b01000
	ACCEL_FS_SEL_8G  = Bit4,        // 0b10000
	ACCEL_FS_SEL_16G = Bit4 | Bit3, // 0b11000
};

// ACCEL_CONFIG2
enum ACCEL_CONFIG2_BIT : uint8_t {
	ACCEL_FCHOICE_B_BYPASS_DLPF = Bit3,
	A_DLPFCFG_BW_41HZ = Bit1 | Bit0,
	A_DLPFCFG_MASK = Bit2 | Bit1 | Bit0,
};

// FIFO_EN
enum FIFO_EN_BIT : uint8_t {
	TEMP_OUT  = Bit7,
	GYRO_XOUT = Bit6,
	GYRO_YOUT = Bit5,
	GYRO_ZOUT = Bit4,
	ACCEL     = Bit3,
};

// I2C_MST_CTRL
enum I2C_MST_CTRL_BIT : uint8_t {
	I2C_MST_P_NSR = Bit4,
	I2C_MST_CLK_400_kHz = 13,
};

// I2C_SLV0_ADDR
enum I2C_SLV0_ADDR_BIT : uint8_t {
	I2C_SLV0_RNW = Bit7,
};

// I2C_SLV0_CTRL
enum I2C_SLV0_CTRL_BIT : uint8_t {
	I2C_SLV0_EN = Bit7,
	I2C_SLV0_BYTE_SW = Bit6, // Swap bytes when reading both the low and high byte of a word
	I2C_SLV0_REG_DIS = Bit5, // transaction does not write a register value (only read data)
	I2C_SLV0_LENG = Bit3 | Bit2 | Bit1 | Bit0,
};

// I2C_SLV4_CTRL
enum I2C_SLV4_CTRL_BIT : uint8_t {
	I2C_SLV4_EN = Bit7,
	I2C_MST_DLY = Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
	I2C_MST_DLY_4_SAMPLES = Bit1 | Bit0,
	I2C_MST_DLY_32_SAMPLES = I2C_MST_DLY,
};

enum I2C_MST_STATUS_BIT : uint8_t {
	I2C_SLV4_DONE = Bit6,
	I2C_SLV4_NACK = Bit4,
};


// INT_PIN_CFG
enum INT_PIN_CFG_BIT : uint8_t {
	ACTL = Bit7,
};

// I2C_MST_DELAY_CTRL
enum I2C_MST_DELAY_CTRL_BIT : uint8_t {
	I2C_SLV0_DLY_EN = Bit0,
	I2C_SLVX_DLY_EN = Bit4 | Bit3 | Bit2 | Bit1 | Bit0, // limit all slave access (1+I2C_MST_DLY),
};

// INT_ENABLE
enum INT_ENABLE_BIT : uint8_t {
	RAW_RDY_EN = Bit0
};

// SIGNAL_PATH_RESET
enum SIGNAL_PATH_RESET_BIT : uint8_t {
	GYRO_RST  = Bit2,
	ACCEL_RST = Bit1,
	TEMP_RST  = Bit0,
};

// USER_CTRL
enum USER_CTRL_BIT : uint8_t {
	FIFO_EN      = Bit6,
	I2C_MST_EN   = Bit5,
	I2C_IF_DIS   = Bit4, // Always write 0 to I2C_IF_DIS.

	FIFO_RST     = Bit2,
	I2C_MST_RST  = Bit1,
	SIG_COND_RST = Bit0,
};

// PWR_MGMT_1
enum PWR_MGMT_1_BIT : uint8_t {
	H_RESET    = Bit7,
	SLEEP      = Bit6,

	// CLKSEL[2:0]
	CLKSEL_0     = Bit0, // It is required that CLKSEL[2:0] be set to 001 to achieve full gyroscope performance.
};

namespace FIFO
{
static constexpr size_t SIZE = 512;

// FIFO_DATA layout when FIFO_EN has GYRO_{X, Y, Z}OUT and ACCEL set
struct DATA {
	uint8_t ACCEL_XOUT_H;
	uint8_t ACCEL_XOUT_L;
	uint8_t ACCEL_YOUT_H;
	uint8_t ACCEL_YOUT_L;
	uint8_t ACCEL_ZOUT_H;
	uint8_t ACCEL_ZOUT_L;
	uint8_t GYRO_XOUT_H;
	uint8_t GYRO_XOUT_L;
	uint8_t GYRO_YOUT_H;
	uint8_t GYRO_YOUT_L;
	uint8_t GYRO_ZOUT_H;
	uint8_t GYRO_ZOUT_L;
};
}

} // namespace InvenSense_MPU6500
