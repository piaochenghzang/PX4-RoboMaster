/****************************************************************************
 *
 * RoboMaster Development Board Type A I2C configuration
 *
 ****************************************************************************/

#include <px4_arch/i2c_hw_description.h>

/*
 * Internal I2C2:
 *
 * PF1 -> I2C2_SCL
 * PF0 -> I2C2_SDA
 *
 * The onboard IST8310 is connected to the MPU6500 auxiliary I2C bus,
 * not directly to this MCU I2C2 bus.
 */

constexpr px4_i2c_bus_t px4_i2c_buses[I2C_BUS_MAX_BUS_ITEMS] = {
	initI2CBusInternal(2),
};
