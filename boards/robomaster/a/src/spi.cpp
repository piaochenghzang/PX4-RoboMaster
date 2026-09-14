/****************************************************************************
 *
 * RoboMaster A SPI hardware description
 *
 ****************************************************************************/

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_t px4_spi_buses[SPI_BUS_MAX_BUS_ITEMS] = {

	initSPIBus(SPI::Bus::SPI5, {

		initSPIDevice(
			DRV_IMU_DEVTYPE_MPU6500,
			SPI::CS{GPIO::PortF, GPIO::Pin6},
			SPI::DRDY{GPIO::PortB, GPIO::Pin8}
		),

	}),
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses);
