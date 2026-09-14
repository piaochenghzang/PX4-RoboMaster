/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file init.c
 *
 * RoboMaster A-specific early startup code.  This file implements the
 * board_app_initialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>

#include <px4_platform_common/init.h>
#include <px4_platform/board_dma_alloc.h>

#include <px4_arch/io_timer.h>

#if defined(FLASH_BASED_PARAMS)
# include <parameters/flashparams/flashfs.h>
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/**
 * Ideally we'd be able to get these from arm_internal.h,
 * but since we want to be able to disable the NuttX use
 * of leds for system indication at will and there is no
 * separate switch, we need to build independent of the
 * CONFIG_ARCH_LEDS configuration switch.
 */
__BEGIN_DECLS
extern void led_init(void);
extern void led_on(int led);
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Protected Functions
 ****************************************************************************/
/****************************************************************************
 * Public Functions
 ****************************************************************************/
/************************************************************************************
 * Name: board_peripheral_reset
 *
 * Description:
 *
 ************************************************************************************/
__EXPORT void board_peripheral_reset(int ms)
{
	/* RoboMaster A has no software-controlled IMU peripheral rail. */
	(void)ms;
}

/****************************************************************************
 * Name: board_on_reset
 ****************************************************************************/

__EXPORT void board_on_reset(int status)
{
        /* Configure all motor outputs as GPIO outputs and hold them low. */
        for (int i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS; ++i) {
                px4_arch_configgpio(io_timer_channel_get_gpio_output(i));
        }

        if (status >= 0) {
                up_mdelay(100);
        }
}

/****************************************************************************
 * Name: stm32_boardinitialize
 ****************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{
        /* Hold all motor outputs low during early boot. */
        board_on_reset(-1);

	/*
	 * Minimal safe board initialization.
	 * Only configure verified RoboMaster A peripherals here.
	 */

	board_autoled_initialize();

	/*
	 * Configure all SPI chip-select pins to their inactive state.
	 */
	stm32_spiinitialize();

	/* RoboMaster A battery ADC inputs. */
	stm32_configgpio(GPIO_ADC1_IN14); /* PC4: battery current */
	stm32_configgpio(GPIO_ADC1_IN15); /* PC5: battery voltage */

	/* IMU heater on PB5, keep off during boot. */
	stm32_configgpio(GPIO_HEATER_OUTPUT);
	HEATER_OUTPUT_EN(0);

	/* TF card detect, active low on PE15. */
	stm32_configgpio(GPIO_SD_CARD_DETECT);
}

#ifdef CONFIG_MMCSD
static struct sdio_dev_s *sdio;
#endif

/****************************************************************************
 * Name: board_app_initialize
 ****************************************************************************/

__EXPORT int board_app_initialize(uintptr_t arg)
{
	(void)arg;

	px4_platform_init();

	/* Configure the DMA allocator. */

	if (board_dma_alloc_init() < 0) {
		syslog(LOG_ERR, "DMA alloc FAILED\n");
	}

#if defined(SERIAL_HAVE_RXDMA)
	/*
	 * Poll serial RX DMA every millisecond for bytes that have not
	 * generated a DMA event.
	 */

	static struct hrt_call serial_dma_call;

	hrt_call_every(&serial_dma_call,
		       1000,
		       1000,
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);
#endif

#if defined(FLASH_BASED_PARAMS)
	static sector_descriptor_t params_sector_map[] = {
		{23, 128 * 1024, 0x081e0000},
		{0, 0, 0},
	};

	int result = parameter_flashfs_init(params_sector_map, NULL, 0);

	if (result != OK) {
		syslog(LOG_ERR, "[boot] parameter flash init failed: %d\n", result);
		return -ENODEV;
	}
#endif

	/* Initial LED state: both physical LEDs off. */

	drv_led_start();
	led_off(LED_RED);
	led_off(LED_GREEN);

	/* Initialize hard-fault storage. */

	if (board_hardfault_init(2, true) != 0) {
		led_on(LED_RED);
	}
#ifdef CONFIG_MMCSD
	/*
	 * RoboMaster A uses the standard STM32F4 SDIO pins:
	 * PC8-PC12 and PD2.
	 *
	 * PE15 is the active-low card-detect switch. Detection is sampled during
	 * boot; inserting or removing a card while powered is not supported.
	 */

	sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);

	if (sdio == NULL) {
		led_on(LED_RED);
		syslog(LOG_ERR,
		       "[boot] Failed to initialize SDIO slot %d\n",
		       CONFIG_NSH_MMCSDSLOTNO);

	} else {
		int ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);

		if (ret != OK) {
			led_on(LED_RED);
			syslog(LOG_ERR,
			       "[boot] Failed to bind SDIO to MMC/SD: %d\n",
			       ret);

		} else {
			sdio_mediachange(sdio, SD_CARD_INSERTED());
		}
	}
#endif

	/*
	 * Register board SPI/I2C hardware descriptions with PX4.
	 */
	px4_platform_configure();

	return OK;
}
