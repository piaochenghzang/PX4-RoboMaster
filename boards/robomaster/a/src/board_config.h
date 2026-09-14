/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * PX4_ROBOMASTER_A internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/
/* Configuration ************************************************************************************/

/* RoboMaster A GPIOs ***********************************************************************************/
/* LEDs
 *
 * RoboMaster A:
 *   Green LED: PF14, active low
 *   Red LED:   PE11, active low
 *
 * GPIO_OUTPUT_SET keeps the LEDs off during GPIO initialization.
 */

#define GPIO_LED_GREEN \
	(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTF | GPIO_PIN14)

#define GPIO_LED_RED \
	(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | GPIO_OUTPUT_SET | GPIO_PORTE | GPIO_PIN11)

/* The board has no physical blue or safety LED.
 * Map the logical LEDs to the two available LEDs.
 */
#define GPIO_LED_BLUE       GPIO_LED_GREEN
#define GPIO_LED_SAFETY     GPIO_LED_RED

#define BOARD_HAS_CONTROL_STATUS_LEDS 1

#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_GREEN
#define BOARD_ARMED_STATE_LED  LED_GREEN

#ifdef CONFIG_STM32_SPI4
#  define BOARD_HAS_BUS_MANIFEST 1 // We support a bus manifest because spi 4 is optional
#endif /* CONFIG_STM32_SPI4 */

/**
 * ADC channels:
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver.
 */
#define ADC_CHANNELS ((1 << 14) | (1 << 15))

/* ADC defines to be used in sensors.cpp to read from a particular channel. */
#define ADC_BATTERY_VOLTAGE_CHANNEL  15
#define ADC_BATTERY_CURRENT_CHANNEL  14

/* Power supply control and monitoring GPIOs. */
// #define GPIO_VDD_BRICK_VALID         (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN5)
// #define GPIO_VDD_USB_VALID           (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTC|GPIO_PIN0)

/* Tone alarm output. */
#define TONE_ALARM_TIMER             12   /* timer 12 */
#define TONE_ALARM_CHANNEL           1    /* channel 1 */
#define GPIO_TONE_ALARM_IDLE         (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTH|GPIO_PIN6)
#define GPIO_TONE_ALARM              GPIO_TIM12_CH1OUT_1

/* Four motor PWM outputs: TIM4 CH1-CH4. */
#define DIRECT_PWM_OUTPUT_CHANNELS   4

/* High-resolution timer */
#define HRT_TIMER                    3  /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL            4  /* use capture/compare channel 4 */

/* RC Serial port */

#define RC_SERIAL_PORT               "/dev/ttyS0"

/* The onboard DBUS transistor provides fixed signal inversion. */
#define RC_INVERT_INPUT(_invert_true) do { (void)(_invert_true); } while (0)

/* Heater pins */
#define GPIO_HEATER_INPUT            (GPIO_INPUT|GPIO_PULLDOWN|GPIO_PORTB|GPIO_PIN5)
#define GPIO_HEATER_OUTPUT           (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)
#define HEATER_OUTPUT_EN(on_true)    px4_arch_gpiowrite(GPIO_HEATER_OUTPUT, (on_true))

/* TF card detect: PE15 is pulled high and grounded by the socket when a
 * card is inserted (SD_EXTI in the RoboMaster A schematic).
 */
#define GPIO_SD_CARD_DETECT          (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTE | GPIO_PIN15)
#define SD_CARD_INSERTED()           (!px4_arch_gpioread(GPIO_SD_CARD_DETECT))

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

#define BOARD_HAS_ON_RESET 1

#define FLASH_BASED_PARAMS 1

__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip-select GPIO pins for the RoboMaster A board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
