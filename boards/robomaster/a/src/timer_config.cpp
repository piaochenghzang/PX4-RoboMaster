/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <px4_arch/io_timer_hw_description.h>

/*
 * RoboMaster A motor outputs:
 *
 * S1: PD12 -> TIM4_CH1
 * S2: PD13 -> TIM4_CH2
 * S3: PD14 -> TIM4_CH3
 * S4: PD15 -> TIM4_CH4
 */

constexpr io_timers_t io_timers[MAX_IO_TIMERS] = {
	initIOTimer(Timer::Timer4, DMA{DMA::Index1, DMA::Stream6, DMA::Channel2}),
};

static inline constexpr timer_io_channels_t initIOTimerChannelPulldown(
	const io_timers_t io_timers_conf[MAX_IO_TIMERS],
	Timer::TimerChannel timer,
	GPIO::GPIOPin pin)
{
	timer_io_channels_t ret = initIOTimerChannel(io_timers_conf, timer, pin);
	ret.gpio_out |= GPIO_OUTPUT_CLEAR | GPIO_PULLDOWN;
	return ret;
}

constexpr timer_io_channels_t timer_io_channels[MAX_TIMER_IO_CHANNELS] = {
	initIOTimerChannelPulldown(io_timers, {Timer::Timer4, Timer::Channel1}, {GPIO::PortD, GPIO::Pin12}),
	initIOTimerChannelPulldown(io_timers, {Timer::Timer4, Timer::Channel2}, {GPIO::PortD, GPIO::Pin13}),
	initIOTimerChannelPulldown(io_timers, {Timer::Timer4, Timer::Channel3}, {GPIO::PortD, GPIO::Pin14}),
	initIOTimerChannelPulldown(io_timers, {Timer::Timer4, Timer::Channel4}, {GPIO::PortD, GPIO::Pin15}),
};

constexpr io_timers_channel_mapping_t io_timers_channel_mapping =
	initIOTimerChannelMapping(io_timers, timer_io_channels);
