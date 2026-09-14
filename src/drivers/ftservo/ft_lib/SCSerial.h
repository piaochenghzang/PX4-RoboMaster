/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2022.3.29
 * 作者: 
 */

#ifndef _SCSERIAL_H
#define _SCSERIAL_H

#include "SCS.h"
#include <cstddef>
#include <cstdint>

#include <px4_platform_common/Serial.hpp>

class SCSerial : public SCS
{
public:
	SCSerial();
	explicit SCSerial(u8 end);
	SCSerial(u8 end, u8 level);
	~SCSerial() override;
	bool begin(uint32_t baudRate = 1'000'000, const char* device = "/dev/ttyS5");
	void end();

	void setTimeout(uint32_t timeout_ms)
	{
		_timeout_ms = timeout_ms;
	}

protected:
	int writeSCS(unsigned char *nDat, int nLen) override;//输出nLen字节
	int readSCS(unsigned char *nDat, int nLen) override;//输入nLen字节
	int readSCS(unsigned char *nDat, int nLen, unsigned long TimeOut) override;
	int writeSCS(unsigned char bDat) override;//输出1字节
	void rFlushSCS() override;//
	void wFlushSCS() override;//

private:
	static constexpr size_t TX_BUFFER_SIZE{256};
	device::Serial _serial{};

	uint8_t _tx_buffer[TX_BUFFER_SIZE]{};
	size_t _tx_length{0};

	uint32_t _timeout_ms{20};
	bool _tx_overflow{false};
};

#endif
