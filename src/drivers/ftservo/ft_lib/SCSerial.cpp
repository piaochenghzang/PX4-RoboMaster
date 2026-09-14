/*
 * SCSerial.h
 * 飞特串行舵机硬件接口层程序
 * 日期: 2022.3.29
 * 作者: 
 */

#include "SCSerial.h"

#include <px4_platform_common/log.h>

SCSerial::SCSerial():SCS{0}
{
}

SCSerial::SCSerial(u8 end):SCS(end)
{
}

SCSerial::SCSerial(u8 end, u8 level):SCS(end, level)
{
}

SCSerial::~SCSerial()
{
    end();
}

bool SCSerial::begin(uint32_t baudrate, const char* device)
{
    if (device == nullptr) {
        PX4_ERR("serial device is null");
        return false;
    }

    end();

    if (!_serial.setPort(device)) {
		PX4_ERR("invalid serial port: %s", device);
		return false;
	}
    
    if (!_serial.setBaudrate(baudrate)) {
		PX4_ERR("unsupported baudrate: %lu",
			static_cast<unsigned long>(baudrate));
		return false;
	}

    if (!_serial.open()) {
		PX4_ERR("failed to open %s", device);
		return false;
	}

    _serial.flush();
	_tx_length = 0;
	_tx_overflow = false;

	PX4_INFO("opened %s at %lu baud",
		 device,
		 static_cast<unsigned long>(baudrate));
    return true;
}


int SCSerial::readSCS(unsigned char *data, int length, unsigned long timeout_ms)
{	
    if (!_serial.isOpen() || data == nullptr || length <= 0) {
		return 0;
	}

	const ssize_t received = _serial.readAtLeast(
		data,
		static_cast<size_t>(length),
		static_cast<size_t>(length),
		static_cast<uint32_t>(timeout_ms)
	);

	if (received < 0) {
		u8Error = ERR_NO_REPLY;
		return 0;
	}

	return static_cast<int>(received);
}

int SCSerial::readSCS(unsigned char *data, int length)
{	
    return readSCS(data, length, _timeout_ms);
}

int SCSerial::writeSCS(unsigned char *data, int length)
{
	if (data == nullptr || length <= 0) {
        return 0;
    }

    const size_t requested = static_cast<size_t>(length);
	const size_t available = TX_BUFFER_SIZE - _tx_length;

    if (requested > available) {
		_tx_overflow = true;
		u8Error = ERR_BUFF_LEN;
		return 0;
	}

    for (size_t i = 0; i < requested; ++i) {
		_tx_buffer[_tx_length++] = data[i];
	}

	return length;
}

int SCSerial::writeSCS(unsigned char data)
{
	if (_tx_length >= TX_BUFFER_SIZE) {
		_tx_overflow = true;
		u8Error = ERR_BUFF_LEN;
		return 0;
	}

	_tx_buffer[_tx_length++] = data;
	return 1;
}

void SCSerial::rFlushSCS()
{
	if (_serial.isOpen())
	{
		_serial.flush();
	}
	
}

void SCSerial::wFlushSCS()
{
	if (!_serial.isOpen() || _tx_length == 0) {
		_tx_length = 0;
		_tx_overflow = false;
		return;
	}

	if (_tx_overflow) {
		PX4_ERR("servo TX packet overflow");
		_tx_length = 0;
		_tx_overflow = false;
		return;
	}

	const ssize_t written = _serial.writeBlocking(
		_tx_buffer,
		_tx_length,
		_timeout_ms
	);

	if (written != static_cast<ssize_t>(_tx_length)) {
		PX4_WARN("servo write failed: %ld/%u",
			 static_cast<long>(written),
			 static_cast<unsigned>(_tx_length));
		u8Error = ERR_NO_REPLY;
	}

	_tx_length = 0;
	_tx_overflow = false;
}

void SCSerial::end()
{
	if (_serial.isOpen()) {
		_serial.close();
	}

	_tx_length = 0;
	_tx_overflow = false;
}
