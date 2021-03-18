/*
 * roboclaw library header
 *
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 * Copyright 2021 (C) Wolfram Köhn <wolfram.koehn@gmx.net>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */
#ifndef CSERIAL_H
#define CSERIAL_H

#include <cstdint>
#include <termios.h>

enum {
	CSERIAL_DEFAULT_RETRIES		= 3,

	CSERIAL_B2400_TIMEOUT_MS	= 200,
	CSERIAL_B9600_TIMEOUT_MS	= 30,
	CSERIAL_B19200_TIMEOUT_MS	= 20,
	CSERIAL_B38400_TIMEOUT_MS	= 15,
	CSERIAL_B57600_TIMEOUT_MS	= 13,
	CSERIAL_MIN_TIMEOUT_MS		= 12
};

// ACK bytes, crc sizes, reply sizes
enum {
	CSERIAL_ACK_BYTE	= 0xFF,
	CSERIAL_ACK_BYTES	= 1,
	CSERIAL_CRC16_BYTES	= 2,
};

class CSerial
{
public:
	/* Constructor for serial (RS232) communication
	 *
	 * parameters:
	 * - tty - the device (e.g. /dev/ttyAMA0, /dev/ttyUSB0, /dev/ttyO1)
	 * - baudrate - has to match the one set on roboclaw (2400, 9600, 19200, 38400, 57600, 115200, 230400, 460800)
	*/
	CSerial(const char* tty, int baudrate);

	/* Destructor
	 *
	 * Closes file descriptor, resets terminal to initial settings.
	 */
	~CSerial(void);

	int send_cmd_wait_answer(uint8_t* sndbuf, int bytes_to_write, uint8_t* recbuf, int bytes_to_read, bool addcrc);

private:
	bool init(const char* tty, int baudrate);
	int write_all(uint8_t* data, int data_size);
	int wait_input(struct termios* io, int bytes_to_read);
	int flush_io(void);

	static uint16_t update_crc16(const uint8_t* packet, int bytes, uint16_t crc);
	static int add_checksum(uint8_t* buffer, int bytes);
	static int termiosCompare(const struct termios sruct_1, const struct termios struct_2);

	int				m_fd;				// file descriptor for serial device
	int				m_timeout;			// serial comunication timeout
	struct termios	m_initial_termios;	// original interface configuration (will be restored at termination)
	struct termios	m_actual_termios;	// new interface configuration
};
#endif	// CSERIAL_H