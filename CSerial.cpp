/*
 * roboclaw library implementation: https://github.com/bmegli/roboclaw
 *
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 * Copyright 2021 (C) Wolfram Köhn <wolfram.koehn@gmx.net>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */

#include <unistd.h>		// read, close
#include <string.h>		// memcpy
#include <fcntl.h>		// O_RDWR file open flag
#include <assert.h>		// assert

#include "Roboclaw.h"
#include "CSerial.h"

CSerial::CSerial(const char* tty, int baudrate)
{
	if (!init(tty, baudrate)) {
		throw runtime_error("Serial interface exception");
	}
}

CSerial::~CSerial(void)
{
	int error = 0;

	error |= tcsetattr(m_fd, TCSANOW, &m_initial_termios) < 0;
	error |= close(m_fd) < 0;

	if (error) {
		printf("error '%s' when setting initial attributes\n", strerror(error));
	}
}

bool CSerial::init(const char* tty, int baudrate)
{
	speed_t speed = 0;

	if		(baudrate == 2400)	 speed = B2400,	  m_timeout = CSERIAL_B2400_TIMEOUT_MS;
	else if (baudrate == 9600)	 speed = B9600,	  m_timeout = CSERIAL_B9600_TIMEOUT_MS;
	else if (baudrate == 19200)	 speed = B19200,  m_timeout = CSERIAL_B19200_TIMEOUT_MS;
	else if (baudrate == 38400)	 speed = B38400,  m_timeout = CSERIAL_B38400_TIMEOUT_MS;
	else if (baudrate == 57600)	 speed = B57600,  m_timeout = CSERIAL_B57600_TIMEOUT_MS;
	else if (baudrate == 115200) speed = B115200, m_timeout = CSERIAL_MIN_TIMEOUT_MS;
	else if (baudrate == 230400) speed = B230400, m_timeout = CSERIAL_MIN_TIMEOUT_MS;
	// B460800 is non-standard but is defined on modern systems
#ifdef B460800
	if (baudrate == 460800) speed = B460800, m_timeout = CSERIAL_MIN_TIMEOUT_MS;
#endif

	if (speed == 0) {	// wrong baudrate
		errno = EINVAL;
		printf("wrong baudrate '%d'\n", baudrate);
		return false;
	}

	if ((m_fd = open(tty, O_RDWR)) < 0) {	// open interface
		printf("error '%s' when opening device\n", strerror(errno));
		return false;
	}

	if (tcgetattr(m_fd, &m_initial_termios) < 0) {	// read current settings
		printf("error '%s' when reading current settings\n", strerror(errno));
		close(m_fd);
		return false;
	}

	bzero(&m_actual_termios, sizeof(m_actual_termios));	// clear new structure
	cfmakeraw(&m_actual_termios);						// set raw-mode

	m_actual_termios.c_iflag = m_actual_termios.c_oflag = m_actual_termios.c_lflag = 0;
	m_actual_termios.c_cflag = CS8 | CREAD | CLOCAL;	// 8 bit characters	
	m_actual_termios.c_cc[VMIN] = 1;
	m_actual_termios.c_cc[VTIME] = 0;

	if (cfsetispeed(&m_actual_termios, speed) < 0 ||
		cfsetospeed(&m_actual_termios, speed) < 0) {
		printf("error '%s' when setting speed\n", strerror(errno));
		close(m_fd);
		return false;
	}

	// Reason for the following loop:
	//'Note that tcsetattr() returns success if any of the  requested  changes'
	//'could  be  successfully  carried  out.  Therefore, when making multiple'
	//'changes it may be necessary to follow this call with a further call  to'
	//'tcgetattr() to check that all changes have been performed successfully.'
	struct termios check_termios;
	for (int retry = 0; retry < CSERIAL_DEFAULT_RETRIES; retry++) {
		if (tcflush(m_fd, TCIFLUSH) ||
			tcsetattr(m_fd, TCSAFLUSH, &m_actual_termios) < 0) {
			printf("error '%s' when setting attributes\n", strerror(errno));
			close(m_fd);
			return false;
		}

		if (tcgetattr(m_fd, &check_termios) < 0) {
			printf("error '%s' when reading attributes\n", strerror(errno));
			close(m_fd);
			return false;
		}

		if (termiosCompare(check_termios, m_actual_termios) == 0)
			return true;

		printf("retry %d started\n", retry + 1);
	}

	printf("unable to set attributes\n");
	close(m_fd);
	return false;
}

//*****************************************************************************
// Send the `bytes_to_write` long command and wait until timeout for `bytes read` bytes reply.
// Retry at most `CSERIAL_DEFAULT_RETRIES` times both if timeout on send or received incorrect checksum.
// If expected `bytes_to_read` is single byte, expect the 0xFF ACK byte, otherwise expect reply followed by CRC.
// 'addcrc' controls, if a crc has to be added before sending.
// Returns: ROBOCLAW_ERROR (-1) on IO error (errno set), ROBOCLAW_RETRIES_EXCEEDED(-2) on exceeded retries, ROBOCLAW_OK (0) on success.
//*****************************************************************************
int CSerial::send_cmd_wait_answer(uint8_t* sndbuf, int bytes_to_write, uint8_t* recbuf, int bytes_to_read, bool addcrc)
{
	int retry = 0;
	int status;

	// checksum required
	if (addcrc)
		bytes_to_write = add_checksum(sndbuf, bytes_to_write);

	// block with send/receive/crc check retries (this means roboclaw received correctly but crc failure happened on response)
	for (; retry < CSERIAL_DEFAULT_RETRIES; ++retry) {
		// block with send/wait for answer retries on timeout (this means that roboclaw didn't receive correctly)
		for (; retry < CSERIAL_DEFAULT_RETRIES; ++retry) {
			if (write_all(sndbuf, bytes_to_write) < 0)
				return ROBOCLAW_ERROR;	// error, errno set

			if ((status = wait_input(&m_actual_termios, bytes_to_read)) == 1)
				break;	// got answer

			if (status < 0)
				return ROBOCLAW_ERROR;	// error, errno set

			printf("inner retry %d started\n", retry + 1);
			// otherwise ret == 0, timetout, retry if requested
			if (flush_io() < 0)
				return ROBOCLAW_ERROR;
		}

		if (retry >= CSERIAL_DEFAULT_RETRIES) {	// max. number of retries reached
			printf("retries exxeded %d\n", retry + 1);
			return ROBOCLAW_RETRIES_EXCEEDED;
		}

		int bytes_read;
		// variable length:
		if (bytes_to_read == ROBOCLAW_READ_VARIABLE_BYTES) {
			int i = 0;
			do {
				if (wait_input(&m_actual_termios, bytes_to_read) < 0)
					return ROBOCLAW_ERROR;
				if ((bytes_read = read(m_fd, recbuf + i, ROBOCLAW_BUFFER_SIZE)) < 0) {
					printf("error '%s' reading from device\n", strerror(errno));
					return ROBOCLAW_ERROR;	// errno set
				}
				i += bytes_read;
			} while ((i < (CSERIAL_CRC16_BYTES + 1)) ||			// minimal number of bytes read
			  (recbuf[i - (CSERIAL_CRC16_BYTES + 1)] != 0));	// till end of line plus checksum

			bytes_read = i;	// length of variable block
		// fix length:
		} else {
			if ((bytes_read = read(m_fd, recbuf, ROBOCLAW_BUFFER_SIZE)) < 0) {
				printf("error '%s' reading from device\n", strerror(errno));
				return ROBOCLAW_ERROR;	// errno set
			}
			// if no error we may expect to be woken when enough bytes are waiting, this is what we told the OS through VMINq
			assert(bytes_read == bytes_to_read);
		}

		// special case, command with ACK reply only has no crc only ACK byte
		if (bytes_to_read == CSERIAL_ACK_BYTES) {
			if (recbuf[0] == CSERIAL_ACK_BYTE)
				return ROBOCLAW_OK;	// success

			if (flush_io() < 0)
				return ROBOCLAW_ERROR;

			printf("no ACK, retry %d started\n", retry + 1);
			continue;
		}
		// now calculate and check crc16	
		if (update_crc16(recbuf, bytes_read, update_crc16(sndbuf, bytes_to_write, 0)) == 0)
			return ROBOCLAW_OK;	// success

		// if we got this far we received answer with CRC faiulure
		if (flush_io() < 0)
			return ROBOCLAW_ERROR;

		printf("outer retry %d started\n", retry + 1);
	}

	printf("retries exxeded %d\n", retry + 1);
	return ROBOCLAW_RETRIES_EXCEEDED;
}

//- Private -------------------------------------------------------------------

//*****************************************************************************
// send data via serial interface
//*****************************************************************************
int CSerial::write_all(uint8_t* data, int data_size)
{
	int written = 0;

	while (written < data_size) {
		int result = write(m_fd, data + written, (size_t)(data_size - written));
		if (result < 0) {
			printf("error '%s' when writing to device\n", strerror(errno));
			return -1;
		}
		written += result;
	}
	return 0;
}

//*****************************************************************************
// Tell through termios that we want to be woken up only if bytes_read bytes are waiting.
// Timewout if CSERIAL_PACKET_TIMEOUT_MS is exceeded.
// Returns: -1 on error(can be signal like EINTR), 0 on timetout, 1 on input ok.
//*****************************************************************************
int CSerial::wait_input(struct termios* io, int bytes_to_read)
{
	fd_set			rfds;
	struct timeval	tv;

	FD_ZERO(&rfds);
	FD_SET(m_fd, &rfds);

	tv.tv_sec = 0;
	tv.tv_usec = m_timeout * 1000;

	if (bytes_to_read != ROBOCLAW_READ_VARIABLE_BYTES) {	// only if we know the number of bytes to receive:
		if (io->c_cc[VMIN] != bytes_to_read) {				// tell the system to only wake us up if enough data is waiting
			io->c_cc[VMIN] = (cc_t)bytes_to_read;
			if (tcsetattr(m_fd, TCSANOW, io) < 0) {
				printf("error '%s' when waiting for input from device\n", strerror(errno));
				return -1;	// different error?
			}
		}
	}
	
	int	ret = select(m_fd + 1, &rfds, NULL, NULL, &tv);		// wait for data
	if (ret < 0) {
		printf("error '%s' when waiting for input from device\n", strerror(errno));
		return ret;	// EINTR or some other error, errno set
	}

	if (ret == 0) {
		printf("timout when waiting for input from device\n");
		return 0;
	}

	return 1;	// success
}

//*****************************************************************************
// flush IO buffers
//*****************************************************************************
int CSerial::flush_io(void)
{
	int ret;
	if ((ret = tcflush(m_fd, TCIOFLUSH)) < 0)
		printf("error '%s' when flushing IO\n", strerror(errno));

	return ret;
}

//*****************************************************************************
// calculate crc16 (x^16 + x^15 + x^2 + 1) checksum
//*****************************************************************************
uint16_t CSerial::update_crc16(const uint8_t* packet, int bytes, uint16_t crc)
{
	while (bytes--) {
		register int x = crc >> 8 ^ *packet++;
		x ^= x >> 4;
		crc = (uint16_t)((crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x));
	}
	return crc;
}

//*****************************************************************************
// append checksum to end of buffer
//*****************************************************************************
int CSerial::add_checksum(uint8_t* buffer, int bytes)
{
	uint16_t checksum = update_crc16(buffer, bytes, 0);
	buffer[bytes++] = (uint8_t)((checksum >> 8) & 0xFF);
	buffer[bytes++] = (uint8_t)(checksum & 0xFF);
	return bytes;
}

//*****************************************************************************
// compares two termios structures
//*****************************************************************************
int CSerial::termiosCompare(const struct termios sruct_1, const struct termios struct_2)
{
	if (sruct_1.c_iflag != struct_2.c_iflag)
		return -1;
	if (sruct_1.c_oflag != struct_2.c_oflag)
		return -1;
	if (sruct_1.c_cflag != struct_2.c_cflag)
		return -1;
	if (sruct_1.c_lflag != struct_2.c_lflag)
		return -1;
	if (sruct_1.c_line != struct_2.c_line)
		return -1;
	for (int i = 0; i < NCCS; i++) {
		if (sruct_1.c_cc[i] != struct_2.c_cc[i])
			return -1;
	}
	if (sruct_1.c_ispeed != struct_2.c_ispeed)
		return -1;
	if (sruct_1.c_ospeed != struct_2.c_ospeed)
		return -1;

	return 0;	// success
}