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

#include <unistd.h>		// usleep
#include <string.h>		// memcpy
#include <algorithm>	// transform
#include <fstream>		// ifstream

#include "CSerial.h"
#include "Roboclaw.h"

static bool		g_initialized = false;
static CSerial*	g_serial = NULL;

CRoboclaw::CRoboclaw(uint16_t motor)
{
	Reset();

	m_motor = motor;	// set motor number

	if (!readConfigFile())
		throw runtime_error("Interface init exception");

	m_address = (uint8_t)stoi(getConfigValue("address"), 0, 16);

	if (!g_initialized) {	// only once:
		g_serial = new CSerial(getConfigValue("device").c_str(), stoi(getConfigValue("baudrate")));
		if (roboclaw_read_version() != ROBOCLAW_OK)
			throw runtime_error("Module communication exception");
	}

	if (parseXMLFile()) {		// XML-config file existing and ok
		if (!g_initialized) {	// only once:
			if (roboclaw_reset() != ROBOCLAW_OK)	// reset to faxctory defaults
				throw runtime_error("Module init exception");

			usleep(500000);	// give time for reset
		}

		if (!initMotors()) // do the rest
			return;
	}

	g_initialized = true;
}

CRoboclaw::~CRoboclaw(void)
{
	delete g_serial;
}

bool CRoboclaw::initMotors(void)
{
	if (m_motor == 0) {

		// set PID-values:
		if (roboclaw_set_pid_m1(
			(uint32_t)(stod(getConfigValue("d1")) * 1000.0),
			(uint32_t)(stod(getConfigValue("p1")) * 1000.0),
			(uint32_t)(stod(getConfigValue("i1")) * 1000.0),
			stoi(getConfigValue("qpps1"))) != ROBOCLAW_OK) {
			printf("error setting PID-values\n");
			return false;
		}
		// set pos-PID-values:
		if (roboclaw_set_pos_pid_m1(
			(uint32_t)(stod(getConfigValue("posd1")) * 1000.0),
			(uint32_t)(stod(getConfigValue("posp1")) * 1000.0),
			(uint32_t)(stod(getConfigValue("posi1")) * 1000.0),
			stoi(getConfigValue("imax1")),
			stoi(getConfigValue("deadzone1")),
			stoi(getConfigValue("min1")),
			stoi(getConfigValue("max1"))) != ROBOCLAW_OK) {
			printf("error setting pos-PID-values\n");
			return false;
		}
		// clear encoder
		if (roboclaw_set_encoder_m1(0) != ROBOCLAW_OK) {
			printf("error setting pos-PID-values\n");
			return false;
		}
	} else if (m_motor == 1) {
		// set PID-values:
		if (roboclaw_set_pid_m2(
			(uint32_t)(stod(getConfigValue("d2")) * 1000.0),
			(uint32_t)(stod(getConfigValue("p2")) * 1000.0),
			(uint32_t)(stod(getConfigValue("i2")) * 1000.0),
			stoi(getConfigValue("qpps2"))) != ROBOCLAW_OK) {
			printf("error setting PID-values\n");
			return false;
		}
		// set pos-PID-values:
		if (roboclaw_set_pos_pid_m2(
			(uint32_t)(stod(getConfigValue("posd2")) * 1000.0),
			(uint32_t)(stod(getConfigValue("posp2")) * 1000.0),
			(uint32_t)(stod(getConfigValue("posi2")) * 1000.0),
			stoi(getConfigValue("imax2")),
			stoi(getConfigValue("deadzone2")),
			stoi(getConfigValue("min2")),
			stoi(getConfigValue("max2"))) != ROBOCLAW_OK) {
			printf("error setting pos-PID-values\n");
			return false;
		}
		// clear encoder
		if (roboclaw_set_encoder_m2(0) != ROBOCLAW_OK) {
			printf("error setting pos-PID-values\n");
			return false;
		}
	} else {
			printf("motor %d not existing\n", m_motor + 1);
			return false;
	}

	return true;
}

void CRoboclaw::startMove(long time = 0)
{
	int res;

	if (m_motor == 0)
		res = roboclaw_speed_accel_decel_pos_m1((uint32_t)m_accel, (uint32_t)m_speed, (uint32_t)m_decel, (int32_t)m_target / m_microSteps);
	else if (m_motor == 1)
		res = roboclaw_speed_accel_decel_pos_m2((uint32_t)m_accel, (uint32_t)m_speed, (uint32_t)m_decel, (int32_t)m_target / m_microSteps);
	else {
		printf("motor %d not existing\n", m_motor + 1);
		return;
	}
	if (res != ROBOCLAW_OK) {
		printf("problem communicating with roboclaw\n");
		return;
	}
}

bool CRoboclaw::waitCompletion(void) {
	int32_t count;
	uint8_t status;
	uint8_t cmd;
	const int delay = 10000;	// 10 msec

	if (m_motor == 0)
		cmd = GETM1ENC;
	else if (m_motor == 1)
		cmd = GETM2ENC;
	else {
		printf("motor %d not existing\n", m_motor + 1);
		return 0;
	}

	bool reached = false;	// target reached flag
	int i = m_filter;
	while (i > 0) {			// wait for target reached permanently
		usleep(delay);
		if (roboclaw_read_encoder(&count, &status, cmd) != ROBOCLAW_OK) {	// read count
			printf("problem reading count\n");
			return 0;
		}
		count *= m_microSteps;
		if (abs(m_target - count) <= m_corridor) {	// corridor reached:
			i--;			// count a cycle
			reached = true;	// motor has stopped
		}
		if (reached && (abs(m_target - count) > m_corridor)) {	// corridor lost:
			i = m_filter;	// restart wait cycle
			reached = false;
		}
	}

	return 0;
}

long CRoboclaw::getPosition(void)
{
	int32_t count;
	uint8_t status;
	uint8_t cmd;

	if (m_motor == 0)
		cmd = GETM1ENC;
	else if (m_motor == 1)
		cmd = GETM2ENC;
	else {
		printf("motor %d not existing\n", m_motor + 1);
		return 0;
	}

	if (roboclaw_read_encoder(&count, &status, cmd) != ROBOCLAW_OK) {
		printf("problem reading stop speed\n");
		return 0;
	}
	return (long)count * m_microSteps;
}

//- Private -------------------------------------------------------------------

//*****************************************************************************
// User communication functions (ACKed commands)
// 
// On success roboclaw replies with single byte 0xFF as acknowledge.
// User encodes the command, calculates the crc and sends command with crc checksum.
//*****************************************************************************

// commands for both motors

int CRoboclaw::roboclaw_reset()
{
	int bytes = encode_reset(sndbuf);
	return g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, CSERIAL_ACK_BYTES, true);
}

// commands for each motor

int CRoboclaw::roboclaw_set_encoder_m1(uint32_t count)
{
	int bytes = encode_set_encoder(sndbuf, count, SETM1ENCCOUNT);
	return g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, CSERIAL_ACK_BYTES, true);
}

int CRoboclaw::roboclaw_set_encoder_m2(uint32_t count)
{
	int bytes = encode_set_encoder(sndbuf, count, SETM2ENCCOUNT);
	return g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, CSERIAL_ACK_BYTES, true);
}

int CRoboclaw::roboclaw_set_pid_m1(uint32_t D, uint32_t P, uint32_t I, uint32_t QPPS)
{
	int bytes = encode_set_pid(sndbuf, D, P, I, QPPS, SETM1PID);
	return g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, CSERIAL_ACK_BYTES, true);
}

int CRoboclaw::roboclaw_set_pid_m2(uint32_t D, uint32_t P, uint32_t I, uint32_t QPPS)
{
	int bytes = encode_set_pid(sndbuf, D, P, I, QPPS, SETM2PID);
	return g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, CSERIAL_ACK_BYTES, true);
}

int CRoboclaw::roboclaw_set_pos_pid_m1(uint32_t D, uint32_t P, uint32_t I, uint32_t MaxI, uint32_t Deadzone, int32_t MinPos, int32_t MaxPos)
{
	int bytes = encode_set_pos_pid(sndbuf, D, P, I, MaxI, Deadzone, MinPos, MaxPos, SETM1POSPID);
	return g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, CSERIAL_ACK_BYTES, true);
}

int CRoboclaw::roboclaw_set_pos_pid_m2(uint32_t D, uint32_t P, uint32_t I, uint32_t MaxI, uint32_t Deadzone, int32_t MinPos, int32_t MaxPos)
{
	int bytes = encode_set_pos_pid(sndbuf, D, P, I, MaxI, Deadzone, MinPos, MaxPos, SETM2POSPID);
	return g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, CSERIAL_ACK_BYTES, true);
}

int CRoboclaw::roboclaw_speed_accel_decel_pos_m1(uint32_t accel, uint32_t speed, uint32_t deccel, int32_t pos)
{
	int bytes = encode_speed_accel_deccel_pos(sndbuf, accel, speed, deccel, pos, M1SPEEDACCELDECCELPOS);
	return g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, CSERIAL_ACK_BYTES, true);
}

int CRoboclaw::roboclaw_speed_accel_decel_pos_m2(uint32_t accel, uint32_t speed, uint32_t deccel, int32_t pos)
{
	int bytes = encode_speed_accel_deccel_pos(sndbuf, accel, speed, deccel, pos, M2SPEEDACCELDECCELPOS);
	return g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, CSERIAL_ACK_BYTES, true);
}

//*****************************************************************************
// User communication functions (Commands with replies)
// 
// On success roboclaw replies with answer followed by CRC checksum.
// User calculates CRC on command he sends, doesn't send the crc
// Roboclaw calulates CRC starting with user command and continues on its reply, sends the CRC.
// Finally user retrieves answer with CRC and calculates CRC both on what was sent and received
// compares calculated to received.
//*****************************************************************************

int CRoboclaw::roboclaw_read_version(void)
{
	int ret;

	int bytes = encode(sndbuf, GETVERSION);

	if ((ret = g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, -1, false)) < 0)
		return ret;	// IO error or retries exceeded

	m_version = string((char*)recbuf);
	printf("Firmware version: '%s'", m_version.c_str());
	return ROBOCLAW_OK;
}

int CRoboclaw::roboclaw_read_encoder(int32_t* count, uint8_t* status, uint8_t cmd)
{
	int ret;

	int bytes = encode(sndbuf, cmd);

	if ((ret = g_serial->send_cmd_wait_answer(sndbuf, bytes, recbuf, ROBOCLAW_READ_ENCODERS_REPLY_BYTES, false)) < 0)
		return ret;	// IO error or retries exceeded

	decode_read_encoder(recbuf, count, status);

	return ROBOCLAW_OK;
}

//*****************************************************************************
// Protocol related implementation
//*****************************************************************************

// encode and decode primitives functions

int CRoboclaw::encode_uint16(uint8_t* buffer, int bytes, uint16_t value)
{
	value = htobe16(value);
	memcpy(buffer + bytes, &value, sizeof(value));
	return sizeof(value);
}

uint16_t CRoboclaw::decode_uint16_t(uint8_t* buffer)
{
	uint16_t value;
	memcpy(&value, buffer, sizeof(value));
	return be16toh(value);
}

int CRoboclaw::encode_uint32(uint8_t* buffer, int bytes, uint32_t value)
{
	value = htobe32(value);
	memcpy(buffer + bytes, &value, sizeof(value));
	return sizeof(value);
}

uint32_t CRoboclaw::decode_uint32_t(uint8_t* buffer)
{
	uint32_t value;
	memcpy(&value, buffer, sizeof(value));
	return be32toh(value);
}

//*****************************************************************************
// encode commands functions
//*****************************************************************************

int CRoboclaw::encode(uint8_t* buffer, uint8_t cmd) const
{
	uint8_t bytes = 0;
	buffer[bytes++] = m_address;
	buffer[bytes++] = cmd;
	return bytes;
}

int CRoboclaw::encode_reset(uint8_t* buffer) const
{
	int bytes = 0;
	buffer[bytes++] = m_address;
	buffer[bytes++] = RESTOREDEFAULTS;
	return bytes;
}

int CRoboclaw::encode_set_encoder(uint8_t* buffer, uint32_t count, uint8_t cmd) const
{
	int bytes = 0;
	buffer[bytes++] = m_address;
	buffer[bytes++] = cmd;
	bytes += encode_uint32(buffer, bytes, count);
	return bytes;
}

int CRoboclaw::encode_set_pid(uint8_t* buffer, uint32_t D, uint32_t P, uint32_t I, uint32_t QPPS, uint8_t cmd) const
{
	int bytes = 0;
	buffer[bytes++] = m_address;
	buffer[bytes++] = cmd;
	bytes += encode_uint32(buffer, bytes, D);
	bytes += encode_uint32(buffer, bytes, P);
	bytes += encode_uint32(buffer, bytes, I);
	bytes += encode_uint32(buffer, bytes, QPPS);
	return bytes;
}

int CRoboclaw::encode_set_pos_pid(uint8_t* buffer, uint32_t D, uint32_t P, uint32_t I, uint32_t MaxI, uint32_t Deadzone, int32_t MinPos, int32_t MaxPos, uint8_t cmd) const
{
	int bytes = 0;
	buffer[bytes++] = m_address;
	buffer[bytes++] = cmd;
	bytes += encode_uint32(buffer, bytes, D);
	bytes += encode_uint32(buffer, bytes, P);
	bytes += encode_uint32(buffer, bytes, I);
	bytes += encode_uint32(buffer, bytes, MaxI);
	bytes += encode_uint32(buffer, bytes, Deadzone);
	bytes += encode_uint32(buffer, bytes, MinPos);
	bytes += encode_uint32(buffer, bytes, MaxPos);
	return bytes;
}

int CRoboclaw::encode_speed_accel_deccel_pos(uint8_t* buffer, uint32_t accel1, uint32_t speed1, uint32_t deccel1, int32_t pos1, uint8_t cmd) const
{
	int bytes = 0;
	buffer[bytes++] = m_address;
	buffer[bytes++] = cmd;
	bytes += encode_uint32(buffer, bytes, accel1);
	bytes += encode_uint32(buffer, bytes, speed1);
	bytes += encode_uint32(buffer, bytes, deccel1);
	bytes += encode_uint32(buffer, bytes, pos1);
	buffer[bytes++] = 1;	// buffer: 0 = buffered, 1 = immediately
	return bytes;
}

//*****************************************************************************
// encode read commands functions
// 
// roboclaw doesn't expect checksum sent on commands with replies
// however it calculates checksum on both command it received and and reply it sends (combined)
//*****************************************************************************

void CRoboclaw::decode_read_encoder(uint8_t* buffer, int32_t* enc, uint8_t* status)
{
	*enc = (int32_t)decode_uint32_t(buffer);
	*status = *(buffer + sizeof(*enc));
}


//*****************************************************************************
// update copnfig-map from config file
//*****************************************************************************
bool CRoboclaw::readConfigFile(void) {

	ifstream cFile("roboclaw.ini");

	if (cFile.is_open()) {
		string line;
		while (getline(cFile, line)) {
			line.erase(remove_if(line.begin(), line.end(), ::isspace), line.end());	// remove whitespaces
			if (line[0] == '#' || line.empty())
				continue;
			auto delimiterPos = line.find("=");
			auto key = line.substr(0, delimiterPos);
			transform(key.begin(), key.end(), key.begin(), ::tolower);	// convert key to lowercase
			auto value = line.substr(delimiterPos + 1);
			m_config_map.insert(make_pair(key, value));
		}
		cFile.close();
		return true;
	}
	
	printf("no config file found\n");
	return false;
}

//*****************************************************************************
// get value from config-map
//*****************************************************************************
string CRoboclaw::getConfigValue(string key)
{
	for (map<string, string>::iterator it = m_config_map.begin(); it != m_config_map.end(); ++it) {
		if (key.compare(it->first) == 0)
			return it->second;
	}
	printf("key '%s' not found in config file\n", key.c_str());
	return "";
}

//*****************************************************************************
// Verry simple xml-parser, only usefull for a roboclaw cfg-file.
// update config-map from XML-file.
//*****************************************************************************
bool CRoboclaw::parseXMLFile(void) {

	ifstream xmlFile("roboclaw.cfg");

	if (xmlFile.is_open()) {
		string line;
		unsigned int keyEnd;

		while (getline(xmlFile, line)) {
			unsigned int keyStart;
			if (((keyStart = line.find("  <")) == string::npos) &&	// skip header
				((keyStart = line.find("\t<")) == string::npos))
				continue;
			// parts to ignore:
			if (line.find("title") != string::npos) continue;
			if (line.find("imagefilename") != string::npos) continue;
			if (line.find("description") != string::npos) continue;
			if (line.find("signal_data") != string::npos) { ignore(&xmlFile, "/signal_data"); continue; }
			if (line.find("stream_data") != string::npos) { ignore(&xmlFile, "/stream_data"); continue; }

			keyEnd = line.find(">");
			string key = line.substr(keyStart + 3, keyEnd - keyStart - 3);
			if (line.find("</") == string::npos) { // lines following
				string pattern = "/" + key + ">";
				int i = 1;
				while (getline(xmlFile, line)) {
					if (line.find(pattern) != string::npos)	// all lines done 
						break;
					string newkey = key + to_string(i++);
					m_config_map.insert(make_pair(newkey, getValue(line)));
				}
			} else { // all in single line
				m_config_map.insert(make_pair(key, getValue(line)));
			}
		}
	}
	else {
		printf("no config file found\n");
		return false;
	}

	xmlFile.close();
	return true;
}

//*****************************************************************************
// get a value from line
//*****************************************************************************
string CRoboclaw::getValue(string line) {
	unsigned int StartPos = line.find(">");
	unsigned int EndPos = line.find_last_of("<");

	string value = line.substr(StartPos + 1, EndPos - StartPos - 1);
	return value;
}

//*****************************************************************************
// ignore some lines
//*****************************************************************************
void CRoboclaw::ignore(ifstream* xmlFile, string pattern) {
	string line;
	do {
		getline(*xmlFile, line);
	} while (line.find(pattern) == string::npos);
}