/*
 * roboclaw library header
 *
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 */
#ifndef ROBOCLAW_H
#define ROBOCLAW_H

using namespace std;

#include <map>

// error codes returned by the functions
enum { ROBOCLAW_ERROR = -1, ROBOCLAW_RETRIES_EXCEEDED = -2, ROBOCLAW_OK = 0 };

enum { ROBOCLAW_BUFFER_SIZE = 64 };	// has to be big enough to accomodate commands

enum {
	ROBOCLAW_READ_VARIABLE_BYTES		= -1,
	ROBOCLAW_READ_ENCODERS_REPLY_BYTES	= 7
};

enum {
	ERROR_NONE		= 0x00000000,	// no error
	ERROR_ESTOP		= 0x00000001,	// Error: E-Stop active
	ERROR_TEMP		= 0x00000002,	// Error: Temperature Sensor 1 >=100C
	ERROR_TEMP2		= 0x00000004,	// Error: Temperature Sensor 2 >=100C (available only on some models)
	ERROR_MBATHIGH	= 0x00000008,	// Error: Main Battery Over Voltage
	ERROR_LBATHIGH	= 0x00000010,	// Error: Logic Battery High Voltage
	ERROR_LBATLOW	= 0x00000020,	// Error: Logic Battery Low Voltage
	ERROR_FAULTM1	= 0x00000040,	// Error: Motor 1 Driver Fault (only on some models)
	ERROR_FAULTM2	= 0x00000080,	// Error: Motor 2 Driver Fault (only on some models)
	ERROR_SPEED1	= 0x00000100,	// Error: Motor 1 Speed Error Limit
	ERROR_SPEED2	= 0x00000200,	// Error: Motor 2 Speed Error Limit
	ERROR_POS1		= 0x00000400,	// Error: Motor 1 Position Error Limit
	ERROR_POS2		= 0x00000800,	// Error: Motor 2 Position Error Limit
	ERROR_CURRENT1	= 0x00001000,	// Error: Motor 1 Current Error Limit
	ERROR_CURRENT2	= 0x00002000,	// Error: Motor 2 Current Error Limit
	WARN_CURRENTM1	= 0x00010000,	// Warning: Motor 1 Current Limited
	WARN_CURRENTM2	= 0x00020000,	// Warning: Motor 2 Current Limited
	WARN_MBATHIGH	= 0x00040000,	// Warning: Main Battery Voltage High
	WARN_MBATLOW	= 0x00080000,	// Warning: Main Battery Low Voltage
	WARN_TEMP		= 0x00100000,	// Warning: Temperature Sensor 1 >=85C
	WARN_TEMP2		= 0x00200000,	// Warning: Temperature Sensor 2 >=85C (available only on some models)
	WARN_S4			= 0x00400000,	// Warning: Motor 1 Home/Limit Signal
	WARN_S5			= 0x00800000,	// Warning: Motor 2 Home/Limit Signal
	WARN_SPEEDLIMIT	= 0x01000000,	// Warning: Speed Error Limit
	WARN_POSLIMIT	= 0x02000000,	// Warning: Position Error Limit
};

// Roboclaw commands
enum {
	M1FORWARD					= 0	,	// Drive Forward Motor 1
	M1BACKWARD					= 1	,	// Drive Backwards Motor 1
	SETMINMB					= 2	,	// Set Main Voltage Minimum
	SETMAXMB					= 3	,	// Set Main Voltage Maximum
	M2FORWARD					= 4	,	// Drive Forward Motor 2
	M2BACKWARD					= 5	,	// Drive Backwards Motor 2
	M17BIT						= 6	,	// Drive Motor 1 (7 Bit)
	M27BIT						= 7	,	// Drive Motor 2 (7 Bit)
	MIXEDFORWARD				= 8	,	// Drive Forward Mixed Mode
	MIXEDBACKWARD				= 9	,	// Drive Backwards Mixed Mode
	MIXEDRIGHT					= 10,	// Turn Right Mixed Mode
	MIXEDLEFT					= 11,	// Turn Left Mixed Mode
	MIXEDFB						= 12,	// Drive Forward or Backward (7 bit)
	MIXEDLR						= 13,	// Turn Left or Right (7 Bit)
	SETSERTIMEOUT				= 14,	// Set Serial Timeout
	GETSERTIMEOUT				= 15,	// Read Serial Timeout
	GETM1ENC					= 16,	// Read Encoder Count/Value for M1
	GETM2ENC					= 17,	// Read Encoder Count/Value for M2
	GETM1SPEED					= 18,	// Read M1 Speed in Encoder Counts Per Second
	GETM2SPEED					= 19,	// Read M2 Speed in Encoder Counts Per Second
	RESETENC					= 20,	// Resets Encoder Registers for M1 and M2(Quadrature only)
	GETVERSION					= 21,	// Read Firmware Version
	SETM1ENCCOUNT				= 22,	// Set Encoder 1 Register(Quadrature only)
	SETM2ENCCOUNT				= 23,	// Set Encoder 2 Register(Quadrature only)
	GETMBATT					= 24,	// Read Main Battery Voltage
	GETLBATT					= 25,	// Read Logic Battery Voltage
	SETMINLB					= 26,	// Set Minimum Logic Voltage Level
	SETMAXLB					= 27,	// Set Maximum Logic Voltage Level
	SETM1PID					= 28,	// Set Velocity PID Constants for M1
	SETM2PID					= 29,	// Set Velocity PID Constants for M2
	GETM1ISPEED					= 30,	// Read Current M1 Raw Speed
	GETM2ISPEED					= 31,	// Read Current M2 Raw Speed
	M1DUTY						= 32,	// Drive M1 With Signed Duty Cycle. (Encoders not required)
	M2DUTY						= 33,	// Drive M2 With Signed Duty Cycle. (Encoders not required)
	MIXEDDUTY					= 34,	// Drive M1 / M2 With Signed Duty Cycle. (Encoders not required)
	M1SPEED						= 35,	// Drive M1 With Signed Speed
	M2SPEED						= 36,	// Drive M2 With Signed Speed
	MIXEDSPEED					= 37,	// Drive M1 / M2 With Signed Speed
	M1SPEEDACCEL				= 38,	// Drive M1 With Signed Speed And Acceleration
	M2SPEEDACCEL				= 39,	// Drive M2 With Signed Speed And Acceleration
	MIXEDSPEEDACCEL				= 40,	// Drive M1 / M2 With Signed Speed And Acceleration
	M1SPEEDDIST					= 41,	// Drive M1 With Signed Speed And Distance. Buffered
	M2SPEEDDIST					= 42,	// Drive M2 With Signed Speed And Distance. Buffered
	MIXEDSPEEDDIST				= 43,	// Drive M1 / M2 With Signed Speed And Distance. Buffered
	M1SPEEDACCELDIST			= 44,	// Drive M1 With Signed Speed, Acceleration and Distance. Buffered
	M2SPEEDACCELDIST			= 45,	// Drive M2 With Signed Speed, Acceleration and Distance. Buffered
	MIXEDSPEEDACCELDIST			= 46,	// Drive M1 / M2 With Signed Speed, Acceleration And Distance. Buffered
	GETBUFFERS					= 47,	// Read Buffer Length
	GETPWMS						= 48,	// Read Motor PWMs
	GETCURRENTS					= 49,	// Read Motor Currents
	MIXEDSPEED2ACCEL			= 50,	// Drive M1 / M2 With Individual Signed Speed and Acceleration
	MIXEDSPEED2ACCELDIST		= 51,	// Drive M1 / M2 With Individual Signed Speed, Accel and Distance
	M1DUTYACCEL					= 52,	// Drive M1 With Signed Duty and Accel. (Encoders not required)
	M2DUTYACCEL					= 53,	// Drive M2 With Signed Duty and Accel. (Encoders not required)
	MIXEDDUTYACCEL				= 54,	// Drive M1 / M2 With Signed Duty and Accel. (Encoders not required)
	READM1PID					= 55,	// Read Motor 1 Velocity PID Constants
	READM2PID					= 56,	// Read Motor 2 Velocity PID Constants
	SETMAINVOLTAGES				= 57,	// Set Main Battery Voltages
	SETLOGICVOLTAGES			= 58,	// Set Logic Battery Voltages
	GETMINMAXMAINVOLTAGES		= 59,	// Read Main Battery Voltage Settings
	GETMINMAXLOGICVOLTAGES		= 60,	// Read Logic Battery Voltage Settings
	SETM1POSPID					= 61,	// Set Position PID Constants for M1
	SETM2POSPID					= 62,	// Set Position PID Constants for M2
	READM1POSPID				= 63,	// Read Motor 1 Position PID Constants
	READM2POSPID				= 64,	// Read Motor 2 Position PID Constants
	M1SPEEDACCELDECCELPOS		= 65,	// Drive M1 with Speed, Accel, Deccel and Position
	M2SPEEDACCELDECCELPOS		= 66,	// Drive M2 with Speed, Accel, Deccel and Position
	MIXEDSPEEDACCELDECCELPOS	= 67,	// Drive M1 / M2 with Speed, Accel, Deccel and Position
	SETM1DEFAULTACCEL			= 68,	// Set default duty cycle acceleration for M1
	SETM2DEFAULTACCEL			= 69,	// Set default duty cycle acceleration for M2
	SETM1DEFAULTSPEED			= 70,	// Set Default Speed for M1
	SETM2DEFAULTSPEED			= 71,	// Set Default Speed for M2
	READDEFAULTSPEEDS			= 72,	// Read Default Speed Settings
	GETM2DITHER					= 73,	// deprecated
	SETPINFUNCTIONS				= 74,	// Set S3,S4 and S5 Modes
	GETPINFUNCTIONS				= 75,	// Read S3,S4 and S5 Modes
	SETDEADBAND					= 76,	// Set DeadBand for RC/Analog controls
	GETDEADBAND					= 77,	// Read DeadBand for RC/Analog controls
	GETENCODERS					= 78,	// Read Encoders Counts
	GETISPEEDS					= 79,	// Read Raw Motor Speeds
	RESTOREDEFAULTS				= 80,	// Restore Defaults
	GETDEFAULTACCEL				= 81,	// Read Default Duty Cycle Accelerations
	GETTEMP						= 82,	// Read Temperature
	GETTEMP2					= 83,	// Read Temperature 2

	GETERROR					= 90,	// Read Status
	GETENCODERMODE				= 91,	// Read Encoder Modes
	SETM1ENCODERMODE			= 92,	// Set Motor 1 Encoder Mode
	SETM2ENCODERMODE			= 93,	// Set Motor 2 Encoder Mode
	WRITENVM					= 94,	// Write Settings to EEPROM
	READNVM						= 95,	// Read Settings from EEPROM

	SETCONFIG					= 98,	// Set Standard Config Settings
	GETCONFIG					= 99,	// Read Standard Config Settings
	SETCTRLSMODE				= 100,	// Set CTRL Modes
	GETCTRLSMODE				= 101,	// Read CTRL Modes
	SETCTRL1					= 102,	// Set CTRL1
	SETCTRL2					= 103,	// Set CTRL2
	GETCTRLS					= 104,	// Read CTRLs
	SETM1AUTOHOMEDUTYSPEEDTMO	= 105,	// Set Auto Home Duty/Speed and Timeout M1
	SETM2AUTOHOMEDUTYSPEEDTMO	= 106,	// Set Auto Home Duty/Speed and Timeout M2
	GETAUTOHOMESETTINGS			= 107,	// Read Auto Home Settings
	GETMOTORAVERAGESPEEDS		= 108,	// Read Motor Average Speeds
	SETSPEEDERRORLIMITS			= 109,	// Set Speed Error Limits
	GETSPEEDERRORLIMITS			= 110,	// Read Speed Error Limits
	GETSPEEDERRORS				= 111,	// Read Speed Errors
	SETPOSOERRORLIMITS			= 112,	// Set Position Error Limits
	DETPOSOERRORLIMITS			= 113,	// Read Position Error Limits
	GETPOSERRORS				= 114,	// Read Position Errors
	SETBATVOLTAGEOFFSETS		= 115,	// Set Battery Voltage Offsets
	GETBATVOLTAGEOFFSETS		= 116,	// Read Battery Voltage Offsets
	SETBLANKINGPERCENTAGES		= 117,	// Set Current Blanking Percentages
	GETBLANKINGPERCENTAGES		= 118,	// Read Current Blanking Percentages
	DRIVEM1POS					= 119,	// Drive M1 with Position
	DRIVEM2POS					= 120,	// Drive M2 with Position
	DRIVEMIXEDPOS				= 121,	// Drive M1/M2 with Position
	M1SPEEDPOS					= 122,	// Drive M1 with Speed and Position
	M2SPEEDPOS					= 123,	// Drive M2 with Speed and Position
	MIXEDSPEEDPOS				= 124,	// Drive M1/M2 with Speed and Postion

	SETM1LR						= 128,
	SETM2LR						= 129,
	GETM1LR						= 130,
	GETM2LR						= 131,
	CALIBRATELR					= 132,
	SETM1MAXCURRENT				= 133,	// Set M1 Maximum Current
	SETM2MAXCURRENT				= 134,	// Set M2 Maximum Current
	GETM1MAXCURRENT				= 135,	// Read M1 Maximum Current
	GETM2MAXCURRENT				= 136,	// Read M2 Maximum Current
	SETDOUT						= 137,
	GETDOUTS					= 138,
	SETPRIORITY					= 139,
	GETPRIORITY					= 140,
	SETADDRESSMIXED				= 141,
	GETADDRESSMIXED				= 142,
	SETSIGNAL					= 143,
	GETSIGNALS					= 144,
	SETSTREAM					= 145,
	GETSTREAMS					= 146,
	GETSIGNALSDATA				= 147,
	SETPWMMODE					= 148,	// Set PWM Mode
	GETPWMMODE					= 149,	// Read PWM Mode
	SETNODEID					= 150,
	GETNODEID					= 151,

	RESETSTOP					= 200,
	SETESTOPLOCK				= 201,
	GETESTOPLOCK				= 202,

	SETSCRIPTAUTORUN			= 246,
	GETSCRIPTAUTORUN			= 247,
	STARTSCRIPT					= 248,
	STOPSCRIPT					= 249,

	GETUSEREEPROMLOCATION		= 252,	// Read User EEPROM Memory Location
	SETUSEREEPROMLOCATION		= 253,	// Write User EEPROM Memory Location
	READSCRIPT					= 254,
	FLAGBOOTLOADER				= 255
};	// Only available via USB communications

class CRoboclaw
{
public:
	explicit CRoboclaw(uint16_t motor);
	~CRoboclaw(void);

	void startMove(long time);

	bool waitCompletion(void);

	long getPosition(void);

	void Reset(void)
	{
		m_motor = 0;
		m_speed = 0;
		m_position = 0;
		m_target = 0;
		m_accel = 0;
		m_decel = 0;
		m_microSteps = 0;
		m_motorSteps = 0;
		m_corridor = 0;
		m_filter = 10;
	}

	void setSteps(int motorsteps)
	{
		m_motorSteps = motorsteps;
	}
	void setSteps(const char* motorsteps)
	{
		setSteps(stoi(motorsteps));
	}

	void setMicrostep(int microsteps)
	{
		m_microSteps = microsteps;
	}
	void setMicrostep(const char* microsteps)
	{
		setMicrostep(stoi(microsteps));
	}

	void setTarget(const char* target)
	{
		m_target = stol(target);
	}

	void setSpeed(int speed)
	{
		m_speed = speed;
	}
	void setSpeed(const char* speed)
	{
		setSpeed(stol(speed));
	}

	void setAcceleration(const char* steps)
	{
		m_accel = stol(steps);
	}

	void setDeceleration(const char* steps)
	{
		m_decel = stol(steps);
	}

	void setCorridor(const char* corridor)
	{
		m_corridor = stoi(corridor);
	}

	void setFilter(const char* filter)
	{
		m_filter = stoi(filter);
	}

private:

	bool initMotors(void);

	/*
	 * All the commands return:
	 * - ROBOCLAW_OK on success
	 * - ROBOCLAW_ERROR on IO error with errno set
	 * - ROBOCLAW_RETRIES_EXCEEDED if sending didn't result in reply or the checksum didn't match `replies` times (configurable)
	 *
	 * The failure codes are guarateed to have negative values
	 * so in simplest scenario it is enough to test for ROBOCLAW_OK
	 *
	 * All the commands take parametrs:
	 * - m_rc - pointer to library internal data
	 * - address - the address of roboclaw that should receive command (from 0x80 to 0x87)
	 * - ...
	 * - and command specific parameters
	 */

	//*****************************************************************************
	// ACKed commands
	//*****************************************************************************
	int roboclaw_reset(void);
	int roboclaw_set_encoder_m1(uint32_t count);
	int roboclaw_set_encoder_m2(uint32_t count);
	int roboclaw_set_pid_m1(uint32_t D, uint32_t P, uint32_t I, uint32_t QPPS);
	int roboclaw_set_pid_m2(uint32_t D, uint32_t P, uint32_t I, uint32_t QPPS);
	int roboclaw_set_pos_pid_m1(uint32_t D, uint32_t P, uint32_t I, uint32_t MaxI, uint32_t Deadzone, int32_t MinPos, int32_t MaxPos);
	int roboclaw_set_pos_pid_m2(uint32_t D, uint32_t P, uint32_t I, uint32_t MaxI, uint32_t Deadzone, int32_t MinPos, int32_t MaxPos);
	int roboclaw_speed_accel_decel_pos_m1(uint32_t accel, uint32_t speed, uint32_t deccel, int32_t pos);
	int roboclaw_speed_accel_decel_pos_m2(uint32_t accel, uint32_t speed, uint32_t deccel, int32_t pos);

	//*****************************************************************************
	// Commands with reply
	//*****************************************************************************
	int roboclaw_read_version(void);
	int roboclaw_read_encoder(int32_t* count, uint8_t* status, uint8_t cmd);

	//*****************************************************************************
	// encode commands functions
	//*****************************************************************************
	int encode_reset(uint8_t* buffer) const;
	int encode_set_encoder(uint8_t* buffer, uint32_t count, uint8_t cmd) const;
	int encode_set_pid(uint8_t* buffer, uint32_t D, uint32_t P, uint32_t I, uint32_t QPPS, uint8_t cmd) const;
	int encode_set_pos_pid(uint8_t* buffer, uint32_t D, uint32_t P, uint32_t I, uint32_t MaxI, uint32_t Deadzone, int32_t MinPos, int32_t MaxPos, uint8_t cmd) const;
	int encode_speed_accel_deccel_pos(uint8_t* buffer, uint32_t accel1, uint32_t speed1, uint32_t deccel1, int32_t pos1, uint8_t cmd) const;

	//*****************************************************************************
	// encode read commands functions
	//*****************************************************************************
	int encode(uint8_t* buffer, uint8_t cmd) const;
	static void decode_read_encoder(uint8_t* buffer, int32_t* enc, uint8_t* status);

	static int encode_uint16(uint8_t* buffer, int bytes, uint16_t value);
	static uint16_t decode_uint16_t(uint8_t* buffer);
	static int encode_uint32(uint8_t* buffer, int bytes, uint32_t value);
	static uint32_t decode_uint32_t(uint8_t* buffer);

	//*****************************************************************************
	// config-file functions
	//*****************************************************************************
	bool readConfigFile(void);
	string getConfigValue(string key);
	bool parseXMLFile(void);
	static string getValue(string line);
	static void ignore(ifstream* xmlFile, string pattern);

	int		m_motor;		// motor number (0, 1)
	long	m_position;		// current position
	long	m_target;		// target position
	long	m_accel;		// acceleration
	long	m_decel;		// deceleration
	long	m_speed;		// cruising speed
	int		m_corridor;		// stopping area
	int		m_filter;		// min. number of stop counts 
	int		m_microSteps;	// current microstep level (1,2,4,8,...)
	int		m_motorSteps;	// motor steps per revolution (usually 200)

	// global variables
	uint8_t				m_address;		// address of roboclaw unit
	string				m_version;		// firmware version
	map<string, string>	m_config_map;	// configuration map
	uint8_t				sndbuf[ROBOCLAW_BUFFER_SIZE];	// send buffer
	uint8_t				recbuf[ROBOCLAW_BUFFER_SIZE];	// receive buffer
};
#endif	// ROBOCLAW_H