#ifndef ESCBUS_TYPES_H
#define ESCBUS_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Note: keep this length
#define ESCBUS_VERSION				"YUNEEC ESCBUS 1.0"
#define ESC_SOFTWARE_VERSION		"YUNEEC ESC 1.0   "
#define ESC_HARDWARE_VERSION		"YUNEEC ESC FCOY  "

#define ESCBUS_VERSION_LENGTH			sizeof(ESCBUS_VERSION)
#define ESC_SOFTWARE_VERSION_LENGTH		sizeof(ESC_SOFTWARE_VERSION)
#define ESC_HARDWARE_VERSION_LENGTH		sizeof(ESC_HARDWARE_VERSION)

/*
 * Specify what kinds of sensors the ESC has
 */
#define ESC_HAVE_VOLTAGE_SENSOR 0
#define ESC_HAVE_CURRENT_SENSOR 0
#define ESC_HAVE_TEMPERATURE_SENSOR 0

/*
 * Channel info definition
 */
#define MAX_CHANNEL_AVAILABLE	6
#define CH1		0
#define CH2		1
#define CH3		2
#define CH4		3
#define CH5		4
#define CH6		5
#define CH7		6
#define CH8		7

/*
 * Airframe info definition
 */
#define HEXA_FRAME
/*  options:
 *  QUAD_FRAME
 *  TRI_FRAME
 *  HEXA_FRAME
 *  Y6_FRAME
 *  OCTA_FRAME
 *  OCTA_QUAD_FRAME
 *  HELI_FRAME
 *  SINGLE_FRAME
 *  COAX_FRAME
 */

/*
 * Channel map table definition
 *
 * Specify channel ID according to hardware ID on ESC.
 * If one ESC is not in use, type "-1" to disable it.
 * For example, map table for quad:
 * 	ESC_ID		0	1	2	3	4	5
 * 	CH_ID		0	1	2	3	-1	-1
 */
#if	defined(QUAD_FRAME)
	#define ESCBUS_CHANNEL_MAP_TABLE	{0, 1, 2, 3, -1, -1}
#elif defined(HEXA_FRAME)
	#define ESCBUS_CHANNEL_MAP_TABLE	{2, 1, 5, 3, 0, 4}
#elif defined(OCTA_FRAME)
	#define ESCBUS_CHANNEL_MAP_TABLE	{}
#elif defined(OCTA_QUAD_FRAME)
	#define ESCBUS_CHANNEL_MAP_TABLE	{}
#endif

/*
 * ESCBUS types definition
 */
// Enable ESCBus check message length by default
#ifndef ESCBUS_CHECK_MESSAGE_LENGTH
#define ESCBUS_CHECK_MESSAGE_LENGTH 1
	#define ESCBUS_MESSAGE_LENGTHS		{ESCBUS_MESSAGE_LENGTH_CONFIG_BASIC, ESCBUS_MESSAGE_LENGTH_CONFIG_FULL, ESCBUS_MESSAGE_LENGTH_RUN, ESCBUS_MESSAGE_LENGTH_TUNE, ESCBUS_MESSAGE_LENGTH_DO_CMD,  ESCBUS_MESSAGE_LENGTH_REQUEST_INFO, ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_BASIC, ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_FULL, ESCBUS_MESSAGE_LENGTH_RUN_INFO, ESCBUS_MESSAGE_LENGTH_STUDY_INFO, ESCBUS_MESSAGE_LENGTH_COMM_INFO, ESCBUS_MESSAGE_LENGTH_DEVICE_INFO, ESCBUS_MESSAGE_LENGTH_ASSIGNED_ID}
#endif


#define ESCBUS_MAX_DATA_LEN 64
#define ESCBUS_NUM_CRC_BYTES 1
// find the nearest data field size to contain both data and CRC value in uint8_t buffer
#define ESCBUS_DATA_CRC_LEN	(ESCBUS_MAX_DATA_LEN + ESCBUS_NUM_CRC_BYTES)

/*
 * The state machine for the char parser
 */
typedef enum {
    ESCBUS_PARSE_STATE_UNINIT=0,
    ESCBUS_PARSE_STATE_IDLE,
    ESCBUS_PARSE_STATE_GOT_START,
    ESCBUS_PARSE_STATE_GOT_MSGID,
    ESCBUS_PARSE_STATE_GOT_LENGTH,
    ESCBUS_PARSE_STATE_GOT_DATA,
} ESCBUS_ENUM_PARSE_STATE;

/*
 * The bus status for the char parser
 */
typedef struct {
	uint8_t msgReceived;	// Number of received messages
    uint8_t bufferOverrun;	// Number of buffer overruns
    uint8_t parseError;		// Number of parse errors
    ESCBUS_ENUM_PARSE_STATE parseState;	// Parsing state machine
    uint8_t dataIndex;	// Index of data field during parsing
} EscbusStatusType;

/*
 * This is the data format of ESCBus message.
 * ESCBus begins with a specific start sign which is 0xFE, follow that is one ID number for specifying type of message.
 * Then comes the length of data field which can be used to locate the CRC byte easily. CRC is calculated for the whole
 * message except the start sign. It is recommended that hardware CRC unit should be used instead of software if it's
 * supported by your MCU.
 *
 * Here is the graph of ESCBus protocol:
 *  -----------------------------------------------------------------------
 *  | START(0XFE) |   MSG ID   |   LENTH   | --- DATA FIELD --- |   CRC   |
 *  -----------------------------------------------------------------------
 *
 * Note: We should check the length of data field according to its ID number, or you can't get the right position of CRC byte
 * if the length byte was corrupted.
 */

/*
 * ESCBUS Message ID definition
 */
typedef enum {
	// messages or command to ESC
	ESCBUS_MSG_ID_CONFIG_BASIC = 0,
	ESCBUS_MSG_ID_CONFIG_FULL,
	ESCBUS_MSG_ID_RUN,
	ESCBUS_MSG_ID_TUNE,
	ESCBUS_MSG_ID_DO_CMD,
	ESCBUS_MSG_ID_REQUEST_INFO,
	// messages from ESC
	ESCBUS_MSG_ID_CONFIG_INFO_BASIC,	// simple configuration info for request from flight controller
	ESCBUS_MSG_ID_CONFIG_INFO_FULL,		// full configuration info for request from host such as computer
	ESCBUS_MSG_ID_RUN_INFO,				// feedback message in RUN mode
	ESCBUS_MSG_ID_STUDY_INFO,			// studied parameters in STUDY mode
	ESCBUS_MSG_ID_COMM_INFO,			// communication method info
	ESCBUS_MSG_ID_DEVICE_INFO,			// ESC device info
	ESCBUS_MSG_ID_ASSIGNED_ID,
	// never touch ESCBUS_MSG_ID_MAX_NUM
	ESCBUS_MSG_ID_MAX_NUM
} ESCBUS_ENUM_MESSAGE_ID;

/*
 * ESCBUS message type
 */
typedef struct {
	uint8_t start;	// start sign for a message package
	uint8_t length;	// length of data field
	uint8_t msgid;	// ID for this message
	uint8_t data[ESCBUS_DATA_CRC_LEN]; // length of data field is 255 and plus one byte for CRC
} EscbusMessageType;

/******************************************************************************************
 * ESCBUS_MSG_ID_CONFIG_BASIC packet
 *
 * This is the data structure for selecting types of feedback message of ESCs and
 * configuring motor control mode.
 *
 * maxChannelInUse: set maximum number of channels in use according to airframe
 * channelMapTabel: bit0-3 mapping table from hardware ID to channel number for each ESC
 * 					bit4-7 set direction of motor spin, 0 for CW, 1 for CCW
 *
 * monitorMsgType: enable the corresponding type of feedback message to monitor
 * 		Bit		Name			Range				Accuracy		Unit
 * 		0		speed			-32767 - +32768		1				rpm
 * 		1		voltage			0.00 - 100.00		0.01			V
 * 		2		current			0.0 - 200.0			0.1				A
 * 		3		temperature		0 - 255				1				celsius degree
 * 		4-7		Reserved
 *
 * controlMode: specify motor control method.
 * 		Value		Mode
 * 		0 			open loop (default)
 * 		1 			close loop without break
 * 		2			close loop with break
 * 		others		Reserved
 *
 */
// channelMapTable bit mask
#define CHANNEL_MAP_TABLE_CHANNEL_NUM	0x0f
#define CHANNEL_MAP_TABLE_DIRECTION		0xf0

// motor spin direction definition
typedef enum {
	CW = 0,
	CCW
} ESCBUS_ENUM_SPIN_DIRECTION;

// monitor message type enable mask
#define MONITOR_MSG_TYPE_SPEED_MASK				((uint8_t)1 << 0)
#define MONITOR_MSG_TYPE_VOLTAGE_MASK			((uint8_t)1 << 1)
#define MONITOR_MSG_TYPE_CURRENT_MASK			((uint8_t)1 << 2)
#define MONITOR_MSG_TYPE_TEMPERATURE_MASK		((uint8_t)1 << 3)

// control mode definition
typedef enum {
	CONTROL_MODE_OPEN_LOOP,
	CONTROL_MODE_CLOSE_LOOP_NO_BREAK,
	CONTROL_MODE_CLOSE_LOOP_WITH_BREAK,
} ESCBUS_ENUM_CONTROL_MODE;

// the real packet definition for ESCBUS_MSG_ID_CONFIG_BASIC
typedef struct {
	uint8_t maxChannelInUse;
	int8_t channelMapTable[MAX_CHANNEL_AVAILABLE];
	uint8_t monitorMsgType;
	uint8_t controlMode;
} EscbusConfigBasicPacket;

#define ESCBUS_MESSAGE_LENGTH_CONFIG_BASIC	sizeof(EscbusConfigBasicPacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_CONFIG_FULL packet
 *
 * This is the data structure for selecting types of feedback message
 * of ESCs and configuring motor control mode and many other parameters.
 *
 * EscbusConfigBasicPacket: see above
 * minChannelValue: minimum channel value mapping to minSpeed
 * maxChannelValue:	maximum channel value mapping to maxSpeed
 * minSpeedSet: set minimum speed while given channel value is minChannelValue
 * maxSpeedSet: set maximum speed while given channel value is maxChannelValue
 * breakLevel: level of break while channel value is below minChannelValue
 * cutVoltage: cut ESC output while voltage is below this value
 * limitCurrent: maximum output current to motor
 * advanceAngle: angle of advance
 * freqOfPWM: frequency of PWM
 * polesNum: number of poles
 * PID_P: P parameter of PID controller
 * PID_I: I parameter of PID controller
 */

// the real packet definition for ESCBUS_MSG_ID_CONFIG_FULL
typedef struct {
	EscbusConfigBasicPacket	basicConfig;
	uint16_t minChannelValue;
	uint16_t maxChannelValue;
	int16_t minSpeedSet;
	int16_t maxSpeedSet;
	uint8_t breakLevel;
	uint16_t cutVoltage;
	uint8_t limitCurrent;
	uint8_t advanceAngle;
	uint16_t freqOfPWM;
	uint8_t polesNum;
	float PID_P;
	float PID_I;
} EscbusConfigFullPacket;

#define ESCBUS_MESSAGE_LENGTH_CONFIG_FULL	sizeof(EscbusConfigFullPacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_RUN packet
 *
 * This is the compressed data for controlling motor speed.
 * 	bit			definition
 * 	0-10		channel value (0 - 2048)
 * 	11 			red led turn on if 1
 * 	12			green led turn on if 1
 * 	13			blue led turn on if 1
 * 	14			feedback enable if 1
 * 	15			reverse if 1
 * We compress packet of running state into two bytes per channel to save transmit
 * time for real-time motor control.
 */

// bit mask of ESCBUS_MSG_ID_RUN packet
#define RUN_CHANNEL_VALUE_MASK		(uint16_t)0x07ff
#define RUN_RED_LED_ON_MASK			((uint16_t)1 << 11)
#define RUN_GREEN_LED_ON_MASK		((uint16_t)1 << 12)
#define RUN_BLUE_LED_ON_MASK		((uint16_t)1 << 13)
#define RUN_LED_ON_MASK				(RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK)
#define RUN_FEEDBACK_ENABLE_MASK	((uint16_t)1 << 14)
#define RUN_REVERSE_MASK			((uint16_t)1 << 15)

// LED state definition
typedef enum {
	RUN_RED_LED_ON,
	RUN_GREEN_LED_ON,
	RUN_BLUE_LED_ON,
	RUN_LED_ON,
} ESCBUS_ENUM_LED_STATE;

// the real packet definition for ESCBUS_MSG_ID_RUN
typedef struct {
	int16_t value[MAX_CHANNEL_AVAILABLE];
} EscbusRunPacket;

//XXX This length should be updated according to maximum number of channels be used
#define ESCBUS_MESSAGE_LENGTH_RUN	sizeof(EscbusRunPacket)

/*
 * ESCBUS_MSG_ID_TUNE packet
 *
 * This is a command to set ESC in TUNE mode.
 *
 * frequency: motor frequency
 * counter: motor shake times at certain frequency
 * strength: strength of tune
 */

// the real packet definition for ESCBUS_MSG_ID_TUNE
typedef struct {
	uint16_t frequency; // 0 - 20kHz
	uint16_t counter;
	uint8_t strength;
} EscbusTunePacket;

#define ESCBUS_MESSAGE_LENGTH_TUNE	sizeof(EscbusTunePacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_DO_CMD packet
 *
 * This is a structure of commands for each channels.
 *
 * channelIDMask: enable channel to do command
 * Command:
 * 	DO_RESET: reset channel
 * 	DO_STUDY: enter study mode
 * 	DO_ID_ASSIGNMENT: enter ID assignment mode
 *
 */

// bit mask of channel ID
#define DO_CMD_CHANNEL1		((uint8_t)1 << CH1)
#define DO_CMD_CHANNEL2		((uint8_t)1 << CH2)
#define DO_CMD_CHANNEL3		((uint8_t)1 << CH3)
#define DO_CMD_CHANNEL4		((uint8_t)1 << CH4)
#define DO_CMD_CHANNEL5		((uint8_t)1 << CH5)
#define DO_CMD_CHANNEL6		((uint8_t)1 << CH6)
#define DO_CMD_CHANNEL7		((uint8_t)1 << CH7)
#define DO_CMD_CHANNEL8		((uint8_t)1 << CH8)

// command definition
typedef enum {
	DO_RESET = 0,
	DO_STUDY,
	DO_ID_ASSIGNMENT,
} ESCBUS_ENUM_COMMAND;

// the real packet definition for ESCBUS_MSG_ID_DO_CMD
typedef struct {
	uint8_t channelIDMask;
	uint8_t command;
} EscbusDoCmdPacket;

#define ESCBUS_MESSAGE_LENGTH_DO_CMD	sizeof(EscbusDoCmdPacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_REQUEST_INFO packet
 *
 * This is a command to ask one specific channel to send back configuration info.
 *
 * channelID: which channel to send back info
 * requestBIt: enable the corresponding type of info to be sent back, user can request
 * multiple types of info by set bit
 *
 *	Bit			Request Info
 *	0			REQUEST_INFO_CONFIG_BASIC
 *	1			REQUEST_INFO_CONFIG_FULL
 *	2			REQUEST_INFO_RUN
 *	3			REQUEST_INFO_STUDY
 *	4			REQUEST_INFO_COMM
 *	5			REQUEST_INFO_DEVICE
 *
 */

// request info bit mask
#define REQUEST_INFO_CONFIG_BASIC_MASK		((uint8_t)1 << 0)
#define REQUEST_INFO_CONFIG_FULL_MASK		((uint8_t)1 << 1)
#define REQUEST_INFO_RUN_MASK				((uint8_t)1 << 2)
#define REQUEST_INFO_STUDY_MASK				((uint8_t)1 << 3)
#define REQUEST_INFO_COMM_MASK				((uint8_t)1 << 4)
#define REQUEST_INFO_DEVICE_MASK			((uint8_t)1 << 5)

// the real packet definition for ESCBUS_MSG_ID_REQUEST_INFO
typedef struct {
	uint8_t channelID;
	uint8_t requestBit;
} EscbusRequestInfoPacket;

#define ESCBUS_MESSAGE_LENGTH_REQUEST_INFO	sizeof(EscbusRequestInfoPacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_CONFIG_INFO_BASIC packet
 *
 * Basic configuration info of ESC
 *
 * channelID: specify which channel send this message
 * maxChannelInUse: see above, every ESC should have the same value
 * channelMapTabel: see above, every ESC should have the same value
 * monitorMsgType: enable the corresponding type of feedback message to monitor
 * 		Bit		Name			Range				Accuracy		Unit
 * 		0		speed			-32767 - +32768		1				rpm
 * 		1		voltage			0.00 - 100.00		0.01			V
 * 		2		current			0.0 - 200.0			0.1				A
 * 		3		temperature		0 - 255				1				celsius degree
 * 		4-7		Reserved
 *
 * controlMode: specify motor control method, 0 for open loop (default), 1 for close loop
 *
 */

// the real packet definition for ESCBUS_MSG_ID_CONFIG_INFO_BASIC
typedef struct {
	uint8_t channelID;
	uint8_t maxChannelInUse;
	uint8_t channelMapTable[MAX_CHANNEL_AVAILABLE];
	uint8_t monitorMsgType;
	uint8_t controlMode;
} EscbusConfigInfoBasicPacket;

#define ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_BASIC	sizeof(EscbusConfigInfoBasicPacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_CONFIG_INFO_FULL packet
 *
 * Full configuration info of ESC
 *
 * EscbusConfigInfoBasicPacket: see above
 * minChannelValue: minimum channel value mapping to minSpeed
 * maxChannelValue:	maximum channel value mapping to maxSpeed
 * minSpeedSet: set minimum speed while given channel value is minChannelValue
 * maxSpeedSet: set maximum speed while given channel value is maxChannelValue
 * breakLevel: level of break while channel value is below minChannelValue
 * cutVoltage: cut ESC output while voltage is below this value
 * limitCurrent: maximum output current to motor
 * advanceAngle: angle of advance
 * freqOfPWM: frequency of PWM
 * polesNum: number of poles
 * PID_P: P parameter of PID controller
 * PID_I: I parameter of PID controller
 */

// the real packet definition for ESCBUS_MSG_ID_CONFIG_INFO_FULL
typedef struct {
	EscbusConfigInfoBasicPacket configInfoBasic;
	uint16_t minChannelValue;
	uint16_t maxChannelValue;
	int16_t minSpeedSet;
	int16_t maxSpeedSet;
	uint8_t breakLevel;
	uint16_t cutVoltage;
	uint8_t limitCurrent;
	uint8_t advanceAngle;
	uint16_t freqOfPWM;
	uint8_t polesNum;
	float PID_P;
	float PID_I;
} EscbusConfigInfoFullPacket;

#define ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_FULL	sizeof(EscbusConfigInfoFullPacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_RUN_INFO packet
 *
 * Monitor message of ESCs while motor is running
 *
 * ID_MotorStatus: bit0-3 assigned channel number
 * 				   bit4-7 motor status
 *
 * 	bit4-7	Motor status
 * 	0		MOTOR_STOP
 * 	1		MOTOR_ACCEL
 * 	2		MOTOR_RUNNIG
 *  3		MOTOR_ERROR_LOSE_STEP
 *
 * ESCStatus: status of ESC
 * 	Bit		Health status
 * 	0		HEALTHY
 * 	1		WARNING_LOW_VOLTAGE
 * 	2		WARNING_OVER_CURRENT
 * 	3		WARNING_OVER_HEAT
 *	4		ERROR_CURRENT_PEAK
 *	5		ERROR_MOTOR_LOSE_STEP
 *
 * speed: -32767 - 32768 rpm
 * temperature: 0 - 256 celsius degree
 * voltage: 0.00 - 100.00 V
 * current: 0.0 - 200.0 A
 */

// ID_MotorStatus bit mask
#define ESCBUS_CHANNEL_ID_MASK		0x0f
#define ESCBUS_MOTOR_STATUS_MASK	0xf0
#define ESCBUS_MOTOR_STATUS_BITS	4

// convenience define for set or get channel id and motor status
#define ESCBUS_GET_CHANNEL_ID(packet)				(uint8_t)((packet)->ID_MotorStatus & ESCBUS_CHANNEL_ID_MASK)
#define ESCBUS_GET_MOTOR_STATUS(packet)				(uint8_t)(((packet)->ID_MotorStatus & ESCBUS_MOTOR_STATUS_MASK) >> ESCBUS_MOTOR_STATUS_BITS)
#define ESCBUS_SET_CHANNEL_ID(packet, id)			(packet)->ID_MotorStatus &= ESCBUS_MOTOR_STATUS_MASK;\
													(packet)->ID_MotorStatus |= (id & ESCBUS_CHANNEL_ID_MASK)
#define ESCBUS_SET_MOTOR_STATUS(packet, status)		(packet)->ID_MotorStatus &= ESCBUS_CHANNEL_ID_MASK;\
													(packet)->ID_MotorStatus |= (status << ESCBUS_MOTOR_STATUS_BITS)

// motor status definition
typedef enum {
	MOTOR_STOP,
	MOTOR_ACCEL,
	MOTOR_RUNNING,
	MOTOR_ERROR_LOSE_STEP
} ESCBUS_ENUM_MOTOR_STATUS;

// status of ESC definition
#define ESC_STATUS_HEALTHY_MASK					((uint8_t)1 << 0)
#define ESC_STATUS_WARNING_LOW_VOLTAGE_MASK		((uint8_t)1 << 1)
#define ESC_STATUS_WARNING_OVER_CURRENT_MASK	((uint8_t)1 << 2)
#define ESC_STATUS_WARNING_OVER_HEAT_MASK		((uint8_t)1 << 3)
#define ESC_STATUS_ERROR_CURRENT_PEAK_MASK		((uint8_t)1 << 4)
#define ESC_STATUS_ERROR_MOTOR_LOSE_STEP_MASK	((uint8_t)1 << 5)

// the real packet definition for ESCBUS_MSG_ID_RUN_INFO
typedef struct {
	uint8_t ID_MotorStatus;
	uint8_t ESCStatus;
	int16_t speed;			// -32767 - 32768 rpm
#if ESC_HAVE_VOLTAGE_SENSOR
	uint16_t voltage;		// 0.00 - 100.00 V
#endif
#if ESC_HAVE_CURRENT_SENSOR
	uint16_t current;		// 0.0 - 200.0 A
#endif
#if ESC_HAVE_TEMPERATURE_SENSOR
	uint8_t temperature;	// 0 - 256 celsius degree
#endif
} EscbusRunInfoPacket;

#define ESCBUS_MESSAGE_LENGTH_RUN_INFO	sizeof(EscbusRunInfoPacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_STUDY_INFO packet
 *
 * Study info of ESC while studying
 *
 * channelID: specify which channel send this message
 * motorMinSpeed: available minimum speed of motor, -32768 to + 32768 rpm
 * motorMaxSpeed: available maximum speed of motor, -32768 to + 32768 rpm
 */

// the real packet definition for ESCBUS_MSG_ID_STUDY_INFO
typedef struct {
	uint8_t channelID;
	int16_t motorMinSpeed;
	int16_t motorMaxSpeed;
} EscbusStudyInfoPacket;

#define ESCBUS_MESSAGE_LENGTH_STUDY_INFO	sizeof(EscbusStudyInfoPacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_COMM_INFO packet
 *
 * Hardware and software version of ESC and ESCBUS version
 *
 * hardwareVersion: show the hardware version of this ESC
 * softwareVersion: show the software version of this ESC
 * ESCBUS_Version: show the current version of ESCBUS protocol
 * inputMode: show the current communication method
 * baudRate: show the current baud rate
 */

// input mode definition
typedef enum {
	INPUT_MODE_UART = 0,
	INPUT_MODE_PWM,
	INPUT_MODE_CAN,
	INPUT_MODE_I2C
} ESCBUS_ENUM_INPUT_MODE;

// uart baud rate definition
typedef enum {
	BAUDRATE_DEFAULT,
	BAUDRATE_1000000,
	BAUDRATE_500000,
	BAUDRATE_250000,
	BAUDRATE_115200,
	BAUDRATE_9600,
} ESCBUS_ENUM_BAUDRATE;

// the real packet definition for ESCBUS_MSG_ID_COMM_INFO
typedef struct {
	uint8_t channelID;
	uint8_t inputMode;
	uint8_t baudRate;
} EscbusCommInfoPacket;

#define ESCBUS_MESSAGE_LENGTH_COMM_INFO	sizeof(EscbusCommInfoPacket)

/******************************************************************************************
 * ESCBUS_MSG_ID_DEVICE_INFO packet
 *
 * Hardware and software version of ESC and ESCBUS version
 *
 * hardwareVersion: show the hardware version of this ESC
 * softwareVersion: show the software version of this ESC
 * ESCBUS_Version: show the current version of ESCBUS protocol
 * inputMode: show the current communication method
 * baudRate: show the current baud rate
 */

// the real packet definition for ESCBUS_MSG_ID_DEVICE_INFO
typedef struct {
	uint8_t channelID;
	char* ESC_HardwareVersion;
	char* ESC_SoftwareVersion;
	char* ESCBUS_Version;
} EscbusDeviceInfoPacket;

#define ESCBUS_MESSAGE_LENGTH_DEVICE_INFO	(1 + ESCBUS_VERSION_LENGTH + ESC_SOFTWARE_VERSION_LENGTH + ESC_HARDWARE_VERSION_LENGTH)

/******************************************************************************************
 * ESCBUS_MSG_ID_ASSIGNED_ID packet
 *
 * This is a structure for assignment mode. When one ESC detect motor spin, it should send
 * back its assigned channelID with escID to flight controller or host in order to let FC or
 * host know that one channel has been assigned.
 *
 * channelID: show the channel ID assigned
 * escID: show the ESC hardware ID
 */

// the real packet definition for ESCBUS_MSG_ID_ASSIGNED_ID
typedef struct {
	uint8_t channelID;
	uint8_t escID;
} EscbusAssignedIDPacket;

#define ESCBUS_MESSAGE_LENGTH_ASSIGNED_ID	sizeof(EscbusAssignedIDPacket)

#endif

