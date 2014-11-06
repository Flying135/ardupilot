/*
 * ESCBus.c
 *
 *  Created on: Oct 24, 2014
 *      Author: maelok
 */
#include "ESCBus.h"
#include "CRC8.h"

/*
 * ESCBUS types definition
 */
#define ESCBUS_START 0xFE
#define ESCBUS_NUM_HEADER_BYTES	3
#define ESCBUS_NUM_NON_DATA_FIELD_BYTES (ESCBUS_NUM_HEADER_BYTES + ESCBUS_NUM_CRC_BYTES)

// CRC is immediately after the data bytes
#define _ESCBUS_CRC_BEGIN(msg) ((uint8_t *)(&((msg)->length)))
#define ESCBUS_CRC_BYTE(msg) ((msg).data[(msg).length])

/*
 * Enable this option to check the length of each message.
 * This allows invalid messages to be caught much sooner. Use if the transmission
 * medium is prone to missing (or extra) characters.
 */
#if ESCBUS_CHECK_MESSAGE_LENGTH == 1
	static uint8_t escbus_message_lengths[ESCBUS_MSG_ID_MAX_NUM] = ESCBUS_MESSAGE_LENGTHS;
#define ESCBUS_MESSAGE_LENGTH(msgid) escbus_message_lengths[msgid]
#else
#error "ESCBUS: message length doesn't match, please check ENUM definition of message length!"
#endif

// This holds crc8 table
#ifndef USE_CRC8_TABLE
static uint8_t crcTable[256];
#endif

// This is used by escbus_parse_char
static EscbusMessageType rxmsg = {0};
static EscbusStatusType rxstatus = {0};

/*
 * Software CRC8 calculation
 */
#ifdef ESCBUS_CRC_SOFTWARE
uint8_t _escbus_crc_software(EscbusMessageType *msgbuf, uint8_t length)
{
	return crc_table(_ESCBUS_CRC_BEGIN(msgbuf), (length + 2), crcTable);
}
#endif
/*
 * Hardware CRC8 calculation
 */
#ifdef ESCBUS_CRC_HARDWARE
uint8_t _escbus_crc_hardware(EscbusMessageType *msgbuf, uint8_t length)
{
	// Implement your hardware CRC here
	return 0;
}
#endif

/*************************************************************************************
 * Initiate ESCBus
 */
void escbus_init(uint8_t maxChannelInUse)
{
	// generate CRC8 table if using software CRC
#ifndef	USE_CRC8_TABLE
#if defined(ESCBUS_CRC_SOFTWARE)
	generate_table(CRC8_POLYNOMIAL, crcTable);
#endif
#endif

	// update message lengths table
	if(maxChannelInUse < MAX_CHANNEL_AVAILABLE)
		escbus_message_lengths[ESCBUS_MSG_ID_RUN] = maxChannelInUse << 1;
}

/*************************************************************************************
 * convenience function to send basic config message
 */
void escbus_send_msg_config_basic( uint8_t maxChannelInUse, const int8_t* channelMapTable, uint8_t monitorMsgType, uint8_t controlMode )
{
	EscbusConfigBasicPacket packet;
	packet.maxChannelInUse = maxChannelInUse;
	memcpy(packet.channelMapTable, channelMapTable, MAX_CHANNEL_AVAILABLE);
	packet.monitorMsgType = monitorMsgType;
	packet.controlMode = controlMode;

	escbus_message_send(ESCBUS_MSG_ID_CONFIG_BASIC, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_CONFIG_BASIC);
}

/*************************************************************************************
 * convenience function to send full config message
 */
void escbus_send_msg_config_full( uint8_t maxChannelInUse, int8_t* channelMapTable, uint8_t monitorMsgType, uint8_t controlMode,
								  uint16_t minChannelValue, uint16_t maxChannelValue, int16_t minSpeedSet, int16_t maxSpeedSet, uint8_t breakLevel, uint16_t cutVoltage,
								  uint8_t limitCurrent, uint8_t advanceAngle, uint16_t freqOfPWM, uint8_t polesNum, float PID_P, float PID_I )
{
	EscbusConfigFullPacket packet;
	packet.basicConfig.maxChannelInUse = maxChannelInUse;
	memcpy(packet.basicConfig.channelMapTable, channelMapTable, MAX_CHANNEL_AVAILABLE);
	packet.basicConfig.monitorMsgType = monitorMsgType;
	packet.basicConfig.controlMode = controlMode;
	packet.minChannelValue = minChannelValue;
	packet.maxChannelValue = maxChannelValue;
	packet.minSpeedSet = minSpeedSet;
	packet.maxSpeedSet = maxSpeedSet;
	packet.breakLevel = breakLevel;
	packet.cutVoltage = cutVoltage;
	packet.limitCurrent = limitCurrent;
	packet.advanceAngle = advanceAngle;
	packet.freqOfPWM = freqOfPWM;
	packet.polesNum = polesNum;
	packet.PID_P = PID_P;
	packet.PID_I = PID_I;

	escbus_message_send(ESCBUS_MSG_ID_CONFIG_FULL, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_CONFIG_FULL);
}

/*************************************************************************************
 * convenience function to send run command
 */
void escbus_send_msg_run( int16_t* value, uint8_t channelNum )
{
	EscbusRunPacket packet;
	uint8_t length = channelNum << 1;
	if (length > escbus_message_lengths[ESCBUS_MSG_ID_RUN])
		length = escbus_message_lengths[ESCBUS_MSG_ID_RUN];
	memcpy((uint8_t *)packet.value, (uint8_t *)value, length);
	escbus_message_send(ESCBUS_MSG_ID_RUN, (const char *)&packet, length);
}

/*************************************************************************************
 * convenience function to send tune command
 */
void escbus_send_msg_tune( uint16_t frequency, uint16_t counter, uint8_t strength )
{
	EscbusTunePacket packet;
	packet.frequency = frequency;
	packet.counter = counter;
	packet.strength = strength;

	escbus_message_send(ESCBUS_MSG_ID_TUNE, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_TUNE);
}

/*************************************************************************************
 * convenience function to send study command
 */
void escbus_send_msg_do_cmd( uint8_t channelIDMask, uint8_t command )
{
	EscbusDoCmdPacket packet;
	packet.channelIDMask = channelIDMask;
	packet.command = command;

	escbus_message_send(ESCBUS_MSG_ID_DO_CMD, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_DO_CMD);
}

/*************************************************************************************
 * convenience function to send request info command
 */
void escbus_send_msg_request_info( uint8_t channelID, uint8_t requestBit )
{
	EscbusRequestInfoPacket packet;
	packet.channelID = channelID;
	packet.requestBit = requestBit;

	escbus_message_send(ESCBUS_MSG_ID_REQUEST_INFO, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_REQUEST_INFO);
}

/*************************************************************************************
 * convenience function to send basic config info
 */
void escbus_send_msg_config_info_basic( uint8_t channelID, uint8_t maxChannelInUse, int8_t* channelMapTable, uint8_t monitorMsgType, uint8_t controlMode )
{
	EscbusConfigInfoBasicPacket packet;
	packet.channelID = channelID;
	packet.maxChannelInUse = maxChannelInUse;
	memcpy(packet.channelMapTable, channelMapTable, MAX_CHANNEL_AVAILABLE);
	packet.monitorMsgType = monitorMsgType;
	packet.controlMode = controlMode;

	escbus_message_send(ESCBUS_MSG_ID_CONFIG_INFO_BASIC, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_BASIC);
}

/*************************************************************************************
 * convenience function to send full config info
 */
void escbus_send_msg_config_info_full( uint8_t channelID, uint8_t maxChannelInUse, int8_t* channelMapTable, uint8_t monitorMsgType, uint8_t controlMode,
									   uint16_t minChannelValue, uint16_t maxChannelValue, int16_t minSpeedSet, int16_t maxSpeedSet, uint8_t breakLevel, uint16_t cutVoltage,
									   uint8_t limitCurrent, uint8_t advanceAngle, uint16_t freqOfPWM, uint8_t polesNum, float PID_P, float PID_I )
{
	EscbusConfigInfoFullPacket packet;
	packet.configInfoBasic.channelID = channelID;
	packet.configInfoBasic.maxChannelInUse = maxChannelInUse;
	memcpy(packet.configInfoBasic.channelMapTable, channelMapTable, MAX_CHANNEL_AVAILABLE);
	packet.configInfoBasic.monitorMsgType = monitorMsgType;
	packet.configInfoBasic.controlMode = controlMode;
	packet.minChannelValue = minChannelValue;
	packet.maxChannelValue = maxChannelValue;
	packet.minSpeedSet = minSpeedSet;
	packet.maxSpeedSet = maxSpeedSet;
	packet.breakLevel = breakLevel;
	packet.cutVoltage = cutVoltage;
	packet.limitCurrent = limitCurrent;
	packet.advanceAngle = advanceAngle;
	packet.freqOfPWM = freqOfPWM;
	packet.polesNum = polesNum;
	packet.PID_P = PID_P;
	packet.PID_I = PID_I;

	escbus_message_send(ESCBUS_MSG_ID_CONFIG_INFO_FULL, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_FULL);
}

/*************************************************************************************
 * convenience function to send run info
 */
void escbus_send_msg_run_info( uint8_t ID_MotorStatus, uint8_t ESCStatus, int16_t speed, uint16_t voltage, uint16_t current, uint8_t temperature )
{
	EscbusRunInfoPacket packet;
	packet.ID_MotorStatus = ID_MotorStatus;
	packet.ESCStatus = ESCStatus;
	packet.speed = speed;
#if ESC_HAVE_VOLTAGE_SENSOR
	packet.voltage = voltage;
#endif
#if ESC_HAVE_CURRENT_SENSOR
	packet.current = current;
#endif
#if ESC_HAVE_TEMPERATURE_SENSOR
	packet.temperature = temperature;
#endif

	escbus_message_send(ESCBUS_MSG_ID_RUN_INFO, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_RUN_INFO);
}

/*************************************************************************************
 * convenience function to send study info
 */
void escbus_send_msg_study_info( uint8_t channelID, int16_t motorMinSpeed, int16_t motorMaxSpeed )
{
	EscbusStudyInfoPacket packet;
	packet.channelID = channelID;
	packet.motorMinSpeed = motorMinSpeed;
	packet.motorMaxSpeed = motorMaxSpeed;

	escbus_message_send(ESCBUS_MSG_ID_STUDY_INFO, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_STUDY_INFO);
}

/*************************************************************************************
 * convenience function to send communication method info
 */
void escbus_send_msg_comm_info( uint8_t channelID, uint8_t inputMode, uint8_t baudRate )
{
	EscbusCommInfoPacket packet;
	packet.channelID = channelID;
	packet.inputMode = inputMode;
	packet.baudRate = baudRate;

	escbus_message_send(ESCBUS_MSG_ID_COMM_INFO, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_COMM_INFO);
}

/*************************************************************************************
 * convenience function to send device info
 *
 * This one is different from others for handling string send
 */
void escbus_send_msg_device_info( uint8_t channelID, char* ESC_HardwareVersion, char* ESC_SoftwareVersion, char* ESCBUS_Version )
{
	EscbusMessageType msgbuf;
	uint8_t msgLength = ESCBUS_MESSAGE_LENGTH_DEVICE_INFO + ESCBUS_NUM_NON_DATA_FIELD_BYTES;
	// ESCBus header
	msgbuf.start = ESCBUS_START;
	msgbuf.length = ESCBUS_MESSAGE_LENGTH_DEVICE_INFO;
	msgbuf.msgid = ESCBUS_MSG_ID_DEVICE_INFO;

	// ESCBus fill in data field
	msgbuf.data[0] = channelID;
    memcpy (_ESCBUS_DATA_LOCATE(msgbuf, 1), ESC_HardwareVersion, ESC_HARDWARE_VERSION_LENGTH);
    memcpy (_ESCBUS_DATA_LOCATE(msgbuf, 1 + ESC_HARDWARE_VERSION_LENGTH), ESC_SoftwareVersion, ESC_SOFTWARE_VERSION_LENGTH);
    memcpy (_ESCBUS_DATA_LOCATE(msgbuf, 1 + ESC_HARDWARE_VERSION_LENGTH + ESC_SOFTWARE_VERSION_LENGTH), ESCBUS_Version, ESCBUS_VERSION_LENGTH);

	// Calculate CRC value for the whole message except start sign
#if	defined(ESCBUS_CRC_HARDWARE)
    ESCBUS_CRC_BYTE(msgbuf) = ESCBUS_CRC_HARDWARE(&msgbuf, ESCBUS_MESSAGE_LENGTH_DEVICE_INFO);
#elif defined(ESCBUS_CRC_SOFTWARE)
    ESCBUS_CRC_BYTE(msgbuf) = ESCBUS_CRC_SOFTWARE(&msgbuf, ESCBUS_MESSAGE_LENGTH_DEVICE_INFO);
#else
#error "ESCBUS: Miss hardware or software implement of CRC8 calculation!"
#endif

    // Transmit message package
#if defined(ESCBUS_SEND_UART)
    ESCBUS_SEND_UART(&msgbuf, msgLength);
#elif defined(ESCBUS_SEND_CAN)
    ESCBUS_SEND_CAN(&msgbuf, msgLength);
#else
#error "ESCBUS: Miss instance for sending message!"
#endif
}

/*************************************************************************************
 * convenience function to send assigned channel ID
 */
void escbus_send_msg_assigned_id( uint8_t channelID, uint8_t escID )
{
	EscbusAssignedIDPacket packet;
	packet.channelID = channelID;
	packet.escID = escID;

	escbus_message_send(ESCBUS_MSG_ID_ASSIGNED_ID, (const char *)&packet, ESCBUS_MESSAGE_LENGTH_ASSIGNED_ID);
}

/*************************************************************************************
 * convenience function to handle received message to ESC
 *
 * ESC could implement its handle functions here
 */
void escbus_handle_message(EscbusMessageType* msg)
{
    switch (msg->msgid)
    {
	// message or command to ESC
#ifdef HANDLE_MSG_CONFIG_BASIC
    case ESCBUS_MSG_ID_CONFIG_BASIC:
    {
    	EscbusConfigBasicPacket packet;
    	memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_CONFIG_BASIC);
    	HANDLE_MSG_CONFIG_BASIC(&packet);
    	break;
    }
#endif
#ifdef HANDLE_MSG_CONFIG_FULL
    case ESCBUS_MSG_ID_CONFIG_FULL:
    {
    	EscbusConfigFullPacket	packet;
    	memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_CONFIG_FULL);
    	HANDLE_MSG_CONFIG_FULL(&packet);
    	break;
    }
#endif
#ifdef HANDLE_MSG_RUN
    case ESCBUS_MSG_ID_RUN:
    {
    	EscbusRunPacket packet;
    	memcpy((char* )&packet, _ESCBUS_DATA(msg), msg->length);
    	HANDLE_MSG_RUN(&packet);
    	break;
    }
#endif
#ifdef HANDLE_MSG_TUNE
    case ESCBUS_MSG_ID_TUNE:
    {
    	EscbusTunePacket packet;
    	memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_TUNE);
    	HANDLE_MSG_TUNE(&packet);
    	break;
    }
#endif
#ifdef HANDLE_MSG_DO_CMD
    case ESCBUS_MSG_ID_DO_CMD:
    {
    	EscbusDoCmdPacket packet;
    	memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_DO_CMD);
    	HANDLE_MSG_DO_CMD(&packet);
    	break;
    }
#endif
#ifdef HANDLE_MSG_REQUEST_INFO
    case ESCBUS_MSG_ID_REQUEST_INFO:
    {
    	EscbusRequestInfoPacket packet;
		memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_REQUEST_INFO);
		HANDLE_MSG_REQUEST_INFO(&packet);
		break;
    }
#endif
	// message from ESC
#ifdef HANDLE_MSG_CONFIG_INFO_BASIC
    case ESCBUS_MSG_ID_CONFIG_INFO_BASIC:
    {
    	EscbusConfigInfoBasicPacket packet;
		memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_BASIC);
		HANDLE_MSG_CONFIG_INFO_BASIC(&packet);
		break;
    }
#endif
#ifdef HANDLE_MSG_CONFIG_INFO_FULL
    case ESCBUS_MSG_ID_CONFIG_INFO_FULL:
    {
    	EscbusConfigInfoFullPacket packet;
		memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_FULL);
		HANDLE_MSG_CONFIG_INFO_FULL(&packet);
		break;
    }
#endif
#ifdef HANDLE_MSG_RUN_INFO
    case ESCBUS_MSG_ID_RUN_INFO:
    {
    	EscbusRunInfoPacket packet;
		memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_RUN_INFO);
		HANDLE_MSG_RUN_INFO(&packet);

		break;
    }
#endif
#ifdef HANDLE_MSG_STUDY_INFO
    case ESCBUS_MSG_ID_STUDY_INFO:
    {
    	EscbusStudyInfoPacket packet;
		memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_STUDY_INFO);
		HANDLE_MSG_STUDY_INFO(&packet);
		break;
    }
#endif
#ifdef HANDLE_MSG_COMM_INFO
    case ESCBUS_MSG_ID_COMM_INFO:
    {
    	EscbusCommInfoPacket packet;
		memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_COMM_INFO);
		HANDLE_MSG_COMM_INFO(&packet);
		break;
    }
#endif
#ifdef HANDLE_MSG_DEVICE_INFO
    case ESCBUS_MSG_ID_DEVICE_INFO:
    {
    	char ESC_HardwareVersion[ESC_HARDWARE_VERSION_LENGTH];
    	char ESC_SoftwareVersion[ESC_SOFTWARE_VERSION_LENGTH];
    	char ESCBUS_Version[ESCBUS_VERSION_LENGTH];
    	EscbusDeviceInfoPacket packet = {msg->data[0], ESC_HardwareVersion, ESC_SoftwareVersion, ESCBUS_Version};

        memcpy (ESC_HardwareVersion, (_ESCBUS_DATA(msg) + 1), ESC_HARDWARE_VERSION_LENGTH);
        memcpy (ESC_SoftwareVersion, (_ESCBUS_DATA(msg) + 1 + ESC_HARDWARE_VERSION_LENGTH), ESC_SOFTWARE_VERSION_LENGTH);
        memcpy (ESCBUS_Version, (_ESCBUS_DATA(msg) + 1 + ESC_HARDWARE_VERSION_LENGTH + ESC_SOFTWARE_VERSION_LENGTH), ESCBUS_VERSION_LENGTH);
		HANDLE_MSG_DEVICE_INFO(&packet);
		break;
    }
#endif
#ifdef HANDLE_MSG_ASSIGNED_ID
    case ESCBUS_MSG_ID_ASSIGNED_ID:
    {
    	EscbusAssignedIDPacket packet;
		memcpy((char* )&packet, _ESCBUS_DATA(msg), ESCBUS_MESSAGE_LENGTH_ASSIGNED_ID);
		HANDLE_MSG_ASSIGNED_ID(&packet);
		break;
    }
#endif
    default:
    	break;

    }// end switch
}

/*************************************************************************************
 * Reset ESCBus status to idle
 */
void escbus_reset_status(void)
{
	memset(&rxmsg, 0, sizeof(EscbusMessageType));
	memset(&rxstatus, 0, sizeof(EscbusStatusType));
}

/*************************************************************************************
 * Send ESCBus message
 */
void escbus_message_send(uint8_t msgid, const char *packet, uint8_t length)
{
	uint8_t msgLength = length + ESCBUS_NUM_NON_DATA_FIELD_BYTES;
	EscbusMessageType msgbuf;
	// ESCBus header
	msgbuf.start = ESCBUS_START;
	msgbuf.length = length;
	msgbuf.msgid = msgid;

	// return if we can't fill in the message buffer
	if (length > ESCBUS_MAX_DATA_LEN)
		return;

	// ESCBus fill in data field
	if (packet != NULL)
		memcpy(_ESCBUS_DATA_LOCATE(msgbuf, 0), packet, length);

	// Calculate CRC value for the whole message except start sign
#if	defined(ESCBUS_CRC_HARDWARE)
    ESCBUS_CRC_BYTE(msgbuf) = ESCBUS_CRC_HARDWARE(&msgbuf, length);
#elif defined(ESCBUS_CRC_SOFTWARE)
    ESCBUS_CRC_BYTE(msgbuf) = ESCBUS_CRC_SOFTWARE(&msgbuf, length);
#else
#error "ESCBUS: Miss hardware or software implement of CRC8 calculation!"
#endif

    // Transmit message package
#if defined(ESCBUS_SEND_UART)
    ESCBUS_SEND_UART(&msgbuf, msgLength);
#elif defined(ESCBUS_SEND_CAN)
    ESCBUS_SEND_CAN(&msgbuf, msgLength);
#else
#error "ESCBUS: Miss instance for sending message!"
#endif
}

/*************************************************************************************
 * This is a convenience function which handles the complete ESCBus parsing.
 * the function will parse one byte at a time and return the complete packet once
 * it could be successfully decoded. CRC and other failures will be silently
 * ignored.
 *
 * Messages are parsed into an internal buffer. When a complete message is received
 * it is copies into *returnMsg and the channel's status is copied into *returnStats.
 *
 * @param c: The char to parse
 *
 * @param returnMsg: NULL if no message could be decoded, the message data else
 * @param returnStatus: if a message was decoded, this is filled with the bus's status
 * @return 0 if no message could be decoded, 1 else
 *
 * A typical use scenario of this function call is:
 *
 * @code
 * #include <inttypes.h> // For fixed-width uint8_t type
 *
 * EscbusMessageType msg;
 * EscbusStatusType status;
 *
 * while(serial.bytesAvailable > 0)
 * {
 *   	uint8_t byte = serial.getNextByte();
 *   	if (escbus_parse_char(byte, &msg, &status))
 *     	{
 *     		printf("Received message with ID %d, Data length: %d, Error status: %d, Buffer overrun: %d", msg.msgid, msg.length, status.parseError, status.bufferOverrun);
 *     	}
 * }
 *
 *
 * @endcode
 */
uint8_t escbus_parse_char(uint8_t c, EscbusMessageType* returnMsg, EscbusStatusType* returnStatus)
{
	rxstatus.msgReceived = 0;

	switch (rxstatus.parseState)
	{
	case ESCBUS_PARSE_STATE_UNINIT:
	case ESCBUS_PARSE_STATE_IDLE:
		if (c == ESCBUS_START)
		{
			rxstatus.parseState = ESCBUS_PARSE_STATE_GOT_START;
			rxmsg.length = 0;
		}
		break;

	case ESCBUS_PARSE_STATE_GOT_START:
			if (rxstatus.msgReceived
/* Support shorter buffers than the maximum packet size */
#if (ESCBUS_MAX_DATA_LEN < 255)
				|| c > ESCBUS_MAX_DATA_LEN
#endif
				)
		{
			rxstatus.bufferOverrun++;
			rxstatus.parseError++;
			rxstatus.msgReceived = 0;
			rxstatus.parseState = ESCBUS_PARSE_STATE_IDLE;
		}
		else
		{
			rxmsg.length = c;
			rxstatus.dataIndex = 0;
			rxstatus.parseState = ESCBUS_PARSE_STATE_GOT_LENGTH;
		}
		break;

	case ESCBUS_PARSE_STATE_GOT_LENGTH:
#ifdef ESCBUS_CHECK_MESSAGE_LENGTH
		if (rxmsg.length != ESCBUS_MESSAGE_LENGTH(c))
		{
			rxstatus.parseError++;
			rxstatus.parseState = ESCBUS_PARSE_STATE_IDLE;
			if (c == ESCBUS_START)
			{
				rxstatus.parseState = ESCBUS_PARSE_STATE_GOT_START;
			}
		}
		else
		{
#endif
			rxmsg.msgid = c;
			if (rxmsg.length == 0)
			{
				rxstatus.parseState = ESCBUS_PARSE_STATE_GOT_DATA;
			}
			else
			{
				rxstatus.parseState = ESCBUS_PARSE_STATE_GOT_MSGID;
			}
#ifdef ESCBUS_CHECK_MESSAGE_LENGTH
		}
#endif
		break;

	case ESCBUS_PARSE_STATE_GOT_MSGID:
		_ESCBUS_DATA_NON_CONST(rxmsg)[rxstatus.dataIndex++] = (char)c;
		if (rxstatus.dataIndex == rxmsg.length)
		{
			rxstatus.parseState = ESCBUS_PARSE_STATE_GOT_DATA;
		}
		break;

	case ESCBUS_PARSE_STATE_GOT_DATA:
		// Calculate CRC value for the whole message except start sign
#if	defined(ESCBUS_CRC_HARDWARE)
		if (c != ESCBUS_CRC_HARDWARE(&rxmsg, rxmsg.length))
		{
#elif defined(ESCBUS_CRC_SOFTWARE)
		if (c != ESCBUS_CRC_SOFTWARE(&rxmsg, rxmsg.length))
		{
#else
#error "ESCBUS: Miss hardware or software implement of CRC calculation!"
#endif
			// Check CRC byte
			rxstatus.parseError++;
			rxstatus.msgReceived = 0;
			rxstatus.parseState = ESCBUS_PARSE_STATE_IDLE;
			if (c == ESCBUS_START)
			{
				rxstatus.parseState = ESCBUS_PARSE_STATE_GOT_START;
				rxmsg.length = 0;
			}
		}
		else
		{
			// Successfully got message
			rxstatus.msgReceived = 1;
			rxstatus.parseState = ESCBUS_PARSE_STATE_IDLE;
			_ESCBUS_DATA_NON_CONST(rxmsg)[rxstatus.dataIndex] = (char)c;
			memcpy(returnMsg, &rxmsg, sizeof(EscbusMessageType));
		}
		break;
	}

	// copy rxstatus into returnStatus
	returnStatus->bufferOverrun = rxstatus.bufferOverrun;
	returnStatus->parseError = rxstatus.parseError;

	// clear error indicator
	rxstatus.parseError = 0;
	return rxstatus.msgReceived;
}



