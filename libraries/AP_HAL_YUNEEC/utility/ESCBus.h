#ifndef ESCBUS_H
#define ESCBUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ESCBus_Types.h"

/*************************************************************************************
 * Implement your method to send ESCBus message here
 */

extern void escbus_send_uart(const uint8_t*, uint8_t);
#define ESCBUS_SEND_UART(msgbuf, msgLength)		escbus_send_uart((const uint8_t *)msgbuf, msgLength)//Replace_With_Your_Func_Name
//#define ESCBUS_SEND_CAN(msgbuf, msgLength)	Replace_With_Your_Func_Name

/*************************************************************************************
 * Implement your method to calculate CRC8 value here
 *
 * By default we use a software one, however it's highly recommended to use
 * hardware CRC unit to save both space and time if your MCU has CRC unit.
 */
//#define ESCBUS_CRC_HARDWARE(msgbuf, length)	_escbus_crc_hardware(msgbuf, length)
#define ESCBUS_CRC_SOFTWARE(msgbuf, length)		_escbus_crc_software(msgbuf, length)

#ifdef ESCBUS_CRC_HARDWARE
uint8_t ESCBUS_CRC_HARDWARE(EscbusMessageType *msgbuf, uint8_t length);
#endif
#ifdef ESCBUS_CRC_SOFTWARE
uint8_t ESCBUS_CRC_SOFTWARE(EscbusMessageType *msgbuf, uint8_t length);
#endif

/*************************************************************************************
 * Implement your functions to handle message to ESC
 *
 * your functions should follow this definition:
 *
 * void handle_msg_XXX (EscbusXXXPacket *packet)
 */

//#define HANDLE_MSG_CONFIG_BASIC		handle_msg_config_basic
//#define HANDLE_MSG_CONFIG_FULL		handle_msg_config_full
//#define HANDLE_MSG_RUN				handle_msg_run
//#define HANDLE_MSG_TUNE				handle_msg_tune
//#define HANDLE_MSG_DO_CMD				handle_msg_do_cmd
//#define HANDLE_MSG_REQUEST_INFO		handle_msg_request_info

#ifdef HANDLE_MSG_CONFIG_BASIC
void HANDLE_MSG_CONFIG_BASIC(EscbusConfigBasicPacket *packet);
#endif
#ifdef HANDLE_MSG_CONFIG_FULL
void HANDLE_MSG_CONFIG_FULL(EscbusConfigFullPacket *packet);
#endif
#ifdef HANDLE_MSG_RUN
void HANDLE_MSG_RUN(EscbusRunPacket *packet);
#endif
#ifdef HANDLE_MSG_TUNE
void HANDLE_MSG_TUNE(EscbusTunePacket *packet);
#endif
#ifdef HANDLE_MSG_DO_CMD
void HANDLE_MSG_DO_CMD(EscbusDoCmdPacket *packet);
#endif
#ifdef HANDLE_MSG_REQUEST_INFO
void HANDLE_MSG_REQUEST_INFO(EscbusRequestInfoPacket *packet);
#endif


/*************************************************************************************
 * Implement your functions to handle message from ESC
 *
 * your functions should follow this definition:
 *
 * void handle_msg_XXX (EscbusXXXPacket *packet)
 */
#define HANDLE_MSG_CONFIG_INFO_BASIC	handle_msg_config_info_basic
//#define HANDLE_MSG_CONFIG_INFO_FULL	handle_msg_config_info_full
#define HANDLE_MSG_RUN_INFO				handle_msg_run_info
//#define HANDLE_MSG_STUDY_INFO			handle_msg_study_info
//#define HANDLE_MSG_COMM_INFO			handle_msg_comm_info
//#define HANDLE_MSG_DEVICE_INFO		handle_msg_device_info
#define HANDLE_MSG_ASSIGNED_ID			handle_msg_assigned_info

#ifdef HANDLE_MSG_CONFIG_INFO_BASIC
void HANDLE_MSG_CONFIG_INFO_BASIC(EscbusConfigInfoBasicPacket *packet);
#endif
#ifdef HANDLE_MSG_CONFIG_INFO_FULL
void HANDLE_MSG_CONFIG_INFO_FULL(EscbusConfigInfoFullPacket *packet);
#endif
#ifdef HANDLE_MSG_RUN_INFO
void HANDLE_MSG_RUN_INFO(EscbusRunInfoPacket *packet);
#endif
#ifdef HANDLE_MSG_STUDY_INFO
void HANDLE_MSG_STUDY_INFO(EscbusStudyInfoPacket *packet);
#endif
#ifdef HANDLE_MSG_COMM_INFO
void HANDLE_MSG_COMM_INFO(EscbusCommInfoPacket *packet);
#endif
#ifdef HANDLE_MSG_DEVICE_INFO
void HANDLE_MSG_DEVICE_INFO(EscbusDeviceInfoPacket *packet);
#endif
#ifdef HANDLE_MSG_ASSIGNED_ID
void HANDLE_MSG_ASSIGNED_ID(EscbusAssignedIDPacket *packet);
#endif

/*
 * Convenience macro
 */
#define _ESCBUS_DATA(msg) ( (const char *)( &((msg)->data[0]) ) )
#define _ESCBUS_DATA_NON_CONST(msg) ( (char *)( &((msg).data[0]) ) )
#define _ESCBUS_DATA_LOCATE(msg, index) ( (char *)( &(msg.data[0]) + index) )

/******************************************************************************************
 * Export Functions
 */
void escbus_init(uint8_t maxChannelInUse);
void escbus_send_msg_config_basic( uint8_t maxChannelInUse, const int8_t* channelMapTable, uint8_t monitorMsgType, uint8_t controlMode );
void escbus_send_msg_config_full( uint8_t maxChannelInUse, int8_t* channelMapTable, uint8_t monitorMsgType, uint8_t controlMode,
								  uint16_t minChannelValue, uint16_t maxChannelValue, int16_t minSpeedSet, int16_t maxSpeedSet, uint8_t breakLevel, uint16_t cutVoltage,
								  uint8_t limitCurrent, uint8_t advanceAngle, uint16_t freqOfPWM, uint8_t polesNum, float PID_P, float PID_I );
void escbus_send_msg_run( int16_t* value, uint8_t channelNum );
void escbus_send_msg_tune( uint16_t frequency, uint16_t counter, uint8_t strength );
void escbus_send_msg_do_cmd( uint8_t channelIDMask, uint8_t command );
void escbus_send_msg_request_info( uint8_t channelID, uint8_t requestBit );
void escbus_send_msg_config_info_basic( uint8_t channelID, uint8_t maxChannelInUse, int8_t* channelMapTable, uint8_t monitorMsgType, uint8_t controlMode);
void escbus_send_msg_config_info_full( uint8_t channelID, uint8_t maxChannelInUse, int8_t* channelMapTable, uint8_t monitorMsgType, uint8_t controlMode,
									   uint16_t minChannelValue, uint16_t maxChannelValue, int16_t minSpeedSet, int16_t maxSpeedSet, uint8_t breakLevel, uint16_t cutVoltage,
									   uint8_t limitCurrent, uint8_t advanceAngle, uint16_t freqOfPWM, uint8_t polesNum, float PID_P, float PID_I );
void escbus_send_msg_run_info( uint8_t ID_MotorStatus, uint8_t ESCStatus, int16_t speed, uint16_t voltage, uint16_t current, uint8_t temperature );
void escbus_send_msg_study_info( uint8_t channelID, int16_t motorMinSpeed, int16_t motorMaxSpeed );
void escbus_send_msg_comm_info( uint8_t channelID, uint8_t inputMode, uint8_t baudRate );
void escbus_send_msg_device_info( uint8_t channelID, char* ESC_HardwareVersion, char* ESC_SoftwareVersion, char* ESCBUS_Version );
void escbus_send_msg_assigned_id( uint8_t channelID, uint8_t escID );
void escbus_handle_message( EscbusMessageType* msg );
void escbus_reset_status( void );
void escbus_message_send( uint8_t msgid, const char *packet, uint8_t length );
uint8_t escbus_parse_char( uint8_t c, EscbusMessageType* returnMsg, EscbusStatusType* returnStatus );

#ifdef __cplusplus
} // extern "C"
#endif

#endif
