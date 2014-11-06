// -*- Mode: C++; c-basic-offset: 8; indent-tabs-mode: nil -*-

//
// Example code for the AP_HAL AVRUARTDriver, based on FastSerial
//
// This code is placed into the public domain.
//

#include <AP_Common.h>
#include <AP_Math.h>
#include <StorageManager.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <utility/pinmap_typedef.h>
#include <utility/ESCBus_Types.h>
#include <utility/ESCBus.h>
#include <utility/CRC8.h>
#include <ctype.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_HAL::DigitalSource *green_led;
AP_HAL::DigitalSource *red_led;
AP_HAL::DigitalSource *blue_led;
AP_HAL::DigitalSource *s0;
AP_HAL::DigitalSource *s1;
AP_HAL::DigitalSource *s2;
int16_t user_input;
char buf[64];
uint8_t i = 0;

EscbusMessageType msg;
EscbusStatusType status;
uint8_t maxChNum = 0;

void _escbus_send_uart(const uint8_t *buffer, uint8_t length) {
	hal.uartC->write(buffer, length);
}

void handle_msg_config_info_basic(EscbusConfigInfoBasicPacket *packet) {
	uint8_t ii = 0;
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_CONFIG_INFO_BASIC\n");
	hal.console->printf("channelID = %d\n", packet->channelID);
	hal.console->printf("maxChannelInUse = %d\n", packet->maxChannelInUse);
	hal.console->printf("channelMapTable = {");
	for (ii = 0; ii < MAX_CHANNEL_AVAILABLE; ii++) {
		hal.console->printf("%d ", packet->channelMapTable[ii]);
	}
	hal.console->printf("}\n");
	hal.console->printf("monitorMsgType = %02x\n", packet->monitorMsgType);
	hal.console->printf("contorlMode = %d\n\n", packet->controlMode);
}

void handle_msg_config_info_full(EscbusConfigInfoFullPacket *packet) {
	uint8_t ii = 0;
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_CONFIG_INFO_FULL\n");
	hal.console->printf("channelID = %d\n", packet->configInfoBasic.channelID);
	hal.console->printf("maxChannelInUse = %d\n", packet->configInfoBasic.maxChannelInUse);
	hal.console->printf("channelMapTable = {");
	for (ii = 0; ii < MAX_CHANNEL_AVAILABLE; ii++) {
		hal.console->printf("%d ", packet->configInfoBasic.channelMapTable[ii]);
	}
	hal.console->printf("}\n");
	hal.console->printf("monitorMsgType = %02x\n"
						"contorlMode = %d\n"
						"minChannelValue = %d\n"
						"maxChannelValue = %d\n"
						"minSpeedSet = %d\n"
						"maxSpeedSet = %d\n"
						"breakLevel = %d\n"
						"cutVoltage = %d\n"
						"limitCurrent = %d\n"
						"advanceAngle = %d\n"
						"freqOfPWM = %d\n"
						"polesNum = %d\n"
						"PID_P = %f\n"
						"PID_I = %f\n\n", packet->configInfoBasic.monitorMsgType, packet->configInfoBasic.controlMode, packet->minChannelValue, packet->maxChannelValue, packet->minSpeedSet, packet->maxSpeedSet, packet->breakLevel, packet->cutVoltage, packet->limitCurrent, packet->advanceAngle, packet->freqOfPWM, packet->polesNum, packet->PID_P, packet->PID_I);

}

void handle_msg_run_info(EscbusRunInfoPacket *packet) {
	uint8_t channel = ESCBUS_GET_CHANNEL_ID(packet);
	uint8_t motorStatus = ESCBUS_GET_MOTOR_STATUS(packet);
	static const char *str1 = "stop";
	static const char *str2 = "running";
	char str[10] ;

	if (motorStatus == MOTOR_STOP)
		strcpy(str, str1);
	else
		strcpy(str, str2);

	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_RUN_INFO\n");
	hal.console->printf_P(PSTR("Channel: %d  Motor Status: %s ESC Status: %d Speed: %d \n\n"), channel, str, packet->ESCStatus, packet->speed);
}

void handle_msg_study_info(EscbusStudyInfoPacket *packet) {
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_STUDY_INFO\n");
	hal.console->printf("channelID = %d\n"
						"motorMinSpeed = %d\n"
						"motorMaxSpeed = %d\n\n", packet->channelID, packet->motorMinSpeed, packet->motorMaxSpeed);
}

void handle_msg_comm_info(EscbusCommInfoPacket *packet) {
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_COMM_INFO\n");
	hal.console->printf("channelID = %d\n"
						"inputMode = %d\n"
						"bardRate = %d\n\n", packet->channelID, packet->inputMode, packet->baudRate);
}

void handle_msg_device_info(EscbusDeviceInfoPacket *packet) {
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_DEVICE_INFO\n");
	hal.console->printf("channelID = %d\n"
						"ESC_HardwareVersion = %s\n"
						"ESC_SoftwareVersion = %s\n"
						"ESCBUS_Version = %s\n\n", packet->channelID, packet->ESC_HardwareVersion, packet->ESC_SoftwareVersion, packet->ESCBUS_Version);
}

void handle_msg_assigned_info(EscbusAssignedIDPacket *packet) {
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_ASSIGNED_ID\n");
    hal.console->printf_P(PSTR("channelID = %d\nescID = %d\n\n"), packet->channelID, packet->escID);
}

void handle_msg_config_basic(EscbusConfigBasicPacket *packet) {
	uint8_t ii = 0;
	maxChNum = packet->maxChannelInUse;
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_CONFIG_BASIC\n");
	hal.console->printf("maxChannelInUse = %d\n", packet->maxChannelInUse);
	hal.console->printf("channelMapTable = { ");
	for (ii = 0; ii < MAX_CHANNEL_AVAILABLE; ii++) {
		hal.console->printf("%d ", packet->channelMapTable[ii]);
	}
	hal.console->printf("}\n");
	hal.console->printf("monitorMsgType = %02x\n", packet->monitorMsgType);
	hal.console->printf("contorlMode = %d\n\n", packet->controlMode);
}

void handle_msg_config_full(EscbusConfigFullPacket *packet) {
	uint8_t ii = 0;
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_CONFIG_FULL\n");
	hal.console->printf("maxChannelInUse = %d\n", packet->basicConfig.maxChannelInUse);
	hal.console->printf("channelMapTable = {");
	for (ii = 0; ii < MAX_CHANNEL_AVAILABLE; ii++) {
		hal.console->printf("%d ", packet->basicConfig.channelMapTable[ii]);
	}
	hal.console->printf("}\n");
	hal.console->printf("monitorMsgType = %02x\n"
						"contorlMode = %d\n"
						"minChannelValue = %d\n"
						"maxChannelValue = %d\n"
						"minSpeedSet = %d\n"
						"maxSpeedSet = %d\n"
						"breakLevel = %d\n"
						"cutVoltage = %d\n"
						"limitCurrent = %d\n"
						"advanceAngle = %d\n"
						"freqOfPWM = %d\n"
						"polesNum = %d\n"
						"PID_P = %f\n"
						"PID_I = %f\n\n", packet->basicConfig.monitorMsgType, packet->basicConfig.controlMode, packet->minChannelValue, packet->maxChannelValue, packet->minSpeedSet, packet->maxSpeedSet, packet->breakLevel, packet->cutVoltage, packet->limitCurrent, packet->advanceAngle, packet->freqOfPWM, packet->polesNum, packet->PID_P, packet->PID_I);
}

void handle_msg_run(EscbusRunPacket *packet) {
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_RUN\n");
	hal.console->printf("value = {");
	for (int ii = 0; ii < maxChNum; ii++) {
		hal.console->printf("%d ", packet->value[ii]);
	}
	hal.console->printf("}\n\n");
}

void handle_msg_tune(EscbusTunePacket *packet) {
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_TUNE\n");
	hal.console->printf("frequency = %d\n", packet->frequency);
	hal.console->printf("counter = %d\n", packet->counter);
	hal.console->printf("strength = %d\n\n", packet->strength);
}

void handle_msg_do_cmd(EscbusDoCmdPacket *packet) {
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_DO_CMD\n");
	hal.console->printf("channelIDMask = %02x\n", packet->channelIDMask);
	hal.console->printf("command = %d\n\n", packet->command);
}

void handle_msg_request_info(EscbusRequestInfoPacket *packet) {
	hal.console->printf("\nReceive MSG ID: ESCBUS_MSG_ID_REQUEST_INFO\n");
	hal.console->printf("channelID = %d\n", packet->channelID);
	hal.console->printf("requestBit = %02x\n\n", packet->requestBit);
}

uint8_t parse_command(char *cmd)
{
	uint8_t cmd_num = 0;
	if (strcmp(cmd, "run") == 0)
		cmd_num = 1;
	else if (strcmp(cmd, "tune") == 0)
		cmd_num = 2;
	else if (strcmp(cmd, "assign") == 0)
		cmd_num = 3;
	else if (strcmp(cmd, "request") == 0)
		cmd_num = 4;
	else if (strcmp(cmd, "reboot") == 0)
		cmd_num = 5;
	else if (strcmp(cmd, "pc") == 0)
		cmd_num = 6;
	else if (strcmp(cmd, "pr") == 0)
		cmd_num = 7;
	else if (strcmp(cmd, "testuart") == 0)
		cmd_num = 8;
	else if (strcmp(cmd, "testescbus") == 0)
		cmd_num = 9;
	else if (strcmp(cmd, "testrc") == 0)
		cmd_num = 10;
	return cmd_num;
}

int get_int_from_buf(void) {
	int value = 0;

    while( hal.console->available() ) {
        user_input = hal.console->read();

		if (!isspace(user_input)) {
			buf[i++] = user_input;
		}
		else {
			buf[i] = '\0';
			i = 0;
			value = atoi(buf);
			break;
        }
    }

    return value;
}

void handle_msg(void) {
	hal.console->printf_P(PSTR("Receive data: "));
    // stop until user input something
    while( !hal.console->available() ) {
        if (hal.uartC->available()) {
        	uint8_t c = hal.uartC->read();
			hal.console->printf("%02x ", c);
        	if ( escbus_parse_char(c, &msg, &status) ) {
        		hal.console->printf_P(PSTR("Receive MSG ID: %d\n"), msg.msgid);
        	    break;
        	}
        }
    }

    while( !hal.console->available() ) {
        if (hal.uartC->available()) {
        	uint8_t c = hal.uartC->read();
			hal.console->printf("%c", c);
        	if ( escbus_parse_char(c, &msg, &status) ) {
        		hal.console->printf_P(PSTR("\nReceive MSG ID: %d\n"), msg.msgid);
        	    escbus_handle_message(&msg);
        	    break;
        	}
        }
    }

	hal.console->printf("\n");
    // clear buffer
	while ( hal.console->available() ) {
        hal.console->read();
	}
}

#define CHANNEL_NUM	6
void send_run(void) {
	uint8_t channel = get_int_from_buf();
	int16_t pwm = get_int_from_buf();
	int16_t value[CHANNEL_NUM] = {0};
	uint32_t last_time = 0;

	value[channel] = (pwm & RUN_CHANNEL_VALUE_MASK) | RUN_FEEDBACK_ENABLE_MASK;

	hal.console->printf_P(PSTR("Set channel: %d pwm: %d value: 0x%04x\r\n"), channel, pwm, value[channel]);

    green_led->write(1);
    red_led->write(0);
    blue_led->write(1);

    last_time = hal.scheduler->millis();
    while (!hal.console->available()) {
    	if (hal.scheduler->millis() - last_time > 2) {
        	escbus_send_msg_run(value, CHANNEL_NUM);
        	last_time = hal.scheduler->millis();
    	}

    	if (hal.uartC->available()) {
    		uint8_t c = hal.uartC->read();
    		if (escbus_parse_char(c, &msg, &status))
    			escbus_handle_message(&msg);
    	}
    }

	value[channel] = 0;
    last_time = hal.scheduler->millis();
	uint32_t time = last_time;
    while (hal.scheduler->millis() - last_time < 300) {
    	if (hal.scheduler->millis() - time > 2) {
        	escbus_send_msg_run(value, CHANNEL_NUM);
        	time += 2;
    	}
    	if (hal.uartC->available()) {
    		uint8_t c = hal.uartC->read();
    		if (escbus_parse_char(c, &msg, &status))
    			escbus_handle_message(&msg);
    	}
    }

    // clear buffer
	while ( hal.console->available() ) {
        hal.console->read();
	}
}

void send_tune() {
	uint16_t freq = get_int_from_buf();
	uint16_t conter = get_int_from_buf();
	uint8_t strenth = get_int_from_buf();

	escbus_send_msg_tune(freq, conter, strenth);
}

void send_assign() {
	uint8_t channel = get_int_from_buf() - 1;

	channel = (uint8_t)1 << channel;
	escbus_send_msg_do_cmd(channel, DO_ID_ASSIGNMENT);
	handle_msg();
}

void send_request() {
	uint8_t channel = get_int_from_buf() - 1;
	int16_t info = get_int_from_buf();

	channel = (uint8_t)1 << channel;
	info = (uint8_t)1 << info;

	escbus_send_msg_request_info(channel, info);
	handle_msg();
}

void send_reboot() {
	uint8_t channel = get_int_from_buf() - 1;
	uint8_t led_on = 0;

	channel = (uint8_t)1 << channel;
	escbus_send_msg_do_cmd(channel, DO_RESET);

    green_led->write(1);
    red_led->write(1);

    for (i = 0; i < 4; i++) {
    	led_on = led_on ^ 0x01;
        blue_led->write(led_on);
        hal.scheduler->delay(250);
    }
}

void print_crcTable(void) {
	uint8_t polynomial = get_int_from_buf();

	hal.console->printf_P("const uint8_t crc8_table[256] = {\n");

	for (int ii = 0; ii < 256; ii++) {
		uint8_t entry = ii;

		for (int jj = 0; jj < 8; jj++) {
			if (entry & 0x80)
				entry = (entry << 1) ^ polynomial;
			else
				entry <<= 1;
		}
		hal.console->printf_P("0x%02x", entry);
		if (ii < 255)
			hal.console->printf_P(", ");
		if ( (ii + 1) % 12 == 0)
			hal.console->printf_P("\n");
	}
	hal.console->printf_P("};\n");
}


#define RPM_CONV_CONST      833333.3333
#define RPM_SCALE       	4
#define RPM_TABLE_STEP 		8
#define RPM_TABLE_SIZE 		189
#define RPM_TABLE_OFFSET 	96

void print_rpmTable() {
	int8_t  bb[RPM_TABLE_SIZE];
	int16_t aa[RPM_TABLE_SIZE];

	int ii;
	float val = RPM_TABLE_OFFSET;

	for (ii=0; ii < RPM_TABLE_SIZE; ii++)
	{
		aa[ii] = (RPM_CONV_CONST/RPM_SCALE)  / val + 0.5;
		bb[ii] = -(RPM_CONV_CONST/RPM_SCALE) / (val*val) - 0.5;
		val   += RPM_TABLE_STEP;
	}

	hal.console->printf_P("static const int8_t aa[RPM_TABLE_SIZE] = {\n");
	for (ii = 0; ii < RPM_TABLE_SIZE; ii++) {
		hal.console->printf_P("%d", aa[ii]);
		if (ii < 255)
			hal.console->printf_P(", ");
		if ( (ii + 1) % 12 == 0)
			hal.console->printf_P("\n");
	}
	hal.console->printf_P("};\n");

	hal.scheduler->delay(100);

	hal.console->printf_P("static const int8_t bb[RPM_TABLE_SIZE] = {\n");
	for (ii = 0; ii < RPM_TABLE_SIZE; ii++) {
		hal.console->printf_P("%d", bb[ii]);
		if (ii < 255)
			hal.console->printf_P(", ");
		if ( (ii + 1) % 12 == 0)
			hal.console->printf_P("\n");
	}
	hal.console->printf_P("};\n");
}

void esc_uart_test() {
	uint8_t ii;

    green_led->write(1);
    red_led->write(1);
    blue_led->write(0);

	hal.console->printf_P("ESC UART FUNCTION TEST START\r\n");

	hal.console->printf_P("Send: \r\n");
	for (ii = 0; ii < 100; ii++) {
		hal.uartC->write(ii);
		hal.console->printf("%d ", ii);
	}
	hal.console->printf_P("\nReceive: \r\n");
	hal.scheduler->delay(10);

	ii = 0;
	while ( !hal.console->available() ) {
		if (hal.uartC->available()) {
			ii++;
			uint8_t c = hal.uartC->read();
			hal.console->printf("%d ", c);
		}

		if (ii == 100)
			break;
	}
	hal.console->printf_P("\nESC UART FUNCTION TEST END\r\n");

    // clear buffer
	while ( hal.console->available() ) {
        hal.console->read();
	}
}

void escbus_test() {
	uint8_t maxChannelInUse = 4;
	int8_t channelMapTable[MAX_CHANNEL_AVAILABLE] = {0, 1, 2, 3, 4, 5};
	uint8_t monitorMsgType = MONITOR_MSG_TYPE_SPEED_MASK;
	uint8_t controlMode = CONTROL_MODE_OPEN_LOOP;
	int16_t value[MAX_CHANNEL_AVAILABLE] = {0};
	uint8_t channelIDMask = DO_CMD_CHANNEL1;
	uint8_t channelId = 0;
	uint8_t requestBit = REQUEST_INFO_CONFIG_BASIC_MASK;
	uint8_t test_status = 0;
	uint8_t dataIdx = 0;
	hal.console->printf("\nESCBUS Test Start\n");

	escbus_init(maxChannelInUse);

	for (int seq = 0; seq < ESCBUS_MSG_ID_MAX_NUM; seq++) {
		switch (seq) {
		case 0:
			hal.console->printf("\nESCBUS: Send basic config message\n");
			escbus_send_msg_config_basic(maxChannelInUse, channelMapTable, monitorMsgType, controlMode);
			break;
		case 1:
			hal.console->printf("\nESCBUS: Send full config message\n");
			escbus_send_msg_config_full( maxChannelInUse, channelMapTable, monitorMsgType, controlMode,
					  	  	  	  	  	  0,100,10,90,0,20,40,30,20000,12,0.01,0.001);
			break;
		case 2:
			hal.console->printf("\nESCBUS: Send run message\n");
			escbus_send_msg_run(value, maxChannelInUse);
			break;
		case 3:
			hal.console->printf("\nESCBUS: Send tune message\n");
			escbus_send_msg_tune(30, 30, 30);
			break;
		case 4:
			hal.console->printf("\nESCBUS: Send do command - DO_RESET message\n");
			escbus_send_msg_do_cmd(channelIDMask, DO_RESET);
			break;
		case 5:
			hal.console->printf("\nESCBUS: Send request info message\n");
			escbus_send_msg_request_info(channelId, requestBit);
			break;
		case 6:
			hal.console->printf("\nESCBUS: Send basic config info message\n");
			escbus_send_msg_config_info_basic(0, maxChannelInUse, channelMapTable, monitorMsgType, controlMode);
			break;
		case 7:
			hal.console->printf("\nESCBUS: Send full config info message\n");
			escbus_send_msg_config_info_full( 0, maxChannelInUse, channelMapTable, monitorMsgType, controlMode,
					  	  	  	  	  	  0,100,10,90,0,20,40,30,20000,12,0.01,0.001);
			break;
		case 8:
			hal.console->printf("\nESCBUS: Send run info message\n");
			escbus_send_msg_run_info( 0, 0, 1000, 0, 0, 0 );
			break;
		case 9:
			hal.console->printf("\nESCBUS: Send study info message\n");
			escbus_send_msg_study_info( 0, 10, 10000 );
			break;
		case 10:
			hal.console->printf("\nESCBUS: Send comm info message\n");
			escbus_send_msg_comm_info( 0, INPUT_MODE_UART, BAUDRATE_DEFAULT );
			break;
		case 11:
			hal.console->printf("\nESCBUS: Send device info message\n");
			escbus_send_msg_device_info(0, ESC_HARDWARE_VERSION, ESC_SOFTWARE_VERSION, ESCBUS_VERSION);
			break;
		case 12:
			hal.console->printf("\nESCBUS: Send assigned info message\n");
			escbus_send_msg_assigned_id( 0, 1 );
			break;
		}
		hal.scheduler->delay(10);
		hal.console->printf("Received Data: ");
		while (hal.uartC->available()) {
			uint8_t c = hal.uartC->read();
			hal.console->printf("%02x ", c);
			dataIdx++;
			if (escbus_parse_char(c, &msg, &status)) {
				hal.console->printf("\nData length: %d\n", dataIdx);
				escbus_handle_message(&msg);
				test_status = 1;
				dataIdx = 0;
			}
		}

		if (test_status == 0) {
			hal.console->printf("\nSeq %d failed\n", seq);
			test_status = 0;
		}

		hal.scheduler->delay(200);
	}
	hal.console->printf("\nESCBUS Test Stop\n");
}

void esc_rc_test() {
    uint16_t channel = get_int_from_buf();
    int16_t value[CHANNEL_NUM] = {0};
    uint16_t channel_value;
    uint32_t last_time;
    uint32_t last_time_show;

    green_led->write(1);
    red_led->write(0);
    blue_led->write(1);

    last_time = hal.scheduler->millis();
    last_time_show = last_time;
    while ( !hal.console->available()) {
    	if (hal.rcin->new_input()) {
    		channel_value = hal.rcin->read(2);
    		value[channel] = (channel_value & RUN_CHANNEL_VALUE_MASK) | RUN_FEEDBACK_ENABLE_MASK;
    	}
    	if (hal.scheduler->millis() - last_time_show > 50) {
    		hal.console->printf_P(PSTR("Set channel: %d pwm: %d\n"), channel, channel_value);
    		last_time_show = hal.scheduler->millis();
    	}
     	if (hal.scheduler->millis() - last_time > 2) {
         	escbus_send_msg_run(value, CHANNEL_NUM);
         	last_time = hal.scheduler->millis();
     	}
     	if (hal.uartC->available()) {
     		uint8_t c = hal.uartC->read();
     		if (escbus_parse_char(c, &msg, &status))
     			escbus_handle_message(&msg);
     	}
    }

 	value[channel] = 0;
     last_time = hal.scheduler->millis();
 	uint32_t time = last_time;
     while (hal.scheduler->millis() - last_time < 300) {
     	if (hal.scheduler->millis() - time > 2) {
         	escbus_send_msg_run(value, CHANNEL_NUM);
         	time += 2;
     	}
     	if (hal.uartC->available()) {
     		uint8_t c = hal.uartC->read();
     		if (escbus_parse_char(c, &msg, &status))
     			escbus_handle_message(&msg);
     	}
     }

     // clear buffer
 	while ( hal.console->available() ) {
         hal.console->read();
 	}
}

void setup(void)
{
    green_led = hal.gpio->channel(PE9);
    red_led = hal.gpio->channel(PE8);
    blue_led = hal.gpio->channel(PB0);

    green_led->mode(HAL_GPIO_OUTPUT);
    red_led->mode(HAL_GPIO_OUTPUT);
    blue_led->mode(HAL_GPIO_OUTPUT);

    green_led->write(0);
    red_led->write(0);
    blue_led->write(0);

    s0 = hal.gpio->channel(PC15);
    s1 = hal.gpio->channel(PD8);
    s2 = hal.gpio->channel(PB2);

    s0->mode(HAL_GPIO_OUTPUT);
    s1->mode(HAL_GPIO_OUTPUT);
    s2->mode(HAL_GPIO_OUTPUT);

    s0->write(0);
    s1->write(0);
    s2->write(0);

    hal.scheduler->delay(10);

    for (i = 0; i < 4; i++) {
        green_led->toggle();
        red_led->toggle();
        blue_led->toggle();
        hal.scheduler->delay(250);
    }

//    hal.uartA->begin(115200);
    hal.console->begin(115200);
    hal.uartC->begin(250000);

    // Test start
    hal.console->print_P(PSTR("ESCBus Test Start\r\n"));
    escbus_init(4);
    hal.scheduler->delay(500);

}


void loop(void)
{
    uint8_t cmd_num = 0;
    uint32_t last_time = 0;
    static uint8_t led_status = 0;

    last_time = hal.scheduler->millis();

    hal.console->println();
    hal.console->println_P(PSTR(
    "Menu:\r\n"
    "    1) run channel pwm\r\n"
    "    2) tune frequency counter strength\r\n"
    "    3) assign channel\r\n"
    "    4) request channel info(0: config basic; 1: config full 2: run 3: study 4: comm 5: device)\r\n"
    "    5) reboot channel\r\n"
	"    6) pc polynomial (print CRC8 table)\r\n"
    "    7) pr (print rpm table)\r\n"
	"    8) testuart (test uart on ESC)\r\n"
    "    9) testescbus (test ESCBUS)\r\n"
    "   10) testrc channel(test ESC with rc)\r\n"));

    // wait for user input
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
        if (hal.scheduler->millis() - last_time > 1000) {
            led_status = led_status ^ 0x01;
            green_led->write(led_status);
            red_led->write(led_status);
            blue_led->write(led_status);
            last_time = hal.scheduler->millis();
        }
        if ( hal.uartC->available() ) {
        	uint8_t c = hal.uartC->read();
        	hal.console->printf("%c", c);
        }
    }

    // read in user input
    while( hal.console->available() ) {
        user_input = hal.console->read();
        green_led->write(0);
        red_led->write(1);
        blue_led->write(1);
        if (!isspace(user_input))
        {
        	buf[i++] = user_input;
        }
        else
        {
        	buf[i] = '\0';
        	i = 0;
        	break;
        }
    }

	cmd_num = parse_command(buf);

	switch (cmd_num)
	{
	case 1:
		send_run();
		break;
	case 2:
		send_tune();
		break;
	case 3:
		send_assign();
		break;
	case 4:
		send_request();
		break;
	case 5:
		send_reboot();
		break;
	case 6:
		print_crcTable();
		break;
	case 7:
		print_rpmTable();
		break;
	case 8:
		esc_uart_test();
		break;
	case 9:
		escbus_test();
		break;
	case 10:
		esc_rc_test();
		break;
	default:
		break;
	}
}

AP_HAL_MAIN();
