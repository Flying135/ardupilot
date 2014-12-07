#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCOutput.h"
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include <utility/ESCBus_Types.h>
#include <utility/ESCBus.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

#define MIN_ESCUS_UPDATE_PERIODS	2

#define ESCBUS_DETECT_DELAY			1000

#if ESC_CHANNEL_NUM	== 4
	#define ESC_CHANNEL_EXIST_MASK	0b00001111
#elif ESC_CHANNEL_NUM == 6
	#define ESC_CHANNEL_EXIST_MASK	0b00111111
#elif ESC_CHANNEL_NUM == 8
	#define ESC_CHANNEL_EXIST_MASK	0b11111111
#else
#error "Unsupport channel number"
#endif

AP_HAL::UARTDriver* YUNEECRCOutputESCBUS::_escbus_uart = NULL;
uint8_t YUNEECRCOutputESCBUS::_update_period = MIN_ESCUS_UPDATE_PERIODS;
uint8_t YUNEECRCOutputESCBUS::_channel_enable_mask = 0;
int16_t YUNEECRCOutputESCBUS::_channel_value[ESC_CHANNEL_NUM] = {0};
uint32_t YUNEECRCOutputESCBUS::_last_update_time = 0;

extern "C" {
	void escbus_send_uart(const uint8_t *msgbuf, uint8_t length)
	{
		hal.uartC->write(msgbuf, length);
	}
}

bool YUNEECRCOutputESCBUS::init(void) {
	// initiate uart
	_uart_init(hal.uartC);

    // initiate ESCBUS
    if (_escbus_init()) {
    	// return true if detected ESCBUS device
        hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&YUNEECRCOutputESCBUS::_escbus_update));
        return true;
    } else
    	return false;
}

void YUNEECRCOutputESCBUS::set_freq(uint32_t chmask, uint16_t freq_hz) {
	uint16_t period = 1000 / freq_hz;

	if (period < MIN_ESCUS_UPDATE_PERIODS)
		period = MIN_ESCUS_UPDATE_PERIODS;

	_update_period = period;
}

uint16_t YUNEECRCOutputESCBUS::get_freq(uint8_t ch) {
    return 1000 / _update_period ;
}

void YUNEECRCOutputESCBUS::enable_ch(uint8_t ch) {
    switch (ch) {
        case CH_1: _channel_enable_mask |= _BV(CH_1); break;
        case CH_2: _channel_enable_mask |= _BV(CH_2); break;
        case CH_3: _channel_enable_mask |= _BV(CH_3); break;
		case CH_4: _channel_enable_mask |= _BV(CH_4); break;
        case CH_5: _channel_enable_mask |= _BV(CH_5); break;
        case CH_6: _channel_enable_mask |= _BV(CH_6); break;
        case CH_7: _channel_enable_mask |= _BV(CH_7); break;
        case CH_8: _channel_enable_mask |= _BV(CH_8); break;
        default:
            break;
    }
}

void YUNEECRCOutputESCBUS::disable_ch(uint8_t ch) {
    switch (ch) {
        case CH_1: _channel_enable_mask &=~ _BV(CH_1); break;
        case CH_2: _channel_enable_mask &=~ _BV(CH_2); break;
        case CH_3: _channel_enable_mask &=~ _BV(CH_3); break;
		case CH_4: _channel_enable_mask &=~ _BV(CH_4); break;
        case CH_5: _channel_enable_mask &=~ _BV(CH_5); break;
        case CH_6: _channel_enable_mask &=~ _BV(CH_6); break;
        case CH_7: _channel_enable_mask &=~ _BV(CH_7); break;
        case CH_8: _channel_enable_mask &=~ _BV(CH_8); break;
        default:
            break;
    }

    __disable_irq();
    _channel_value[ch] = 0;
    __enable_irq();
}

/* constrain pwm to be between min and max pulsewidth. */
uint16_t YUNEECRCOutputESCBUS::_constrain_period(uint8_t ch, uint16_t p) {
	if (!(_channel_enable_mask & _BV(ch))) return 0;
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

void YUNEECRCOutputESCBUS::write(uint8_t ch, uint16_t period_us) {
    uint16_t pwm = _constrain_period(ch, period_us);

    if (ch < ESC_CHANNEL_NUM) {
    	__disable_irq();
    	_channel_value[ch] = pwm & RUN_CHANNEL_VALUE_MASK;
    	__enable_irq();
    }
}

void YUNEECRCOutputESCBUS::write(uint8_t ch, uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t YUNEECRCOutputESCBUS::read(uint8_t ch) {
	uint16_t pwm = 0;

	if (ch < ESC_CHANNEL_NUM)
		pwm = _channel_value[ch] & RUN_CHANNEL_VALUE_MASK;

    return pwm;
}

void YUNEECRCOutputESCBUS::read(uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

void YUNEECRCOutputESCBUS::_uart_init(AP_HAL::UARTDriver* uartX) {
	if (uartX == NULL) {
		hal.scheduler->panic("No UART port for ESC, Please specify one!");
		return ; // never reach
	}

	_escbus_uart = uartX;
	_escbus_uart->begin(250000);
}

bool YUNEECRCOutputESCBUS::_escbus_init(void) {
	static const int8_t channelMapTable[ESC_CHANNEL_NUM] = ESCBUS_CHANNEL_MAP_TABLE;
//	uint8_t channelExist = 0;
//	uint32_t last_time = 0;
//	EscbusMessageType msg;
//	EscbusStatusType status;

	// init ESCBUS
	escbus_init(ESC_CHANNEL_NUM);

	hal.scheduler->delay(1000);

	// send config message: 6 channels, No monitor msg, open loop control mode
	for (uint8_t i = 0; i < 5; i++) {
		escbus_send_msg_config_basic(ESC_CHANNEL_NUM, channelMapTable, 0, CONTROL_MODE_CLOSE_LOOP_NO_BREAK);
		hal.scheduler->delay(5);
	}

	hal.scheduler->delay(10);

//	uint32_t start = hal.scheduler->millis();
//	while (hal.scheduler->millis() - start < ESCBUS_DETECT_DELAY) {
//		// flush received message
//		_escbus_uart->flush();
//		// check if there is any ESCBUS connected
//		// try several times
//		for (uint8_t channelID = 0; channelID < ESC_CHANNEL_NUM; channelID++) {
//			if (( channelExist & (1 << channelID) ) == 0) {
//				// request config basic info
//				escbus_send_msg_request_info(channelID, REQUEST_INFO_CONFIG_BASIC_MASK);
//
//				// wait 10ms for rx message
//				last_time = hal.scheduler->millis();
//				while (hal.scheduler->millis() - last_time < 10) {
//					if (_escbus_uart->available()) {
//						uint8_t c = _escbus_uart->read();
//						// parse message
//						if (escbus_parse_char(c, &msg, &status)) {
//							// got what we need
//							if (msg.msgid == ESCBUS_MSG_ID_CONFIG_INFO_BASIC) {
//								EscbusConfigInfoBasicPacket packet;
//								memcpy((uint8_t *)&packet, (const uint8_t *)&(msg.data[0]), ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_BASIC);
//								// double check
//								if (packet.channelID == channelID) {
//									channelExist |= ((uint8_t)1 << channelID);
//								}
//							}
//						}
//					}
//					hal.scheduler->delay(1);
//				}
//			}
//			if (channelExist == ESC_CHANNEL_EXIST_MASK)
//				goto show_esc;
//		}
//	}
//
//show_esc:
//	// show ESCBUS condition
//	bool escbus_ready = true;
//	for (uint8_t i = 0; i < ESC_CHANNEL_NUM; i++) {
//		if (!(channelExist & (1 << i))) {
//			escbus_ready = false;
//			hal.console->printf_P(PSTR("ESCBUS: can't find channel %d\n"), i);
//			hal.scheduler->delay(1);
//		}
//	}

//	if (escbus_ready) {
		// ESC start up tune
			escbus_send_msg_tune(75, 15, 80);
			hal.scheduler->delay(200);
			escbus_send_msg_tune(50, 15, 80);
			hal.scheduler->delay(200);
			escbus_send_msg_tune(10, 15, 80);
			hal.scheduler->delay(200);
//	}

	return true;//escbus_ready;
}

void YUNEECRCOutputESCBUS::_escbus_update(void) {
	if ((hal.scheduler->millis() - _last_update_time >= _update_period)) {
		escbus_send_msg_run(_channel_value, ESC_CHANNEL_NUM);
		_last_update_time = hal.scheduler->millis();
	}
}

#endif
