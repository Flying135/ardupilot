#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCOutput.h"
#include <stm32f37x.h>
#include <stm32f37x_gpio.h>
#include <stm32f37x_tim.h>
#include <stm32f37x_rcc.h>
#include <utility/ESCBus_Types.h>
#include <utility/ESCBus.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

#define MIN_ESCUS_UPDATE_PERIODS	5
AP_HAL::UARTDriver* YUNEECRCOutputESCBUS::_escbus_uart = NULL;
int8_t YUNEECRCOutputESCBUS::_rx_enable_table[MAX_CHANNEL_PORT] = {-1, -1, -1, -1, -1, -1, -1, -1};
uint8_t YUNEECRCOutputESCBUS::_update_period = MIN_ESCUS_UPDATE_PERIODS; //Hz 5ms
uint8_t YUNEECRCOutputESCBUS::_channel_enable_mask = 0;
int16_t YUNEECRCOutputESCBUS::_channel_value[ESC_CHANNEL_NUM] = {0};
uint32_t YUNEECRCOutputESCBUS::_last_update_time = 0;

extern "C" {
	void escbus_send_uart(const uint8_t *msgbuf, uint8_t length)
	{
		hal.uartC->write(msgbuf, length);
	}
}

void YUNEECRCOutputESCBUS::init(void* machtnichts) {
	// initiate uart
//	_uart_init(hal.uartC);

	// config probe pin mode
//    hal.gpio->pinMode(ESC_RXCH_S1_PIN, HAL_GPIO_OUTPUT);
//    hal.gpio->pinMode(ESC_RXCH_S2_PIN, HAL_GPIO_OUTPUT);
//    hal.gpio->pinMode(ESC_RXCH_S3_PIN, HAL_GPIO_OUTPUT);

    // initiate ESCBUS
    _escbus_init();

    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&YUNEECRCOutputESCBUS::_escbus_update));
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

    _channel_value[ch] = 0;
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

    if (ch < ESC_CHANNEL_NUM)
    	_channel_value[ch] = pwm & RUN_CHANNEL_VALUE_MASK;
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

//void YUNEECRCOutputESCBUS::rx_enable(uint8_t ch) {
//	hal.gpio->write(ESC_RXCH_S1_PIN, (_rx_enable_table[ch] & 0x01));
//	hal.gpio->write(ESC_RXCH_S2_PIN, (_rx_enable_table[ch] & 0x02));
//	hal.gpio->write(ESC_RXCH_S3_PIN, (_rx_enable_table[ch] & 0x04));
//}

void YUNEECRCOutputESCBUS::_uart_init(AP_HAL::UARTDriver* uartX) {
	if (uartX == NULL) {
		hal.scheduler->panic("No UART port for ESC, Please specify one!");
		return ; // never reach
	}

	_escbus_uart = uartX;
	_escbus_uart->begin(115200);
}

void YUNEECRCOutputESCBUS::_escbus_init(void) {
	static const int8_t channelMapTable[ESC_CHANNEL_NUM] = ESCBUS_CHANNEL_MAP_TABLE;
//	uint8_t channelID;
//	uint8_t rxEnableBits;
//	uint32_t last_time = 0;
//	EscbusMessageType msg;
//	EscbusStatusType status;

	// init ESCBUS
	escbus_init(ESC_CHANNEL_NUM);

	hal.scheduler->delay(1000);

	// send config message: 6 channels, No monitor msg, open loop control mode
	for (int i = 0; i < 5; i++) {
		escbus_send_msg_config_basic(ESC_CHANNEL_NUM, channelMapTable, 0, CONTROL_MODE_CLOSE_LOOP_NO_BREAK);
		hal.scheduler->delay(2);
	}

	// this will help us to get rx probe order for each channel
//	for (channelID = 0; channelID < ESC_CHANNEL_NUM; channelID++) {
//		for (rxEnableBits = 0; rxEnableBits < MAX_CHANNEL_PORT; rxEnableBits++) {
//			// probe the channel to receive message
//			hal.gpio->write(ESC_RXCH_S1_PIN, (rxEnableBits & 0x01));
//			hal.gpio->write(ESC_RXCH_S2_PIN, (rxEnableBits & 0x02));
//			hal.gpio->write(ESC_RXCH_S3_PIN, (rxEnableBits & 0x04));
//
//			// request config basic info
//			escbus_send_msg_request_info(channelID, REQUEST_INFO_CONFIG_BASIC_MASK);
//
//			// wait 5ms for rx message
//			last_time = hal.scheduler->millis();
//			while (hal.scheduler->millis() - last_time < 5) {
//				if (_escbus_uart->available()) {
//					uint8_t c = _escbus_uart->read();
//					// parse message
//					if (escbus_parse_char(c, &msg, &status)) {
//						// got what we need
//						if (msg.msgid == ESCBUS_MSG_ID_CONFIG_INFO_BASIC) {
//					    	EscbusConfigInfoBasicPacket packet;
//							memcpy((char *)&packet, (const char *)&msg.data[0], ESCBUS_MESSAGE_LENGTH_CONFIG_INFO_BASIC);
//							// double check
//							if (packet.channelID == channelID) {
//								_rx_enable_table[channelID] = rxEnableBits;
//								break;
//							}
//
//						}
//					}
//				}
//			}
//
//			// we already get this channelID
//			if (_rx_enable_table[channelID] == rxEnableBits)
//				break;
//			else if (rxEnableBits == (MAX_CHANNEL_PORT - 1)) {
//				hal.console->printf_P(PSTR("RCOutputESCBUS: can't find channel %d\n"), channelID);
//			}
//		}
//	}

	// ESC start up tune
//	for (int i = 0; i < 3; i++) {
//		escbus_send_msg_tune(75, 5, 50);
//		hal.scheduler->delay(20);
//		escbus_send_msg_tune(10, 5, 50);
//		hal.scheduler->delay(20);
//	}

}

void YUNEECRCOutputESCBUS::_escbus_update(void) {
	if ((hal.scheduler->millis() - _last_update_time >= _update_period)) {
		escbus_send_msg_run(_channel_value, ESC_CHANNEL_NUM);
		_last_update_time = hal.scheduler->millis();
	}
}

#endif
