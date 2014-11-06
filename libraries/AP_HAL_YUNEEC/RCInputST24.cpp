#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCInput.h"
#include "RCOutput.h"
#include "UARTDriver.h"
#include <stm32f37x.h>
#include <stm32f37x_tim.h>
#include <stm32f37x_misc.h>
#include <utility/pinmap_typedef.h>
#include <utility/st24.h>

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

#define ST24_BIND_TIMEOUT	1000 //ms
#define ST24_PACKET_LENGTH	PACKET_LEGNTH_CHANNELDATA12 + 5

volatile bool YUNEECRCInputST24::_new_input = false;
AP_HAL::UARTDriver* YUNEECRCInputST24::_st24_uart = NULL;
volatile uint8_t YUNEECRCInputST24::_rssi = 0;
volatile uint8_t YUNEECRCInputST24::_rx_count = 0;
volatile uint16_t YUNEECRCInputST24::_channel_count = 0;
volatile uint16_t YUNEECRCInputST24::_channels[ST24_RC_INPUT_CHANNEL] = {0};

void YUNEECRCInputST24::init(void* machtnichts) {
	/* initiate uartC for st24 */
	_st24_init(hal.uartC);

	/* power on st24 */
    hal.gpio->pinMode(_st24_power_pin, HAL_GPIO_OUTPUT);
    hal.gpio->write(_st24_power_pin, 0);
	// config probe pin mode
    hal.gpio->pinMode(ESC_RXCH_S1_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(ESC_RXCH_S2_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(ESC_RXCH_S3_PIN, HAL_GPIO_OUTPUT);
    // open ch8 for ST24 rx
    hal.gpio->write(ESC_RXCH_S1_PIN, 1);
	hal.gpio->write(ESC_RXCH_S2_PIN, 1);
	hal.gpio->write(ESC_RXCH_S3_PIN, 1);

	hal.scheduler->delay(50);

	/* we check the uart ring buffer every 2ms */
    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&YUNEECRCInputST24::_st24_input));

	/* now bind dsm receiver with remote */
	_st24_bind();
}

bool YUNEECRCInputST24::new_input() {
    return _new_input;
}

uint8_t YUNEECRCInputST24::num_channels() {
    return _channel_count;
}

/* constrain captured pulse to be between min and max pulsewidth. */
static inline uint16_t constrain_pulse(uint16_t p) {
    if (p > RC_INPUT_MAX_PULSEWIDTH) return RC_INPUT_MAX_PULSEWIDTH;
    if (p < RC_INPUT_MIN_PULSEWIDTH) return RC_INPUT_MIN_PULSEWIDTH;
    return p;
}

uint16_t YUNEECRCInputST24::read(uint8_t ch) {
    if (ch >= ST24_RC_INPUT_CHANNEL) return 0;

    __disable_irq();
    uint16_t periods = _channels[ch];
    __enable_irq();

    _new_input = false;
    uint16_t pulse = constrain_pulse(periods);
    /* Check for override */
    uint16_t over = _override[ch];
    return (over == 0) ? pulse : over;
}

uint8_t YUNEECRCInputST24::read(uint16_t* periods, uint8_t len) {
    /* constrain len */
    if (len > ST24_RC_INPUT_CHANNEL)
    	len = ST24_RC_INPUT_CHANNEL;

    /* grab channels from isr's memory in critical section */
    __disable_irq();
    for (uint8_t i = 0; i < len; i++) {
        periods[i] = _channels[i];
    }
    __enable_irq();

    /* Outside of critical section, do the math (in place) to scale and
     * constrain the pulse. */
    for (uint8_t i = 0; i < len; i++) {
        periods[i] = constrain_pulse(periods[i]);
        /* check for override */
        if (_override[i] != 0) {
            periods[i] = _override[i];
        }
    }
    _new_input = false;
    return _channel_count;
}

bool YUNEECRCInputST24::set_overrides(int16_t *overrides, uint8_t len) {
    bool res = false;
    for (int i = 0; i < len; i++) {
        res |= set_override(i, overrides[i]);
    }
    return res;
}

bool YUNEECRCInputST24::set_override(uint8_t channel, int16_t override) {
    if (override < 0) return false; /* -1: no change. */
    if (channel < ST24_RC_INPUT_CHANNEL) {
        _override[channel] = override;
        if (override != 0) {
            _new_input = true;
            return true;
        }
    }
    return false;
}

void YUNEECRCInputST24::clear_overrides() {
    for (int i = 0; i < ST24_RC_INPUT_CHANNEL; i++) {
        _override[i] = 0;
    }
}

/**
 * Initialize the ST24 receive functionality
 *
 * Open the uartA for receiving ST24 frames and configure it appropriately
 *
 * Notice that on YUNEEC platform, we can use uartA, which is USART2 of stm32f372
 * and its RX pin as ST24 input port.
 */
void YUNEECRCInputST24::_st24_init(AP_HAL::UARTDriver* uartX) {

	if (uartX == NULL) {
		hal.scheduler->panic("No UART port for ST24, Please specify one!");
		return ; // never reach
	}

	_st24_uart = uartX;
	_st24_uart->begin(115200);
}

/**
 * Handle ST24 satellite receiver bind mode handler
 */
void YUNEECRCInputST24::_st24_bind(void) {
	hal.console->printf_P(PSTR("ST24 is in BINDING mode...\n"));

	/* Wait for tx task complete */
	while (_st24_uart->tx_pending())
		;

	uint32_t start = hal.scheduler->millis();
	/* Check if st24 receiver is already binded */
	while (_new_input == false) {
		uint32_t now = hal.scheduler->millis();
		if (now - start > ST24_BIND_TIMEOUT)
			break;
		hal.scheduler->delay(5);
	}

	if (_new_input == true) {
		hal.console->printf_P(PSTR("ST24 BINDED already\n"));
		return;
	}

	/* It is not binded, send bind command */
	StBindCmd bind_cmd = {0, {'B', 'I', 'N', 'D'}};
	ReceiverFcPacket* bind_packet =  st24_encode_bind(&bind_cmd);

	for (int i = 0; i < 5; i++) {
		_st24_uart->write((const uint8_t *)bind_packet, (bind_packet->length + 3));
		hal.scheduler->delay(1);
	}

	/* Confirm dsm receiver is configured correctly */
	while (_new_input == false)
		;

	if (_new_input == true) {
		hal.console->printf_P(PSTR("ST24 BINDING successfully\n"));
	}
}

/**
 * Called periodically to check for input data from the ST24 UART
 */
void YUNEECRCInputST24::_st24_input(void) {
	uint8_t nbytes = _st24_uart->available();

	for (int i = 0; i < nbytes; i++) {
		uint8_t byte = _st24_uart->read();
		// update input status if we parse successfully
		if (!st24_decode(byte, &_rssi, &_rx_count, &_channel_count, _channels, ST24_RC_INPUT_CHANNEL)) {
			// in order roll pitch throttle yaw
			uint16_t ch_temp = _channels[0];
			_channels[0] = _channels[1];
			_channels[1] = _channels[2];
			_channels[2] = ch_temp;
			_new_input = true;
			return;
		}
	}

}

#endif
