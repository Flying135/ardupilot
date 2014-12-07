#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "RCOutput.h"

using namespace YUNEEC;

extern const AP_HAL::HAL& hal;

YUNEECRCOutputPWM YUNEECRCOutput::_pwm;
YUNEECRCOutputESCBUS YUNEECRCOutput::_escbus;

void YUNEECRCOutput::init(void* machtnichts) {
	/* checkout if there is a ESCBUS connected */
	_escbus_exist = _escbus.init();
	/* set ch1 to ch8 according to whether ESCBUS is connected */
	_pwm.init(_escbus_exist);

	if (_escbus_exist)
		hal.console->printf_P(PSTR("ESCBUS device detected\n"));
	else
		hal.console->printf_P(PSTR("PWM output only\n"));
}

void YUNEECRCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
	/* we can't set channel more than 8 with ESCBUS */
	if (_escbus_exist)
		_escbus.set_freq(chmask & 0x0ff, freq_hz);
	/* set pwm any time in case of servo output */
	_pwm.set_freq(chmask, freq_hz);
}

uint16_t YUNEECRCOutput::get_freq(uint8_t ch) {
	uint16_t freq = 0;
	if (ch <= CH_8) {
		if (_escbus_exist)
			freq = _escbus.get_freq(ch);
		else
			freq = _pwm.get_freq(ch);
	} else
		freq = _pwm.get_freq(ch);

	return freq;
}

void YUNEECRCOutput::enable_ch(uint8_t ch) {
	if (ch <= CH_8) {
		if (_escbus_exist)
			_escbus.enable_ch(ch);
		else
			_pwm.enable_ch(ch);
	} else
		_pwm.enable_ch(ch);
}

void YUNEECRCOutput::disable_ch(uint8_t ch) {
	if (ch <= CH_8) {
		if (_escbus_exist)
			_escbus.disable_ch(ch);
		else
			_pwm.disable_ch(ch);
	} else
		_pwm.disable_ch(ch);
}

void YUNEECRCOutput::write(uint8_t ch, uint16_t period_us) {
	if (ch <= CH_8) {
		if (_escbus_exist)
			_escbus.write(ch, period_us);
		else
			_pwm.write(ch, period_us);
	} else
		_pwm.write(ch, period_us);
}

void YUNEECRCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        write(i + ch, period_us[i]);
    }
}

uint16_t YUNEECRCOutput::read(uint8_t ch) {
	uint16_t value = 0;
	if (ch <= CH_8) {
		if (_escbus_exist)
			value = _escbus.read(ch);
		else
			value = _pwm.read(ch);
	} else
		value = _pwm.read(ch);

	return value;
}

void YUNEECRCOutput::read(uint16_t* period_us, uint8_t len) {
    for (int i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

#endif
