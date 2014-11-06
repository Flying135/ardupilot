
#include <AP_Common.h>
#include <AP_Math.h>
#include <StorageManager.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <utility/pinmap_typedef.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
AP_HAL::DigitalSource *green_led;
AP_HAL::DigitalSource *red_led;
AP_HAL::DigitalSource *blue_led;

void multiread(AP_HAL::RCInput* in, uint16_t* channels) {
    /* Multi-channel read method: */
	uint8_t valid_channel;
	valid_channel = in->num_channels();
    in->read(channels, 24);
    hal.console->printf_P(
            PSTR("multi      read %d: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n"),
            (int) valid_channel,
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7],
            channels[8], channels[9], channels[10], channels[11],
            channels[12], channels[13], channels[14], channels[15],
            channels[16], channels[17], channels[18], channels[19],
            channels[20], channels[21], channels[22], channels[23]);
}

void individualread(AP_HAL::RCInput* in, uint16_t* channels) {
    /* individual channel read method: */
	uint8_t valid_channel;
	valid_channel = in->num_channels();
    for (int i = 0; i < 24; i++) {
        channels[i] = in->read(i);
    }
    hal.console->printf_P(
            PSTR("individual read %d: %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\r\n"),
            (int) valid_channel,
            channels[0], channels[1], channels[2], channels[3],
            channels[4], channels[5], channels[6], channels[7],
            channels[8], channels[9], channels[10], channels[11],
            channels[12], channels[13], channels[14], channels[15],
            channels[16], channels[17], channels[18], channels[19],
            channels[20], channels[21], channels[22], channels[23]);
}

void loop (void) {
    static int ctr = 0;
    uint16_t channels[24];
    static uint8_t status = 0;

    switch (status) {
    case 0:
        green_led->write(0);
        red_led->write(1);
        blue_led->write(1);
        status = 1;
        break;
    case 1:
        green_led->write(1);
        red_led->write(0);
        blue_led->write(1);
        status = 2;
        break;
    case 2:
        green_led->write(1);
        red_led->write(1);
        blue_led->write(0);
        status = 0;
        break;
    }

    /* Cycle between using the individual read method
     * and the multi read method*/
    if (ctr < 500) {
        multiread(hal.rcin, channels);
    } else {
        individualread(hal.rcin, channels);
        if (ctr > 1000)  ctr = 0;
    }

    hal.scheduler->delay(100);
    ctr++;
}

void setup (void) {
    green_led = hal.gpio->channel(PE9);
    red_led = hal.gpio->channel(PE8);
    blue_led = hal.gpio->channel(PB0);

    green_led->mode(HAL_GPIO_OUTPUT);
    red_led->mode(HAL_GPIO_OUTPUT);
    blue_led->mode(HAL_GPIO_OUTPUT);

    green_led->write(0);
    red_led->write(0);
    blue_led->write(0);

    for (int i = 0; i < 4; i++) {
        green_led->toggle();
        red_led->toggle();
        blue_led->toggle();
        hal.scheduler->delay(250);
    }

    hal.console->print_P(PSTR("RCInput Test Start\r\n"));
}

AP_HAL_MAIN();
