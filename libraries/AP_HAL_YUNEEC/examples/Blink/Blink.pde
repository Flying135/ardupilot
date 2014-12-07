#include <AP_Common.h>
#include <AP_Math.h>
#include <StorageManager.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_YUNEEC.h>
#include <utility/pinmap_typedef.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

AP_HAL::DigitalSource *blue_led;
uint32_t i = 0;

void loop (void) {
//	for (i = 0; i < 2000000; i++) /* Wait a bit. */;
	uint32_t start = hal.scheduler->millis();
    hal.gpio->write(PC5, 1);
    blue_led->write(0);

    hal.scheduler->delay(500);
//    hal.scheduler->delay_microseconds(10);


//	for (i = 0; i < 2000000; i++) /* Wait a bit. */;

	hal.gpio->write(PC5, 0);
    blue_led->write(1);

    hal.scheduler->delay(500);

    uint32_t period = hal.scheduler->millis() - start;
    hal.console->printf("period: %u", period);
//    hal.scheduler->delay_microseconds(10);

}

void setup (void) {

    hal.gpio->pinMode(PC5, HAL_GPIO_OUTPUT);

    blue_led = hal.gpio->channel(PC4);
    blue_led->mode(HAL_GPIO_OUTPUT);

    hal.gpio->write(PC5, 0);
    blue_led->write(0);

}

AP_HAL_MAIN();
