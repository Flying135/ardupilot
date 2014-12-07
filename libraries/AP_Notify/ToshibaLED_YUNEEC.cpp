/*
  ToshibaLED YUNEEC driver
*/
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "ToshibaLED_YUNEEC.h"

extern const AP_HAL::HAL& hal;

#define LEVEL_FACTOR	20.0f / 256

bool ToshibaLED_YUNEEC::hw_init()
{
	hal.gpio->pinMode(_red_pin, HAL_GPIO_OUTPUT);
	hal.gpio->pinMode(_green_pin, HAL_GPIO_OUTPUT);
	hal.gpio->pinMode(_blue_pin, HAL_GPIO_OUTPUT);

	hal.gpio->write(_red_pin, 0);
	hal.gpio->write(_green_pin, 0);
	hal.gpio->write(_blue_pin, 0);

    return true;
}

// set_rgb - set color as a combination of red, green and blue values
bool ToshibaLED_YUNEEC::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
	hal.gpio->write(_red_pin, red);
	hal.gpio->write(_green_pin, green);
	hal.gpio->write(_blue_pin, blue);
    return true;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC
