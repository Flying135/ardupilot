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

#ifndef __AP_ST24_TELEM_H__
#define __AP_ST24_TELEM_H__

#include <AP_HAL.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Baro.h>
#include <AP_BattMonitor.h>
#include <utility/st24.h>

class AP_ST24_Telem
{
 public:
    //constructor
    AP_ST24_Telem(AP_AHRS &ahrs, AP_BattMonitor &battery, Compass &compass) :
    _initialised(false),
    _ahrs(ahrs),
    _gps(ahrs.get_gps()),
	_ins(ahrs.get_ins()),
    _battery(battery),
    _baro(ahrs.get_baro()),
	_compass(compass)
        {}

    void init(AP_HAL::UARTDriver *port);
    void send_frames(void);

 private:
    void _st24_send_telemetry(void);
    void _st24_pack_and_send(void);

    AP_HAL::UARTDriver *_port;
    bool _initialised;

    AP_AHRS &_ahrs;
    const AP_GPS &_gps;
    const AP_InertialSensor &_ins;
    AP_BattMonitor &_battery;
    const AP_Baro &_baro;
    Compass &_compass;

	TelemetryData _telem;
	ReceiverFcPacket _txpacket;
};
#endif
