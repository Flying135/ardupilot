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

#include "AP_ST24_Telem.h"

extern const AP_HAL::HAL& hal;

void AP_ST24_Telem::init(AP_HAL::UARTDriver *port)
{
    if (port == NULL) {
    	return;
    }
    _port = port;    
    _port->begin(115200);

    _initialised = true;
}

void  AP_ST24_Telem::_st24_pack_and_send(void)
{
	// format data
	_txpacket.header1 = ST24_STX1;
	_txpacket.header2 = ST24_STX2;
	_txpacket.length = PACKET_LEGNTH_TELEMETRYDATA + 2;
	_txpacket.type = ST24_PACKET_TYPE_TELEMETRYDATA;
	memcpy(_txpacket.st24_data, (const uint8_t *)&_telem, PACKET_LEGNTH_TELEMETRYDATA);
	_txpacket.st24_data[PACKET_LEGNTH_TELEMETRYDATA] = st24_crc8((uint8_t *) & (_txpacket.length), _txpacket.length);

	_port->write((const uint8_t *)&_txpacket, _txpacket.length + 3);
}

void  AP_ST24_Telem::_st24_send_telemetry(void)
{
	// add packet count;
	_telem.t++;

	// GPS info
    bool posok = (_gps.status() >= 3);
    if (posok) {
    	//get _gps instance 0
    	Location loc = _gps.location();
        Vector3f velocity = _gps.velocity();
    	_telem.lat = loc.lat;
    	_telem.lon = loc.lng;
    	_telem.alt = loc.alt;
    	_telem.vx = velocity.x / 100;
    	_telem.vy = velocity.y / 100;
    	_telem.vz = velocity.z / 100;
    	_telem.nsat =  _gps.num_sats();
    } else {
    	_telem.lat = 0;
    	_telem.lon = 0;
    	_telem.alt = 0;
    	_telem.vx = 0;
    	_telem.vy = 0;
    	_telem.vz = 0;
    	_telem.nsat = 0;
    }

    // Battery info
    _telem.voltage = (_battery.voltage() - 5) * 10;
    _telem.current = _battery.current_amps() / 2;

    // Attitude info
    _telem.roll = _ahrs.roll_sensor;
    _telem.pitch = _ahrs.pitch_sensor;
	_telem.yaw = _ahrs.yaw_sensor;

	// Motor info
	//XXX: need read the real motor status
	_telem.motorStatus = 0xff;

	// IMU healthy info
	uint8_t status = 0;
	if (_ins.healthy())
		status |= 0x02;
	// we use MPU6050 as primary sensor
	_telem.imuStatus = status | 0x08;

	// Compass and Baro info
	status = 0;
	if (_compass.healthy())
		status |= 0x02;
	if (_baro.healthy())
		status |= 0x20;
	_telem.pressCompassStatus = status;

	_st24_pack_and_send();
}

/*
  send telemetry frames. Should be called at 50Hz. The high rate is to
  allow this code to poll for serial bytes coming from the receiver
  for the SPort protocol
 */
void AP_ST24_Telem::send_frames(void)
{
    if (!_initialised) {
        return;
    }

    _st24_send_telemetry();
}
