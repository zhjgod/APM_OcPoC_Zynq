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

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_uLandingPro.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include "../ArduCopter/utility.h"
#include <math.h>

extern const AP_HAL::HAL& hal;

/*
 The constructor also initializes the proximity sensor. Note that this
 constructor is not called until detect() returns true, so we
 already know that we should setup the proximity sensor
 */
AP_Proximity_uLandingPro::AP_Proximity_uLandingPro(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager) :
		AP_Proximity_Backend(_frontend, _state) {
	uart = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0);
	if (uart != nullptr) {
		uart->begin(
				serial_manager.find_baudrate(
						AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0));
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_ULANDINGPRO_DISTANCE_MAX;
		_distance_valid[i] = true;
	}
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_uLandingPro::detect(AP_SerialManager &serial_manager) {
	//
	return serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

/* update the state of the sensor */
void AP_Proximity_uLandingPro::update(void) {
	/* read uSharp Hub */
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_uLandingPro::get_reading(void) {
	if (uart == nullptr) {
		return false ;
	}

	// read any available lines from the uLanding
	uint16_t dis = 0;
	uint8_t index = 0;

	int16_t nbytes = uart->available();
	// hal.console->printf("uLandingPro reading: ");
	while (nbytes-- > 0) {
		uint8_t c = uart->read();
		// hal.console->printf("%02X ", c);
		// high head
		if (c == 0xEB && index == 0) {
			// low head
			if (nbytes-- > 0) {
				c = uart->read();
				if (c == 0x90) {
					index = 1;
					linebuf_len = 0;
					continue;
				}
			}
		}
		// now it is ready to decode index information
		if (index == 1) {
			linebuf[linebuf_len++] = c;
			if (linebuf_len == 9) {
				dis = (linebuf[4] << 8) + linebuf[5];
				index = 0;
				linebuf_len = 0;
			}
		}
	}
	// hal.console->printf("\n");

	// disk
	Utility::write_my_log("%d\t%d\t%d\t%f\t%f\t%f\n",
				Utility::my_baro_alt,
				Utility::my_inv_alt,
				dis,
				Utility::my_roll,
				Utility::my_pitch,
				Utility::my_yaw);
	hal.console->printf("baro_alt:%d\tinv_alt:%d\tulandingpro_alt:%d\n",
			Utility::my_baro_alt,
			Utility::my_inv_alt,
			dis);

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_distance[i] = PROXIMITY_ULANDINGPRO_DISTANCE_MAX;
	}
	return true ;
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_uLandingPro::distance_max() const {
	return PROXIMITY_ULANDINGPRO_DISTANCE_MAX;
}

float AP_Proximity_uLandingPro::distance_min() const {
	return PROXIMITY_ULANDINGPRO_DISTANCE_MIN;
}
