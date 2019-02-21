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
#include "AP_Proximity_uLandingSt_by.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include "../ArduCopter/utility.h"
#include <math.h>

extern const AP_HAL::HAL& hal;

union {
	float f;
	unsigned char x[4];
} data;



/*
 The constructor also initializes the proximity sensor. Note that this
 constructor is not called until detect() returns true, so we
 already know that we should setup the proximity sensor
 */
AP_Proximity_uLandingSt_by::AP_Proximity_uLandingSt_by(AP_Proximity &_frontend,
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
		_distance[i] = PROXIMITY_ULANDINGST_BY_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	buf = new uint8_t[512];
	idx = 0;
	Utility::my_fd_name = "/st_alt_";
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_uLandingSt_by::detect(AP_SerialManager &serial_manager) {
	//
	return serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

/* update the state of the sensor */
void AP_Proximity_uLandingSt_by::update(void) {
	/* read uSharp Hub */
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_uLandingSt_by::get_reading(void) {
	if (uart == nullptr) {
		return false ;
	}

	if (time_begin == 0) {
		time_begin = AP_HAL::micros();
	}
	uint32_t now = AP_HAL::micros();
	float times_taken = (now - time_begin) / 1000000.0;

	// read any available lines from the uLanding
	uint32_t nbytes = uart->available();
	hal.console->printf("st_alt read %d bytes\n", nbytes);
	while (nbytes-- > 0) {
		uint8_t d = uart->read();
		hal.console->printf("0x%02X ", d);
		if (idx == 0) {
			if (d == 0x5A) {
				buf[idx++] = d;
			}
		} else if (idx == 1) {
			if (d == 0xA5) {
				buf[idx++] = d;
			} else {
				idx = 0;
			}
		} else if (idx == 2) {
			if (d == 0x55) {
				buf[idx++] = d;
			} else {
				idx = 0;
			}
		} else if (idx == 3) {
			if (d == 0xAA) {
				buf[idx++] = d;
				hal.console->printf("********************* find head \n");
			} else {
				idx = 0;
			}
		} else {
			buf[idx++] = d;
			// check tail
			if (idx == 32) {
				// save
				data.x[0] = buf[4];
				data.x[1] = buf[5];
				data.x[2] = buf[6];
				data.x[3] = buf[7];
				float range_mean = data.f;
				data.x[0] = buf[16];
				data.x[1] = buf[17];
				data.x[2] = buf[18];
				data.x[3] = buf[19];
				float range_val = data.f;
				hal.console->printf("********************* one frame %f %f\n", range_mean, range_val);
				Utility::write_my_log_str("%f\t%f\t%d\t%d\t%f\n", range_mean, range_val, Utility::my_baro_alt, Utility::my_inv_alt, times_taken);
				idx = 0;
			}
		}
	}


	for (uint8_t i = 0; i < _num_sectors; i++) {
		_distance[i] = PROXIMITY_ULANDINGST_BY_DISTANCE_MAX;
		
	}
	return true ;
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_uLandingSt_by::distance_max() const {
	return PROXIMITY_ULANDINGST_BY_DISTANCE_MAX;
}

float AP_Proximity_uLandingSt_by::distance_min() const {
	return PROXIMITY_ULANDINGST_BY_DISTANCE_MIN;
}
