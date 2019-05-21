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
#include "AP_Proximity_uLandingSt.h"
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
AP_Proximity_uLandingSt::AP_Proximity_uLandingSt(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager) :
		AP_Proximity_Backend(_frontend, _state) {
	Utility::my_fn = (char*)"st_adc";
	uart = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0);
	if (uart != nullptr) {
		uart->begin(
				serial_manager.find_baudrate(
						AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0));
	}

	uart1 = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0);
	if (uart1 != nullptr) {
		uart1->begin(
				serial_manager.find_baudrate(
						AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0));
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_ULANDINGST_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	buf = new uint8_t[2740];
	idx = 0;
	buf1 = new uint8_t[128];
	idx1 = 0;
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_uLandingSt::detect(AP_SerialManager &serial_manager) {
	//
	return serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

/* update the state of the sensor */
void AP_Proximity_uLandingSt::update(void) {
	/* read uSharp Hub */
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_uLandingSt::get_reading(void) {
	if (uart == nullptr) {
		return false ;
	}

	// read beixing radar
	if (uart1 != nullptr) {
		uint32_t nbytes = uart1->available();
		hal.console->printf("beixing read %d bytes\n", nbytes);
		while (nbytes-- > 0) {
			uint8_t d = uart1->read();
			if (idx1 == 0) {
				if (d == 0x59) {
					buf1[idx1++] = d;
				}
			} else if (idx1 == 1) {
				if (d == 0x59) {
					buf1[idx1++] = d;
				} else {
					idx1 = 0;
				}
			} else {
				buf1[idx1++] = d;
				// check tail
				if (idx1 == 9) {
					// check
					uint16_t calc_check = 0;
					for (uint8_t i = 0; i < 8; i++) {
						calc_check += buf1[i];
					}
					if ((calc_check & 0xFF) == buf1[8]) {
						Utility::my_beixing = buf1[2] + buf1[3] * 256;
						hal.console->printf("===================== one frame %d \n", Utility::my_beixing);
					} else {
						hal.console->printf("===================== error frame \n");
					}
					idx1 = 0;
				}
			}
		}
	}

	// read any available lines from the uLanding
	uint32_t nbytes = uart->available();
	hal.console->printf("infineon read %d bytes\n", nbytes);
	while (nbytes-- > 0) {
		uint8_t d = uart->read();
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
			if (buf[idx - 1] == 0xFE && buf[idx - 2] == 0xEF && buf[idx - 3] == 0xFF && buf[idx - 4] == 0xEE) {
				// check length
				if (idx == 2740) {
					hal.console->printf("********************* one frame \n");
					// save
					Utility::write_my_log_byte(buf, 2740);
					Utility::write_my_log_byte(&Utility::my_beixing, 4);
					Utility::write_my_log_byte(&Utility::my_roll, 4);
					Utility::write_my_log_byte(&Utility::my_pitch, 4);
					Utility::write_my_log_byte(&Utility::my_yaw, 4);
				} else {
					hal.console->printf("********************* error frame \n");
				}
				idx = 0;
			} else if (idx > 2740) {
				hal.console->printf("********************* miss tail \n");
				idx = 0;
			}
		}
	}


	for (uint8_t i = 0; i < _num_sectors; i++) {
		_distance[i] = PROXIMITY_ULANDINGST_DISTANCE_MAX;
	}
	return true ;
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_uLandingSt::distance_max() const {
	return PROXIMITY_ULANDINGST_DISTANCE_MAX;
}

float AP_Proximity_uLandingSt::distance_min() const {
	return PROXIMITY_ULANDINGST_DISTANCE_MIN;
}
