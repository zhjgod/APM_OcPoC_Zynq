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

	uart1 = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0);
	if (uart1 != nullptr) {
		uart1->begin(
				serial_manager.find_baudrate(
						AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0));
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_ULANDINGPRO_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	Utility::my_fd_name = "/pro_";
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
	hal.console->printf("uLandingPro reading: %d \n", nbytes);
	while (nbytes-- > 0) {
		uint8_t c = uart->read();
		// hal.console->printf("%02X ", c);
		// high head
		if (idx == 0 && c == 0xEB) {
			buf[idx++] = c;
		}
		else if (idx == 1) {
			// low head
			if (c == 0x90) {
				buf[idx++] = c;
			} else {
				idx = 0;
			}
		}
		// now it is ready to decode index information
		else {
			buf[idx++] = c;
			if (idx == 32) {
				// check
				uint32_t cal = 0;
				for (int i = 3; i < 31; i++) {
					cal += buf[i];
				}
				if (((cal ^ 0xFF) & 0xFF) == buf[31]) {
					hal.console->printf("********************* one frame %d \n", Utility::my_beixing);
					for (int j = 0; j < 32; j++) {
						Utility::write_my_log_str("%02X ", buf[j]);
					}
					Utility::write_my_log_str("%d %f %f %f %d\n", Utility::my_beixing, Utility::my_roll, Utility::my_pitch, Utility::my_yaw, Utility::my_inv_alt);
				} else {
					hal.console->printf("********************* error frame \n");
				}
				for (int k = 0; k < 32; k++) {
					hal.console->printf("%02X ", buf[k]);
				}
				hal.console->printf("\n");
				idx = 0;
			}
		}
	}

	// disk
//	if (dis > -1) {
//		Utility::write_my_log("%d\t%d\t%d\t%f\t%f\t%f\n",
//					Utility::my_baro_alt,
//					Utility::my_inv_alt,
//					dis,
//					Utility::my_roll,
//					Utility::my_pitch,
//					Utility::my_yaw);
//	}
//	hal.console->printf("baro_alt:%d\tinv_alt:%d\tulandingpro_alt:%d\n",
//			Utility::my_baro_alt,
//			Utility::my_inv_alt,
//			dis);

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
