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
#include <AP_HAL_Linux/CANDriver.h>
#include "AP_Proximity_TIPI16.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include "../ArduCopter/utility.h"
#include <math.h>

extern const AP_HAL::HAL& hal;

#define PI 3.141592653
/*
 The constructor also initializes the proximity sensor. Note that this
 constructor is not called until detect() returns true, so we
 already know that we should setup the proximity sensor
 */
AP_Proximity_TIPI16::AP_Proximity_TIPI16(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager) :
				AP_Proximity_Backend(_frontend, _state) {
	can = new Linux::CANDriver();
	if (can != nullptr) {
		can->init();
	}

	uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0);
	if (uart != nullptr) {
	    uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0));
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_TIPI16_DISTANCE_MAX;
		_distance_valid[i] = true;
	}
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_TIPI16::detect(AP_SerialManager &serial_manager) {
	//
	return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

/* update the state of the sensor */
void AP_Proximity_TIPI16::update(void) {
	if (can == nullptr) {
		return;
	}
	if (!is_started) {
		start_radar();
	}
	/* read uSharp Hub */
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

void AP_Proximity_TIPI16::start_radar(void) {
	uint8_t tx_data[8] = {0x01, 0xFF, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff};
	can->can_write(tx_data, 8, 0x100);
}

bool AP_Proximity_TIPI16::get_reading(void) {
	//
	if (get_can_data()) {
		if (time_begin == 0) {
			time_begin = AP_HAL::micros();
		}
		uint32_t now = AP_HAL::micros();
		float times_taken = (now - time_begin) / 1000000.0;

		// uart
		if (uart != nullptr) {
			// write head
			uint8_t head[2] = { 0xFF, 0xFF };
			uart->write(head, 2);
			// write content
			char buf[128];
			int len;
			len = sprintf(buf, "%f@(%f,%f,%f,%d,%d,%d)\n", times_taken,
					Utility::my_roll, Utility::my_pitch, Utility::my_yaw,
					Utility::my_latitude, Utility::my_longitude,
					Utility::my_inv_alt);
			for (int i=0; i<len; i++) {
				uart->write(buf[i]);
			}
			////
//			cur_data.numObjOut = 2;
//			for (int i = 0; i < cur_data.numObjOut; i++) {
//				cur_data.objs[i].rangeIdx = 111;
//				cur_data.objs[i].dopplerIdx = 666;
//				cur_data.objs[i].peakVal = 3333;
//				cur_data.objs[i].x = 4.4;
//				cur_data.objs[i].y = 5.5;
//				cur_data.objs[i].z = 6.6;
//			}
			////
			//hal.console->printf("Target count: %d \n", cur_data.number_of_targets);
			for (int i = 0; i < cur_data.number_of_targets; i++) {
				len = sprintf(buf, "%d\t%d\t%d\t%f\t%f\t%f\n",
						0, 0, cur_data.objs[i].snr,
						cur_data.objs[i].dis / 100.0 * sin(cur_data.objs[i].agl / 100.0 / 180. * PI),
						cur_data.objs[i].dis / 100.0 * cos(cur_data.objs[i].agl / 100.0 / 180.0 * PI),
						0.0);
				//hal.console->printf("dis:%d agl:%d \n", cur_data.objs[i].dis, cur_data.objs[i].agl);
				for (int j=0; j<len; j++) {
					uart->write(buf[j]);
				}
			}
			// write tail
			uint8_t tail[2] = { 0xFF, 0xFE };
			uart->write(tail, 2);
		}

		// disk
		Utility::write_my_log("%f@(%f,%f,%f,%d,%d,%d)\n",
				times_taken,
				Utility::my_roll,
				Utility::my_pitch,
				Utility::my_yaw,
				Utility::my_latitude,
				Utility::my_longitude,
				Utility::my_inv_alt);
		for (int i = 0; i < cur_data.number_of_targets; i++) {
			Utility::write_my_log("%d\t%d\t%d\t%f\t%f\t%f\n",
					0,
					0,
					cur_data.objs[i].snr,
					cur_data.objs[i].dis / 100.0 * sin(cur_data.objs[i].agl / 100.0 / 180. * PI),
					cur_data.objs[i].dis / 100.0 * cos(cur_data.objs[i].agl / 100.0 / 180.0 * PI),
					0.0);
		}
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_distance[i] = PROXIMITY_TIPI16_DISTANCE_MAX;
	}
	return true ;
}

bool AP_Proximity_TIPI16::get_can_data(void) {
	//
	uint8_t data[8];
	uint32_t id;
	uint8_t idx = 0;
	cur_data.number_of_targets = -1;
	while (can->can_read(data, 8, &id) > 0) {
		if (id == 0x100) {
			continue;
		} else if (id == 0x480) {
			// start frame
		} else if (id == 0x481) {
			cur_data.number_of_targets = data[4];
			// end frame
		} else if (id >= 0x400 && id <= 0x463) {
			// after track frame
			cur_data.objs[idx].id = data[0];
			cur_data.objs[idx].snr = data[1];
			cur_data.objs[idx].dis = data[2] * 256 + data[3];
			cur_data.objs[idx].vel = data[4] * 256 + data[5];
			cur_data.objs[idx].agl = data[6] * 256 + data[7];
			idx++;
		} else if (id >= 0x500 && id <= 0x563) {
			// before track frame
			cur_data.objs[idx].id = data[0];
			cur_data.objs[idx].snr = data[1];
			cur_data.objs[idx].dis = data[2] * 256 + data[3];
			cur_data.objs[idx].vel = data[4] * 256 + data[5];
			cur_data.objs[idx].agl = data[6] * 256 + data[7];
			idx++;
		} else {
			continue;
		}
	}
	return cur_data.number_of_targets > -1 ;
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_TIPI16::distance_max() const {
	return PROXIMITY_TIPI16_DISTANCE_MAX;
}

float AP_Proximity_TIPI16::distance_min() const {
	return PROXIMITY_TIPI16_DISTANCE_MIN;
}
