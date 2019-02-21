/*
 * AP_Proximity_attitude.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: muniu
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_attitude.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include "../ArduCopter/utility.h"

extern const AP_HAL::HAL& hal;

/*
 The constructor also initializes the proximity sensor. Note that this
 constructor is not called until detect() returns true, so we
 already know that we should setup the proximity sensor
 */
AP_Proximity_Attitude::AP_Proximity_Attitude(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager) :
		AP_Proximity_Backend(_frontend, _state) {
	uart = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0);
	if (uart != nullptr) {
		uart->begin(
				serial_manager.find_baudrate(
						AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0));
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_ATTITUDE_DISTANCE_MAX;
		_distance_valid[i] = true;
	}
	buf = new uint8_t[16];
	idx = 0;
	Utility::my_fd_name = "/beixing_";
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_Attitude::detect(AP_SerialManager &serial_manager) {
	//
	return serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_beixing, 0) != nullptr;
}

/* update the state of the sensor */
void AP_Proximity_Attitude::update(void) {
	/* read uSharp Hub */
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_Attitude::get_reading(void) {
	if (uart == nullptr) {
		return false ;
	}

	// read beixing radar
	uint32_t nbytes = uart->available();
	hal.console->printf("beixing read %d bytes\n", nbytes);
	while (nbytes-- > 0) {
		uint8_t d = uart->read();
		if (idx == 0) {
			if (d == 0x59) {
				buf[idx++] = d;
			}
		} else if (idx == 1) {
			if (d == 0x59) {
				buf[idx++] = d;
			} else {
				idx = 0;
			}
		} else {
			buf[idx++] = d;
			// check tail
			if (idx == 9) {
				// check
				uint16_t calc_check = 0;
				for (uint8_t i = 0; i < 8; i++) {
					calc_check += buf[i];
				}
				if ((calc_check & 0xFF) == buf[8]) {
					Utility::my_beixing = buf[2] + buf[3] * 256;
					Utility::write_my_log_str("%d ", Utility::my_beixing);
					hal.console->printf("===================== one frame %d \n", Utility::my_beixing);
				} else {
					hal.console->printf("===================== error frame \n");
				}
				idx = 0;
			}
		}
	}

//	hal.console->printf("inv_alt:%d\troll:%f\tpitch:%f\tyaw:%f\n",
//				Utility::my_inv_alt,
//				Utility::my_roll,
//				Utility::my_pitch,
//				Utility::my_yaw);

//	if (uart != nullptr) {
//		// head
//		uart->write(0x12);
//		uart->write(0x34);
//		uart->write(0x56);
//		uart->write(0x78);
//		// data
//		uart_send_int32(Utility::my_inv_alt);
//		uart_send_int32((int32_t)(Utility::my_roll*1000000));
//		uart_send_int32((int32_t)(Utility::my_pitch*1000000));
//		uart_send_int32((int32_t)(Utility::my_yaw*1000000));
//		// tail
//	}

//	for (uint8_t i = 0; i < _num_sectors; i++) {
//		_distance[i] = PROXIMITY_ATTITUDE_DISTANCE_MAX;
//	}
	return true;
}

void AP_Proximity_Attitude::uart_send_int32(int32_t val) {
	uint8_t b = (val >> 24);
	uart->write(b);
	b = (val >> 16) & 0xFF;
	uart->write(b);
	b = (val >> 8) & 0xFF;
	uart->write(b);
	b = val & 0xFF;
	uart->write(b);
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_Attitude::distance_max() const {
	return PROXIMITY_ATTITUDE_DISTANCE_MAX;
}

float AP_Proximity_Attitude::distance_min() const {
	return PROXIMITY_ATTITUDE_DISTANCE_MIN;
}


