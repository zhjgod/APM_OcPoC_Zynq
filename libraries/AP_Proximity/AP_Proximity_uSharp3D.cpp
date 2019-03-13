/*
 * AP_Proximity_uSharp3D.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: muniu
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_uSharp3D.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include "../ArduCopter/utility.h"
#include <math.h>

#include "ObstacleAvoiding.h"

extern const AP_HAL::HAL& hal;

/*
 The constructor also initializes the proximity sensor. Note that this
 constructor is not called until detect() returns true, so we
 already know that we should setup the proximity sensor
 */
AP_Proximity_uSharp3D::AP_Proximity_uSharp3D(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager) :
		AP_Proximity_Backend(_frontend, _state) {
	uart = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0);
	if (uart != nullptr) {
		uart->begin(
				serial_manager.find_baudrate(
						AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0));
	}
	uart_wifi = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Wifi, 0);

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_USHARP3D_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	memset(buf, 0, 32);
	buf_idx = 0;
	Utility::my_fd_name = "/3d_";
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_uSharp3D::detect(AP_SerialManager &serial_manager) {
	//
	return serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

/* update the state of the sensor */
void AP_Proximity_uSharp3D::update(void) {
	put_sending();
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_uSharp3D::put_sending(void) {
	if (uart == nullptr) {
		return false;
	}
	int8_t pitch_cor = frontend.get_pitch_correction(state.instance);
	hal.console->printf("roll:%f, pitch:%f, yaw:%f, alt:%d, pitch_cor:%d \n", 
		Utility::my_roll, Utility::my_pitch, Utility::my_yaw, Utility::my_inv_alt, pitch_cor);
	// send head
	uart->write(0x5A);
	uart->write(0xA5);
	uart->write(0x55);
	uart->write(0xAA);
	uint16_t sum = 0;
	// send roll
	const uint8_t* p = (const uint8_t*)(&Utility::my_roll);
	uart->write(p, 4);
	for (int i=0; i<4; i++) {
		sum += *(p+i);
	}
	// send pitch
	p = (const uint8_t*)(&Utility::my_pitch);
	uart->write(p, 4);
	for (int i=0; i<4; i++) {
		sum += *(p+i);
	}
	// send yaw
	p = (const uint8_t*)(&Utility::my_yaw);
	uart->write(p, 4);
	for (int i=0; i<4; i++) {
		sum += *(p+i);
	}
	// send alt
	p = (const uint8_t*)(&Utility::my_inv_alt);
	uart->write(p, 4);
	for (int i=0; i<4; i++) {
		sum += *(p+i);
	}
	// send pitch_cor
	p = (const uint8_t*)(&pitch_cor);
	sum += *(p);
	uart->write(p, 1);
	// send sum
	uart->write((uint8_t)(sum&0xFF));
	// send tail
	uart->write(0xEE);
	uart->write(0xFF);
	uart->write(0xEF);
	uart->write(0xFE);
	return true;
}


bool AP_Proximity_uSharp3D::get_reading(void) {
	if (uart == nullptr) {
		return false;
	}

	// read any available lines from the uLanding
	if (time_begin == 0) {
		time_begin = AP_HAL::micros();
	}
	uint32_t now = AP_HAL::micros();
	float times_taken = (now - time_begin) / 1000000.0;

	int16_t nbytes = uart->available();
	hal.console->printf("usharp3d read %d bytes.\n", nbytes);
	while (nbytes-- > 0) {
		uint8_t d = uart->read();
		if (buf_idx == 0) {
			if (d == 0x55) {
				buf[buf_idx++] = d;
			}
		} else if (buf_idx == 1) {
			if (d == 0xAA) {
				buf[buf_idx++] = d;
			} else {
				buf_idx = 0;
			}
		} else {
			buf[buf_idx++] = d;
			if (buf_idx > 15) {
				// err trip. only 1 targets -> 2 + 1 + 8 + 1 + 1 + 2 = 15
				hal.console->printf("miss tail. \n");
				// clear
				buf_idx = 0;
			} else if (buf[buf_idx - 2] == 0x5A && buf[buf_idx - 1] == 0xA5) {
				// end
				uint8_t checksum = buf[buf_idx - 3];
				uint32_t calc_checksum = 0;
				for (int i = 2; i < buf_idx - 3; i++) {
					calc_checksum += buf[i];
				}
				if ((calc_checksum & 0xFF) == checksum) {
					// good data
					hal.console->printf("good data. \n");
					// state
					uint8_t status = buf[2];
					if (status == 0x01) {
//						for (int j=0; j<buf_idx; j++) {
//							hal.console->printf("%02X ", buf[j]);
//						}
//						hal.console->printf("\n");
						uint8_t target_num = buf[buf_idx - 4];
						if (target_num > 0) {
							// calc matlab
							avoid_dis = ((buf[3]<<16) + (buf[4]<<8) + buf[5]) / 100.0;
//							hal.console->printf("%0f\n ", avoid_dis);
						} else {
							avoid_dis = PROXIMITY_USHARP3D_DISTANCE_MAX;
						}
					} else {
						avoid_dis = PROXIMITY_USHARP3D_DISTANCE_MAX;
					}
					int8_t pitch_cor = frontend.get_pitch_correction(state.instance);
					// record data
					Utility::write_my_log_str("%f@(%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d)\n",
									times_taken,
									Utility::my_roll,
									Utility::my_pitch,
									Utility::my_yaw,
									Utility::my_sona_alt,
									Utility::my_baro_alt,
									Utility::my_inv_alt,
									Utility::my_vel_x,
									Utility::my_vel_y,
									Utility::my_vel_z,
									avoid_dis,
									Utility::my_avoid_flag,
									Utility::my_current_velocity,
									Utility::my_desired_velocity,
									Utility::my_avoid_count,
									status,
									pitch_cor);
				} else {
					// bad data
					hal.console->printf("checksum err. \n");
				}
				// clear
				buf_idx = 0;
			}
		}
	}

	Utility::my_prx_dis = avoid_dis;
	if (avoid_dis > 6) {
		_distance[0] = avoid_dis;
	} else {
		_distance[0] = PROXIMITY_USHARP3D_DISTANCE_MAX;
	}
	for (uint8_t i = 1; i < _num_sectors; i++) {
		_distance[i] = PROXIMITY_USHARP3D_DISTANCE_MAX;
	}

	for (uint8_t sector = 0; sector < _num_sectors; sector++) {
	    update_boundary_for_sector(sector);
	}

	return true;
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_uSharp3D::distance_max() const {
	return PROXIMITY_USHARP3D_DISTANCE_MAX;
}

float AP_Proximity_uSharp3D::distance_min() const {
	return PROXIMITY_USHARP3D_DISTANCE_MIN;
}


