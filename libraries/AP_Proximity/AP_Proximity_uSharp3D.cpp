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

	memset(buf, 0, 256);
	buf_idx = 0;
#ifdef PROTOCAL_STR
	end = true;
	t_idx = 0;
#endif
	avoid_dis = PROXIMITY_USHARP3D_DISTANCE_MAX;

	ObstacleAvoiding_initialize();
	//ObstacleAvoiding_terminate();
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

	int8_t pitch_cor = frontend.get_pitch_correction(state.instance);
	int16_t nbytes = uart->available();
	hal.console->printf("usharp3d read %d bytes. pitch_correction: %d \n", nbytes, pitch_cor);
#ifdef PROTOCAL_BYTE
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
			if (buf_idx > 263) {
				// err trip. at most 32 targets -> 2 + 1 + 8 * 32 + 1 + 1 + 2 = 263
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
					// 
					uint8_t target_num = buf[buf_idx - 4];
					//
					roll_FK = Utility::my_roll;
					pitch_FK = Utility::my_pitch;
					yaw_FK = Utility::my_yaw;
					range_size[0] = 1;
					range_size[1] = target_num;
					angleV_size[0] = 1;
					angleV_size[1] = target_num;
					angleH_size[0] = 1;
					angleH_size[1] = target_num;
					//
					for (int j = 0; j < target_num; j++) {
						uint8_t idx = 3 + 8 * j;
						uint16_t dis = buf[idx] * 256 + buf[idx + 1];
						uint8_t vel_ori = buf[idx + 2];
						int8_t vel = vel_ori < 128 ? vel_ori : vel_ori - 256;
						uint8_t agl_h_ori = buf[idx + 3];
						int8_t agl_h = agl_h_ori < 128 ? agl_h_ori : agl_h_ori - 256;
						uint8_t agl_v_ori = buf[idx + 4];
						int8_t agl_v = agl_v_ori < 128 ? agl_v_ori : agl_v_ori - 256;
						uint8_t snr = buf[idx + 5];
						uint16_t oid = buf[idx + 6] * 256 + buf[idx + 7];
						//
						targets[j].snr = snr;
						targets[j].dis = dis / 100.0;
						targets[j].vel = vel / 10.0;
						targets[j].agl_h = agl_h;
						targets[j].pow = 0;
						targets[j].agl_v = agl_v;
						targets[j].oid = oid;
						range_data[j] = dis / 100.0;
//						angleH_data[j] = agl_h;
//						angleV_data[j] = agl_v;
						angleH_data[j] = agl_v;
						angleV_data[j] = agl_h;
					}
					if (target_num > 0) {
						// calc matlab
						avoid_dis = ObstacleAvoiding(roll_FK, pitch_FK, yaw_FK, range_data, range_size,
									angleV_data, angleV_size, angleH_data, angleH_size, pitch_cor);
					} else {
						avoid_dis = PROXIMITY_USHARP3D_DISTANCE_MAX;
					}
					// record data
					// record data
					Utility::write_my_log("%f@(%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%d,%f,%f,%d)\n",
									times_taken,
									roll_FK,
									pitch_FK,
									yaw_FK,
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
									Utility::my_avoid_count);
					for (int i=0; i<target_num; i++) {
						Utility::write_my_log("%f\t%f\t%f\t%f\t%f\t%f\t%d\n",
								targets[i].snr,
								targets[i].dis,
								targets[i].vel,
								targets[i].agl_h,
								targets[i].pow,
								targets[i].agl_v,
								targets[i].oid);
					}
					// calc

				} else {
					// bad data
					hal.console->printf("checksum err. \n");
				}
				// clear
				buf_idx = 0;
			}
		}
	}
#endif

#ifdef PROTOCAL_STR
//	roll_FK = 0.0711;
//	pitch_FK = 0.098372;
//	yaw_FK = -1.294564;
//	range_size[0] = 1;
//	range_size[1] = 3;
//	angleV_size[0] = 1;
//	angleV_size[1] = 3;
//	angleH_size[0] = 1;
//	angleH_size[1] = 3;
//
//	range_data[0] = 14.02;
//	angleH_data[0] = -24.2;
//	angleV_data[0] = -10.15;
//	range_data[1] = 9.45;
//	angleH_data[1] = -19.42;
//	angleV_data[1] = 10.78;
//	range_data[2] = 23.5;
//	angleH_data[2] = -19.85;
//	angleV_data[2] = 5.19;
//
//	avoid_dis = ObstacleAvoiding(roll_FK, pitch_FK, yaw_FK, range_data, range_size, 
//		angleV_data, angleV_size, angleH_data, angleH_size, 0);
//
//	hal.console->printf("avoid result: %f\r\n", avoid_dis);
	while (nbytes-- > 0) {
		uint8_t d = uart->read();
//		if (uart_wifi != nullptr) {
//			uart_wifi->write(d);
//		}
		char c = (char)d;
		if (end) {
			// find head
			if (buf_idx > 0) {
				buf[buf_idx++] = c;
			}
			if (buf_idx == 0 && c == 'T') {
				buf[buf_idx++] = c;
			}
			if (strlen(buf) == 6) {
				if (memcmp(buf, "TG_NUM", 6) == 0) {
					end = false;
				} else {
					buf_idx = 0;
					memset(buf, 0, 128);
				}
			}
		} else {
			if (buf_idx == 128) {
				end = true;
				buf_idx = 0;
				memset(buf, 0, 128);
				t_idx = 0;
			} else {
				buf[buf_idx++] = c;
				if (strlen(buf) == 3 && memcmp(buf, "END", 3) == 0) {
					end = true;
					buf_idx = 0;
					memset(buf, 0, 128);
					// calc matlab
					if (t_idx > 0) {

						roll_FK = Utility::my_roll;
						pitch_FK = Utility::my_pitch;
						yaw_FK = Utility::my_yaw;
						range_size[0] = 1;
						range_size[1] = t_idx;
						angleV_size[0] = 1;
						angleV_size[1] = t_idx;
						angleH_size[0] = 1;
						angleH_size[1] = t_idx;

						avoid_dis = ObstacleAvoiding(roll_FK, pitch_FK, yaw_FK, range_data, range_size,
								angleV_data, angleV_size, angleH_data, angleH_size, 2);

						hal.console->printf("avoid result: %f\r\n", avoid_dis);
					} else {
						avoid_dis = PROXIMITY_USHARP3D_DISTANCE_MAX;
					}
					// record data
					Utility::write_my_log("%f@(%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%d,%f,%f,%d)\n",
									times_taken,
									roll_FK,
									pitch_FK,
									yaw_FK,
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
									Utility::my_avoid_count);
					for (int i=0; i<t_idx; i++) {
						Utility::write_my_log("%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
								targets[i].snr,
								targets[i].dis,
								targets[i].vel,
								targets[i].agl_h,
								targets[i].pow,
								targets[i].agl_v,
								targets[i].agl_v1);
					}
					t_idx = 0;
				} else if (c == '\n') {
					// decode
					float snr=0, dis=0, vel=0, agl_h=0, pow=0, agl_v=0, agl_v1=0;
					int n = sscanf(buf, "S:%f,R:%f,V:%f,A:%f,POW:%f,E1:%f,E2:%f",
							&snr, &dis, &vel, &agl_h, &pow, &agl_v, &agl_v1);
					if (n == 7) {
						targets[t_idx].snr = snr;
						targets[t_idx].dis = dis;
						targets[t_idx].vel = vel;
						targets[t_idx].agl_h = agl_h;
						targets[t_idx].pow = pow;
						targets[t_idx].agl_v = agl_v;
						targets[t_idx].agl_v1 = agl_v1;
						range_data[t_idx] = dis;
//						angleH_data[t_idx] = agl_h;
//						angleV_data[t_idx] = agl_v;
						angleH_data[t_idx] = agl_v;
						angleV_data[t_idx] = agl_h;
						hal.console->printf("sscanf result: %f %f %f %f %f %f %f %d\r\n",
													snr, dis, vel, agl_h, pow, agl_v, agl_v1, t_idx);
						t_idx++;
					}
					// reset buf
					buf_idx = 0;
					memset(buf, 0, 128);
				}
			}
		}
	}
#endif
//	// disk
//	if (dis > -1) {
//		Utility::write_my_log("%d\t%d\t%d\t%f\t%f\t%f\n",
//				Utility::my_baro_alt,
//				Utility::my_inv_alt,
//				dis,
//				Utility::my_roll,
//				Utility::my_pitch,
//				Utility::my_yaw);
//	}
//	hal.console->printf("baro_alt:%d\tinv_alt:%d\tulandingst_alt:%d\n",
//				Utility::my_baro_alt,
//				Utility::my_inv_alt,
//				dis);
	Utility::my_prx_dis = avoid_dis;
	if (avoid_dis > 2) {
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


