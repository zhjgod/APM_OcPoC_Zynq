/*
 * AP_Proximity_usharp60_su.cpp
 *
 *  Created on: March 20, 2020
 *      Author: yc
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_usharp60_su.h"
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
AP_Proximity_usharp60_su::AP_Proximity_usharp60_su(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager) :
		AP_Proximity_Backend(_frontend, _state) {
	Utility::my_fd_name = (char*)"uSharp60_su";
	uart = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0);
	if (uart != nullptr) {
		uart->begin(
				serial_manager.find_baudrate(
						AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0));
	} else {
		hal.console->printf("find_serial null \n");
	}
	uart_wifi = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Wifi, 0);

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_USHARP60_SU_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	memset(buf, 0, 1024);
	buf_idx = 0;
	avoid_dis = PROXIMITY_USHARP60_SU_DISTANCE_MAX;

	//ObstacleAvoiding_initialize();
	//ObstacleAvoiding_terminate();
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_usharp60_su::detect(AP_SerialManager &serial_manager) {
	//
	return serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

/* update the state of the sensor */
void AP_Proximity_usharp60_su::update(void) {
	put_sending();
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_usharp60_su::put_sending(void) {
	if (uart == nullptr) {
		return false;
	}
	int8_t pitch_cor = frontend.get_pitch_correction(state.instance);
	//hal.console->printf("roll:%f, pitch:%f, yaw:%f, alt:%d, pitch_cor:%d \n", 
	//	Utility::my_roll, Utility::my_pitch, Utility::my_yaw, Utility::my_inv_alt, pitch_cor);
	// send head
	uart->write(0x55);
	uart->write(0xAA);
	uart->write(msgNumb); //msg number
	uart->write(0x06); //cmd
	uart->write(0x02); //subcmd
	uart->write((uint8_t)0x00);//len-h
	uart->write(0x11);//len-l
	hal.console->printf("usharp60 write：55 AA %02x 06 02 00 11 ",msgNumb);
	uint16_t sum = 0;

	sum = msgNumb + 0x06 + 0x02 + 0x00 + 0x11;
	msgNumb++;

	// send yaw
	const uint8_t* p = (const uint8_t*)(&Utility::my_yaw);
	uart->write(p, 4);
	for (int i=0; i<4; i++) {
		sum += *(p+i);
		hal.console->printf("%02X ", *(p+i));
	}	
	// send pitch
	p = (const uint8_t*)(&Utility::my_pitch);
	uart->write(p, 4);
	for (int i=0; i<4; i++) {
		sum += *(p+i);
		hal.console->printf("%02X ", *(p+i));
	}
	// send roll
	p = (const uint8_t*)(&Utility::my_roll);
	uart->write(p, 4);
	for (int i=0; i<4; i++) {
		sum += *(p+i);
		hal.console->printf("%02X ", *(p+i));
	}
	// send alt
	p = (const uint8_t*)(&Utility::my_inv_alt);
	uart->write(p, 4);
	for (int i=0; i<4; i++) {
		sum += *(p+i);
		hal.console->printf("%02X ", *(p+i));
	}
	// send pitch_cor
	p = (const uint8_t*)(&pitch_cor);
	sum += *(p);
	uart->write(p, 1);
	hal.console->printf("%02X ", *(p));
	// send sum
	uart->write((uint8_t)((sum>>8)&0xFF));
	hal.console->printf("%02X ", (uint8_t)((sum>>8)&0xFF));
	uart->write((uint8_t)(sum&0xFF));
	hal.console->printf("%02X ", (uint8_t)(sum&0xFF));
	// send tail
	uart->write(0x5A);
	uart->write(0xA5);
	hal.console->printf("5A A5\n");
	return true;
}


bool AP_Proximity_usharp60_su::get_reading(void) {
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
	hal.console->printf("usharp60 read %d bytes. pitch_correction: %d \n", nbytes, pitch_cor);
	while (nbytes-- > 0) {
		uint8_t d = uart->read();
		hal.console->printf("%02X ", d);
		
		buf[buf_idx++] = d;
		if(buf_idx == 1 && d != 0x55)
		{
			buf_idx = 0;
			continue;
		}
		else if(buf_idx == 2 && d != 0xAA)
		{
			buf_idx = 0;
			continue;
		}
		else
		{			
			if(buf_idx == 5)
			{
				if(buf[3] != 0x06)
				{
					buf_idx = 0;
				}
				else if(buf[4] != 0x01 && buf[4] != 0x04)
				{
					buf_idx = 0;
				}
				hal.console->printf("cmd = %d,subCmd = %d\n",buf[3],buf[4]);
				continue;
			}
			
			if(buf[4] == 0x01 && buf_idx == 11)
			{
				target_num = buf[10];
				if (target_num > 64) {
					hal.console->printf("target num is too big %d. \n", target_num);
					buf_idx = 0;
				} 
				else {
					farme_len = 7 + 4 + 9 * target_num + 3 + 4;
				}
				hal.console->printf("farme_len = %d\n",farme_len);
				continue;
			}
			if(buf[4] == 0x04 && buf_idx == 8)
			{
				target_num = buf[7];
				if (target_num > 64) {
					hal.console->printf("target num is too big %d. \n", target_num);
					buf_idx = 0;
				} 
				else {
					farme_len = 7 + 1 + 6 * target_num + 4;
				}
				hal.console->printf("farme_len = %d\n",farme_len);
				continue;
			}
			if(buf_idx >= 1024)
			{	

				buf_idx = 0;
				continue;
			}

			if(buf_idx == farme_len)
			{
				if (buf[buf_idx - 2] == 0x5A && buf[buf_idx - 1] == 0xA5) {
					// end
					uint16_t checksum = buf[buf_idx - 3] + (buf[buf_idx - 4]<<8);
					uint16_t calc_checksum = 0;
					for (int i = 2; i < buf_idx - 4; i++) {
						calc_checksum += buf[i];
					}
					hal.console->printf("checksum = %04x, calc_checksum = %04x\n",checksum,calc_checksum);
					if (calc_checksum == checksum) {
						// good data
						hal.console->printf("good data. \n");
						//
						//uint8_t temp_ori = buf[2];
						//
						if(buf[4] == 0x01)
						{
							uint16_t final_dis = buf[8] * 256 + buf[9];
							avoid_dis = final_dis / 100.0;
							//
							for (int j = 0; j < target_num; j++) {
								uint16_t idx = 11 + 9 * j;
								//
								uint16_t dis = buf[idx] * 256 + buf[idx + 1];
								//
								uint16_t vel_ori = buf[idx + 2] * 256 + buf[idx + 3];
								int16_t vel = vel_ori < 32768 ? vel_ori : vel_ori - 65536;
								//
								uint16_t agl_h_ori = buf[idx + 4] * 256 + buf[idx + 5];
								int16_t agl_h = agl_h_ori < 32768 ? agl_h_ori : agl_h_ori - 65536;
								//
								uint16_t agl_v_ori = buf[idx + 6] * 256 + buf[idx + 7];
								int16_t agl_v = agl_v_ori < 32768 ? agl_v_ori : agl_v_ori - 65536;
								//
								uint8_t snr = buf[idx + 8];
								//
								targets[j].snr = snr;
								targets[j].dis = dis / 100.0;
								targets[j].vel = vel / 10.0;
								targets[j].agl_h = agl_h / 10.0;
								targets[j].agl_v = agl_v / 10.0;
							}
							//
							uint8_t frame_interal = buf[11 + 9 * target_num];
							//
							uint16_t frame_id = buf[11 + 9 * target_num + 1] * 256 + buf[11 + 9 * target_num + 2];
							//
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
											frame_interal,
											frame_id);
							for (int i=0; i<target_num; i++) {
								Utility::write_my_log_str("%f\t%f\t%f\t%f\t%f\n",
										targets[i].dis,
										targets[i].vel,
										targets[i].agl_h,
										targets[i].agl_v,
										targets[i].snr);
							}
							buf_idx = 0;
							continue;
						}
						else if(buf[4] == 0x04)
						{
							memset((uint8_t *)&targets_su,0,sizeof(targets_su));
							for (int j = 0; j < target_num; j++) {
								uint16_t idx = 8 + 6 * j;

								uint16_t t_x = buf[idx+0] << 8 | buf[idx+1];
								int16_t t_y = buf[idx+2] << 8 | buf[idx+3];
								int16_t t_z = buf[idx+4] << 8 | buf[idx+5];

								targets_su[j].x = t_x / 100.0;
								targets_su[j].y = t_y / 100.0;
								targets_su[j].z = t_z / 100.0;
							}
						
							avoid_dis = targets_su[0].x;
							
							//
							uint8_t frame_interal = 0;
							//
							uint16_t frame_id = 0;
							//
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
											frame_interal,
											frame_id);
							for (int i=0; i<target_num; i++) {
								Utility::write_my_log_str("%f\t%f\t%f\n",
										targets_su[i].x,
										targets_su[i].y,
										targets_su[i].z);
							}
							buf_idx = 0;
							continue;
						}
					}
					else {
						// bad data
						hal.console->printf("checksum err. \n");
						buf_idx = 0;
						continue;
					}
				}
				buf_idx = 0;
				continue;
			}			
		}
	}

	Utility::my_prx_dis = avoid_dis;
	if (avoid_dis > 2) {
		_distance[0] = avoid_dis;
	} else {
		_distance[0] = PROXIMITY_USHARP60_SU_DISTANCE_MAX;
	}
	for (uint8_t i = 1; i < _num_sectors; i++) {
		_distance[i] = PROXIMITY_USHARP60_SU_DISTANCE_MAX;
	}

	for (uint8_t sector = 0; sector < _num_sectors; sector++) {
	    update_boundary_for_sector(sector);
	}

	return true;
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_usharp60_su::distance_max() const {
	return PROXIMITY_USHARP60_SU_DISTANCE_MAX;
}

float AP_Proximity_usharp60_su::distance_min() const {
	return PROXIMITY_USHARP60_SU_DISTANCE_MIN;
}


