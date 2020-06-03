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
#include "AP_Proximity_usharp60_sc.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include "../ArduCopter/utility.h"


extern const AP_HAL::HAL& hal;


#if 0
#define SC_CAN_DEBUG_ON

#ifdef SC_CAN_DEBUG_ON
	#define MyPrintf(fmt,...)	hal.console->printf(fmt,##__VA_ARGS__)
#else
	#define MyPrintf(fmt, args...)
#endif
#endif


/*
 The constructor also initializes the proximity sensor. Note that this
 constructor is not called until detect() returns true, so we
 already know that we should setup the proximity sensor
 */
AP_Proximity_usharp60_sc::AP_Proximity_usharp60_sc(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager) :
				AP_Proximity_Backend(_frontend, _state) {
	Utility::my_fd_name = (char*)"uSharp60_sc";
	can = new Linux::CANDriver();
	if (can != nullptr) {
		can->init();
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_USHARP60_SC_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	Utility::my_fd_name = "/usharp60_sc_";
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_usharp60_sc::detect(AP_SerialManager &serial_manager) {
	//
	return true;
}

uint16_t AP_Proximity_usharp60_sc::SUSum16(uint8_t *fp_data, uint16_t f_len) {
 	uint16_t ret = 0;
    	for(int i=0;i<f_len;i++)
	{
		ret += fp_data[i];
	} 


   	 return ret;
}

/* update the state of the sensor */
void AP_Proximity_usharp60_sc::update(void) {
	if (can == nullptr) {
		return;
	}
	send_can_data();
	/* read uSharp Hub */
	if (get_can_data()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_usharp60_sc::get_can_data(void) {
	//
	uint8_t data[8];
	uint32_t id;

	uint16_t sumCheck;
	scCanID_t scCanID;
	int canBytesLen = 0;
	
	uint32_t now = 0;
	float times_taken = 0;
	
	uint8_t frame_interal = 0;
	uint16_t frame_id = 0;
	int i=0;
	uint8_t *p_data= NULL;
	
	
	while (1) {
		canBytesLen = can->can_read(data, 8, &id);
		if(canBytesLen <= 0)
			break;
		

//		hal.console->printf("usharp60_sc read:len = %d:\t%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",canBytesLen-8,id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
		scCanID.id = id;
//		hal.console->printf("ID:reserved=%d,head=%d,deviceID=%d,seq=%d,subSeq=%d\n",scCanID.idBits.reserved,scCanID.idBits.head,scCanID.idBits.deviceID,scCanID.idBits.seq,scCanID.idBits.subSeq);
		
		if(scCanID.idBits.head != SC_CAN_ID_HEAD)
		{
			hal.console->printf("head error\n");
			continue;
		}		
		if(scCanID.idBits.subSeq == 0 && (data[0] != SC_CAN_DEV_ID && data[0] != 0x00))
		{
			hal.console->printf("ID error\n");
			continue;
		}
		
		if(scCanID.idBits.subSeq == 0) //first pkg
		{
			canPkg.data.id.id = id;
			memcpy(&canPkg.data.destDev,data,8);
			canPkg.data.dataLen = canPkg.data.len[0] << 8 | canPkg.data.len[1];
			canPkg.data.dataCrc = canPkg.data.crc[0] << 8 | canPkg.data.crc[1];
			
			
			canPkg.pkgInfo.flag = 1;
			canPkg.pkgInfo.allLen = canPkg.data.dataLen;
			canPkg.pkgInfo.recvLen = 0;
			canPkg.pkgInfo.pkgSeq = scCanID.idBits.seq;
			canPkg.pkgInfo.frameSeq = scCanID.idBits.subSeq+1;
			canPkg.pkgInfo.devID = scCanID.idBits.deviceID;
			
			if(canPkg.pkgInfo.allLen == canPkg.pkgInfo.recvLen)
			{
				sumCheck = SUSum16(&canPkg.data.cmd,canPkg.pkgInfo.allLen+4);
//hal.console->printf("crc1 = %d,crc2=%d\n",sumCheck,canPkg.data.dataCrc);
				if(sumCheck == canPkg.data.dataCrc)
				{
					canPkg.pkgInfo.flag = 2;
				}
				else
				{
					hal.console->printf("crc error\n");
					canPkg.pkgInfo.flag = 0;
					continue;
				}
			}
			else
			{
				continue;
			}
		}
		else
		{
			
//			hal.console->printf("need:%d %d %d %d; current:%d %d %d %d\n",1,canPkg.pkgInfo.pkgSeq,canPkg.pkgInfo.frameSeq,canPkg.pkgInfo.devID,canPkg.pkgInfo.flag,scCanID.idBits.seq,scCanID.idBits.subSeq, scCanID.idBits.deviceID);
			if(canPkg.pkgInfo.flag == 1 && \
					canPkg.pkgInfo.pkgSeq == scCanID.idBits.seq && \
					canPkg.pkgInfo.frameSeq == scCanID.idBits.subSeq && \
					canPkg.pkgInfo.devID == scCanID.idBits.deviceID)
			{
				if(canPkg.pkgInfo.allLen - canPkg.pkgInfo.recvLen >= 8)
					canBytesLen = 8;
				else
					canBytesLen = canPkg.pkgInfo.allLen - canPkg.pkgInfo.recvLen;

				memcpy(canPkg.data.data+canPkg.pkgInfo.recvLen,data,canBytesLen);
				canPkg.pkgInfo.recvLen += canBytesLen;
				canPkg.pkgInfo.frameSeq = scCanID.idBits.subSeq+1;

//hal.console->printf("al=%d,rl=%d\n",canPkg.pkgInfo.allLen,canPkg.pkgInfo.recvLen);

				if(canPkg.pkgInfo.allLen == canPkg.pkgInfo.recvLen)
				{
					sumCheck = SUSum16(&canPkg.data.cmd,canPkg.pkgInfo.allLen+4);
//hal.console->printf("crc1 = %d,crc2=%d\n",sumCheck,canPkg.data.dataCrc);
					if(sumCheck == canPkg.data.dataCrc)
					{
						canPkg.pkgInfo.flag = 2;
					}
					else
					{
						hal.console->printf("crc error\n");
						canPkg.pkgInfo.flag = 0;
						continue;
					}
				}
				else
					continue;

			}
			else
				continue;

		}
		if(canPkg.pkgInfo.flag == 2)
		{
			canPkg.pkgInfo.flag = 0;
//			hal.console->printf(" mn can get full pkg:cmd = %d,subCmd = %d\n",canPkg.data.cmd,canPkg.data.subCmd);
			
			if(canPkg.data.cmd == 0x06 && canPkg.data.subCmd == 0x01)
			{
				
				if (time_begin == 0) 
				{
					time_begin = AP_HAL::micros();
				}
				now = AP_HAL::micros();
				times_taken = (now - time_begin) / 1000000.0;
				time_begin = now;
				p_data = canPkg.data.data;
				uint16_t final_dis = p_data[1] << 8 | p_data[2];
				avoid_dis = final_dis / 100.0;
				
				target_num = p_data[3];
				for(i=0;i<target_num;i++)
				{
					uint16_t idx = 4 + 9 * i;
					//
					uint16_t dis = p_data[idx] * 256 + p_data[idx + 1];
					//
					uint16_t vel_ori = p_data[idx + 2] * 256 + p_data[idx + 3];
					int16_t vel = vel_ori < 32768 ? vel_ori : vel_ori - 65536;
					//
					uint16_t agl_h_ori = p_data[idx + 4] * 256 + p_data[idx + 5];
					int16_t agl_h = agl_h_ori < 32768 ? agl_h_ori : agl_h_ori - 65536;
					//
					uint16_t agl_v_ori = p_data[idx + 6] * 256 + p_data[idx + 7];
					int16_t agl_v = agl_v_ori < 32768 ? agl_v_ori : agl_v_ori - 65536;
					//
					uint8_t snr = p_data[idx + 8];
							
					targets[i].snr = snr;
					targets[i].dis = dis / 100.0;
					targets[i].vel = vel / 10.0;
					targets[i].agl_h = agl_h / 10.0;
					targets[i].agl_v = agl_v / 10.0;

				}
				
				frame_interal = p_data[4 + 9 * target_num];
						//
				frame_id = p_data[4 + 9 * target_num + 1] * 256 + p_data[4 + 9 * target_num + 2];
hal.console->printf("write to log\n");
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
				for (i=0; i<target_num; i++) { //the last is not target
					Utility::write_my_log_str("%f\t%f\t%f\t%f\t%f\n",
							targets[i].dis,
							targets[i].vel,
							targets[i].agl_h,
							targets[i].agl_v,
							targets[i].snr);
				}
				
				Utility::my_prx_dis = avoid_dis;
				if (avoid_dis > 2) {
					_distance[0] = avoid_dis;
				} else {
					_distance[0] = PROXIMITY_USHARP60_SC_DISTANCE_MAX;
				}
				for (i = 1; i < _num_sectors; i++) {
					_distance[i] = PROXIMITY_USHARP60_SC_DISTANCE_MAX;
				}

				for (int sector = 0; sector < _num_sectors; sector++) {
					update_boundary_for_sector(sector);
				}
			}	
			else if(canPkg.data.cmd == 0x06 && canPkg.data.subCmd == 0x04)
			{
				
				if (time_begin == 0) 
				{
					time_begin = AP_HAL::micros();
				}
				now = AP_HAL::micros();
				times_taken = (now - time_begin) / 1000000.0;
				time_begin = now;
				
				p_data = canPkg.data.data;
				target_num = p_data[0];
				
				memset((uint8_t *)&targets_sc,0,sizeof(targets_sc));
				for (int j = 0; j < target_num; j++) {
					uint16_t idx = 1 + 6 * j;

					uint16_t t_x = p_data[idx+0] << 8 | p_data[idx+1];
					int16_t t_y = p_data[idx+2] << 8 | p_data[idx+3];
					int16_t t_z = p_data[idx+4] << 8 | p_data[idx+5];

					targets_sc[j].x = t_x / 100.0;
					targets_sc[j].y = t_y / 100.0;
					targets_sc[j].z = t_z / 100.0;
				}
			
				avoid_dis = targets_sc[0].x;
				
				hal.console->printf("write to log\n");
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
				for (i=0; i<target_num; i++) {
					Utility::write_my_log_str("%f\t%f\t%f\n",
							targets_sc[i].x,
							targets_sc[i].y,
							targets_sc[i].z);
				}
				
				
				Utility::my_prx_dis = avoid_dis;
				if (avoid_dis > 2) {
					_distance[0] = avoid_dis;
				} else {
					_distance[0] = PROXIMITY_USHARP60_SC_DISTANCE_MAX;
				}
				for (i = 1; i < _num_sectors; i++) {
					_distance[i] = PROXIMITY_USHARP60_SC_DISTANCE_MAX;
				}

				for (int sector = 0; sector < _num_sectors; sector++) {
					update_boundary_for_sector(sector);
				}
			}	
			else if(canPkg.data.cmd == 0x06 && canPkg.data.subCmd == 0x05)
			{
				
				if (time_begin == 0) 
				{
					time_begin = AP_HAL::micros();
				}
				now = AP_HAL::micros();
				times_taken = (now - time_begin) / 1000000.0;
				time_begin = now;
				
				p_data = canPkg.data.data;
				target_num = p_data[2];
				frame_id =  p_data[0] << 8 | p_data[1];
				
				memset((uint8_t *)&targets_sc,0,sizeof(targets_sc));
				for (int j = 0; j < target_num; j++) {
					uint16_t idx = 3 + 7 * j;

					uint16_t t_x = p_data[idx+0] << 8 | p_data[idx+1];
					int16_t t_y = p_data[idx+2] << 8 | p_data[idx+3];
					int16_t t_z = p_data[idx+4] << 8 | p_data[idx+5];

					targets_sc[j].x = t_x / 100.0;
					targets_sc[j].y = t_y / 100.0;
					targets_sc[j].z = t_z / 100.0;
					targets_sc[j].snr = p_data[idx+6];
				}
			
				avoid_dis = targets_sc[0].x;
				
				hal.console->printf("write to log\n");
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
				for (i=0; i<target_num; i++) {
					Utility::write_my_log_str("%f\t%f\t%f\t%d\n",
							targets_sc[i].x,
							targets_sc[i].y,
							targets_sc[i].z,
							targets_sc[i].snr);
				}
				
				
				Utility::my_prx_dis = avoid_dis;
				if (avoid_dis > 2) {
					_distance[0] = avoid_dis;
				} else {
					_distance[0] = PROXIMITY_USHARP60_SC_DISTANCE_MAX;
				}
				for (i = 1; i < _num_sectors; i++) {
					_distance[i] = PROXIMITY_USHARP60_SC_DISTANCE_MAX;
				}

				for (int sector = 0; sector < _num_sectors; sector++) {
					update_boundary_for_sector(sector);
				}
			}
			continue;
			
			
		}
			



	}
	return true ;
}
bool AP_Proximity_usharp60_sc::send_can_data(void) {
#if 0   //博鹰协议

	uint8_t data[8];
	uint32_t id;
	float tempF;
	int16_t tempS16;
	uint16_t tempU16;
	uint8_t targetPkg[16];
	int8_t pitch_cor = frontend.get_pitch_correction(state.instance);

	tempF = Utility::my_pitch;
	tempS16 = round(tempF * RADIANS_TO_DEGREES_COEFFICIENT * 100);
	targetPkg[2] = tempS16 & 0xFF;
	targetPkg[3] = (tempS16 >> 8) & 0xFF;

	tempF = Utility::my_roll;
	tempS16 = round(tempF * RADIANS_TO_DEGREES_COEFFICIENT * 100);
	targetPkg[4] = tempS16 & 0xFF;
	targetPkg[5] = (tempS16 >> 8) & 0xFF;

	tempF = Utility::my_yaw;
	tempF += 3.1416;
	tempU16 = round(tempF * RADIANS_TO_DEGREES_COEFFICIENT* 100);
	targetPkg[6] = tempU16 & 0xFF;
	targetPkg[7] = (tempU16 >> 8) & 0xFF;
	
	tempF = Utility::my_inv_alt;
	tempS16 = round(tempF * RADIANS_TO_DEGREES_COEFFICIENT * 100);
	targetPkg[8] = tempS16 & 0xFF;
	targetPkg[9] = (tempS16 >> 8) & 0xFF;

	targetPkg[10] = pitch_cor;
	targetPkg[11] = pitch_cor;

	targetPkg[12] = 0;
	targetPkg[13] = 0;
	targetPkg[14] = 0;
	targetPkg[15] = 0;

	tempU16 = BoYingCRC16(targetPkg+2, 16 - 2);
	targetPkg[0] = tempU16 & 0xFF;
	targetPkg[1] = (tempU16 >> 8) & 0xFF;

	id = CAN_EFF_FLAG | 0x03740403;
	memcpy(data,targetPkg,8);
	
	can->can_write(data, 8, id);

//	hal.console->printf("usharp60_by send:%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

	id = CAN_EFF_FLAG | 0x0374041B;
	memcpy(data,targetPkg+8,8);
	
	can->can_write(data, 8, id);

//	hal.console->printf("usharp60_by send:%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

#elif 1 //木牛协议
	int8_t pitch_cor = frontend.get_pitch_correction(state.instance);
	uint16_t sum = 0;
	uint8_t dataAll[25];
	uint8_t data[8];
	static uint8_t s_seq = 0;
	scCanID_t scCanID; 
	uint32_t id = 0;

	scCanID.idBits.subSeq = 0;
	scCanID.idBits.seq = s_seq;
	scCanID.idBits.deviceID = SC_CAN_DEV_ID;
	scCanID.idBits.head = SC_CAN_ID_HEAD;
	scCanID.idBits.reserved = 0;
	s_seq++;
	
	dataAll[0] = SC_CAN_RADAR_ID;
	dataAll[1] = 0;
	dataAll[4] = 0x06;//cmd
	dataAll[5] = 0x02;//sub cmd
	dataAll[6] = 0;
	dataAll[7] = 0x11;
	
	//  yaw
	const uint8_t* p = (const uint8_t*)(&Utility::my_yaw);
	memcpy(dataAll+8,p,4);
	//  pitch
	p = (const uint8_t*)(&Utility::my_pitch);
	memcpy(dataAll+12,p,4);
	//  roll
	p = (const uint8_t*)(&Utility::my_roll);
	memcpy(dataAll+16,p,4);
	//  alt
	p = (const uint8_t*)(&Utility::my_inv_alt);
	memcpy(dataAll+20,p,4);
	// send pitch_cor
	p = (const uint8_t*)(&pitch_cor);
	dataAll[24] = p[0];

	sum = SUSum16(dataAll+4,dataAll[7]+4);

	dataAll[2] = sum >> 8;
	dataAll[3] = sum & 0xFF;


//第一包
	id = CAN_EFF_FLAG | scCanID.id;
	memcpy(data,dataAll,8);
	
	can->can_write(data, 8, id);

//	hal.console->printf("usharp60_by send:%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

//第二包
	scCanID.idBits.subSeq = 1;
	id = CAN_EFF_FLAG | scCanID.id;
	memcpy(data,dataAll+8,8);
	
	can->can_write(data, 8, id);

//	hal.console->printf("usharp60_by send:%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

//第三包
	scCanID.idBits.subSeq = 2;
	id = CAN_EFF_FLAG | scCanID.id;
	memcpy(data,dataAll+16,8);
	
	can->can_write(data, 8, id);

//	hal.console->printf("usharp60_by send:%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

//第四包
	scCanID.idBits.subSeq = 3;
	id = CAN_EFF_FLAG | scCanID.id;
	memcpy(data,dataAll+24,1);
	
	can->can_write(data, 1, id);

//	hal.console->printf("usharp60_by send:%08x %02x \n",id,data[0]);

#else

#endif
	return true ;

}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_usharp60_sc::distance_max() const {
	return PROXIMITY_USHARP60_SC_DISTANCE_MAX;
}

float AP_Proximity_usharp60_sc::distance_min() const {
	return PROXIMITY_USHARP60_SC_DISTANCE_MIN;
}
