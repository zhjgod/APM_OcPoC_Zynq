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
#include "AP_Proximity_usharp60_mncan.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include "../ArduCopter/utility.h"


extern const AP_HAL::HAL& hal;


#if 0
#define MN_CAN_DEBUG_ON

#ifdef MN_CAN_DEBUG_ON
	#define MyPrintf(fmt,...)	hal.console->printf(fmt,##__VA_ARGS__)
#else
	#define MyPrintf(fmt, args...)
#endif
#endif
// 弧度转度的系数 180/pi = 57.296
#define RADIANS_TO_DEGREES_COEFFICIENT  (57.296f)




const unsigned int gc_crcTable[256] = {
        0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
        0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
        0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
        0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
        0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
        0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
        0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
        0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
        0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
        0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
        0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
        0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
        0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
        0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
        0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
        0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
        0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
        0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
        0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
        0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
        0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
        0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
        0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
        0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
        0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
        0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
        0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
        0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
        0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
        0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
        0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
        0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};



/*
 The constructor also initializes the proximity sensor. Note that this
 constructor is not called until detect() returns true, so we
 already know that we should setup the proximity sensor
 */
AP_Proximity_usharp60_mncan::AP_Proximity_usharp60_mncan(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager) :
				AP_Proximity_Backend(_frontend, _state) {
	Utility::my_fd_name = (char*)"uSharp60_mncan";
	can = new Linux::CANDriver();
	if (can != nullptr) {
		can->init();
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_USHARP60_MNCAN_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	Utility::my_fd_name = "/usharp60_mncan_";
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_usharp60_mncan::detect(AP_SerialManager &serial_manager) {
	//
	return true;
}

uint16_t AP_Proximity_usharp60_mncan::MNSum16(uint8_t *fp_data, uint8_t f_len) {
 	uint16_t ret = 0;
    	for(int i=0;i<f_len;i++)
	{
		ret += fp_data[i];
	} 


   	 return ret;
}

/* update the state of the sensor */
void AP_Proximity_usharp60_mncan::update(void) {
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

uint16_t AP_Proximity_usharp60_mncan::BoYingCRC16(uint8_t *fp_data, uint8_t f_len) {
 	uint16_t value = 0xa635;
    	while (f_len--)
	{
		value = gc_crcTable[(value >> 8 ^ *fp_data++) & 0xff] ^ (value << 8);
	}

   	 return value;
}

bool AP_Proximity_usharp60_mncan::get_can_data(void) {
	//
	uint8_t data[8];
	uint32_t id;

	uint16_t sumCheck;
	mnCanID_t mnCanID;
	int canBytesLen = 0;
	
	uint32_t now = 0;
	float times_taken = 0;
	
	uint8_t frame_interal = 0;
	uint16_t frame_id = 0;
	int i=0;
	uint8_t *p_data= NULL;
	float tempRoll=0,tempPitch=0,tempYaw = 0;
	int16_t tempS16 = 0;
	
	
	while (1) {
		canBytesLen = can->can_read(data, 8, &id);
		if(canBytesLen <= 0)
			break;
		

		//hal.console->printf("usharp60_mncan read:len = %d:\t%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",canBytesLen-8,id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
		mnCanID.id = id;
		//hal.console->printf("ID:reserved=%d,head=%d,deviceID=%d,seq=%d,subSeq=%d\n",mnCanID.idBits.reserved,mnCanID.idBits.head,mnCanID.idBits.deviceID,mnCanID.idBits.seq,mnCanID.idBits.subSeq);
		
		if(mnCanID.idBits.head != MN_CAN_ID_HEAD)
		{
			hal.console->printf("head error\n");
			continue;
		}		
		if(mnCanID.idBits.subSeq == 0 && (data[0] != MN_CAN_DEV_ID && data[0] != 0x00))
		{
			hal.console->printf("ID error\n");
			continue;
		}
		
		if(mnCanID.idBits.subSeq == 0) //first pkg
		{
			canPkg.data.id.id = id;
			memcpy(&canPkg.data.destDev,data,8);
			canPkg.data.dataLen = canPkg.data.len[0] << 8 | canPkg.data.len[1];
			canPkg.data.dataCrc = canPkg.data.crc[0] << 8 | canPkg.data.crc[1];
			
			
			canPkg.pkgInfo.flag = 1;
			canPkg.pkgInfo.allLen = canPkg.data.dataLen;
			canPkg.pkgInfo.recvLen = 0;
			canPkg.pkgInfo.pkgSeq = mnCanID.idBits.seq;
			canPkg.pkgInfo.frameSeq = mnCanID.idBits.subSeq+1;
			canPkg.pkgInfo.devID = mnCanID.idBits.deviceID;
			
			if(canPkg.pkgInfo.allLen == canPkg.pkgInfo.recvLen)
			{
				sumCheck = MNSum16(&canPkg.data.cmd,canPkg.pkgInfo.allLen+4);
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
			
			//hal.console->printf("need:%d %d %d %d; current:%d %d %d %d\n",1,canPkg.pkgInfo.pkgSeq,canPkg.pkgInfo.frameSeq,canPkg.pkgInfo.devID\
,canPkg.pkgInfo.flag,mnCanID.idBits.seq,mnCanID.idBits.subSeq, mnCanID.idBits.deviceID);
			if(canPkg.pkgInfo.flag == 1 && \
					canPkg.pkgInfo.pkgSeq == mnCanID.idBits.seq && \
					canPkg.pkgInfo.frameSeq == mnCanID.idBits.subSeq && \
					canPkg.pkgInfo.devID == mnCanID.idBits.deviceID)
			{
				if(canPkg.pkgInfo.allLen - canPkg.pkgInfo.recvLen >= 8)
					canBytesLen = 8;
				else
					canBytesLen = canPkg.pkgInfo.allLen - canPkg.pkgInfo.recvLen;

				memcpy(canPkg.data.data+canPkg.pkgInfo.recvLen,data,canBytesLen);
				canPkg.pkgInfo.recvLen += canBytesLen;
				canPkg.pkgInfo.frameSeq = mnCanID.idBits.subSeq+1;

//hal.console->printf("al=%d,rl=%d\n",canPkg.pkgInfo.allLen,canPkg.pkgInfo.recvLen);

				if(canPkg.pkgInfo.allLen == canPkg.pkgInfo.recvLen)
				{
					sumCheck = MNSum16(&canPkg.data.cmd,canPkg.pkgInfo.allLen+4);
//hal.console->printf("crc1 = %d,crc2=%d\n",sumCheck,canPkg.data.dataCrc);
					if(sumCheck == canPkg.data.dataCrc)
					{
						canPkg.pkgInfo.flag = 2;
					}
					else
					{
//						hal.console->printf("crc error\n");
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
			hal.console->printf(" mn can get full pkg:cmd = %d,subCmd = %d\n",canPkg.data.cmd,canPkg.data.subCmd);
			
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

					if(i == target_num-1) //the last is not target,it is POS
					{
						tempS16 = p_data[idx] << 8 | p_data[idx + 1];
						tempRoll = tempS16 / 1000.0;

						tempS16 = p_data[idx+2] << 8 | p_data[idx + 3];
						tempPitch = tempS16 / 1000.0;

						tempS16 = p_data[idx+4] << 8 | p_data[idx + 5];
						tempYaw = tempS16 / 1000.0;
					}

				}
				
				frame_interal = p_data[4 + 9 * target_num];
						//
				frame_id = p_data[4 + 9 * target_num + 1] * 256 + p_data[4 + 9 * target_num + 2];
#if 1
hal.console->printf("write to log\n");
				// record data
				Utility::write_my_log_str("%f@(%f,%f,%f,%d,%d,%d,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d)\n",
								times_taken,
								tempRoll,/*Utility::my_roll,*/
								tempPitch,/*Utility::my_pitch,*/
								tempYaw,/*Utility::my_yaw,*/
								Utility::my_sona_alt,
								Utility::my_baro_alt,
								Utility::my_inv_alt,
								Utility::my_roll,
								Utility::my_pitch,
								Utility::my_yaw,
								/*Utility::my_vel_x,
								Utility::my_vel_y,
								Utility::my_vel_z,*/
								avoid_dis,
								Utility::my_avoid_flag,
								Utility::my_current_velocity,
								Utility::my_desired_velocity,
								Utility::my_avoid_count,
								frame_interal,
								frame_id);

				for (i=0; i<target_num-1; i++) { //the last is not target
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
					_distance[0] = PROXIMITY_USHARP60_MNCAN_DISTANCE_MAX;
				}
				for (i = 1; i < _num_sectors; i++) {
					_distance[i] = PROXIMITY_USHARP60_MNCAN_DISTANCE_MAX;
				}

				for (int sector = 0; sector < _num_sectors; sector++) {
					update_boundary_for_sector(sector);
				}
#endif
			}	
			continue;
			
			
		}
			



	}
	return true ;
}
bool AP_Proximity_usharp60_mncan::send_can_data(void) {
#if 1   //博鹰协议

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
	mnCanID_t mnCanID; 
	uint32_t id = 0;

	mnCanID.idBits.subSeq = 0;
	mnCanID.idBits.seq = s_seq;
	mnCanID.idBits.deviceID = MN_CAN_DEV_ID;
	mnCanID.idBits.head = MN_CAN_ID_HEAD;
	mnCanID.idBits.reserved = 0;
	s_seq++;
	
	dataAll[0] = MN_CAN_RADAR_ID;
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

	sum = MNSum16(dataAll+4,dataAll[7]+4);

	dataAll[2] = sum >> 8;
	dataAll[3] = sum & 0xFF;


//第一包
	id = CAN_EFF_FLAG | mnCanID.id;
	memcpy(data,dataAll,8);
	
	can->can_write(data, 8, id);

//	hal.console->printf("usharp60_by send:%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

//第二包
	mnCanID.idBits.subSeq = 1;
	id = CAN_EFF_FLAG | mnCanID.id;
	memcpy(data,dataAll+8,8);
	
	can->can_write(data, 8, id);

//	hal.console->printf("usharp60_by send:%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

//第三包
	mnCanID.idBits.subSeq = 2;
	id = CAN_EFF_FLAG | mnCanID.id;
	memcpy(data,dataAll+16,8);
	
	can->can_write(data, 8, id);

//	hal.console->printf("usharp60_by send:%08x %02x %02x %02x %02x %02x %02x %02x %02x \n",id,data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);

//第四包
	mnCanID.idBits.subSeq = 3;
	id = CAN_EFF_FLAG | mnCanID.id;
	memcpy(data,dataAll+24,1);
	
	can->can_write(data, 1, id);

//	hal.console->printf("usharp60_by send:%08x %02x \n",id,data[0]);

#else

#endif
	return true ;

}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_usharp60_mncan::distance_max() const {
	return PROXIMITY_USHARP60_MNCAN_DISTANCE_MAX;
}

float AP_Proximity_usharp60_mncan::distance_min() const {
	return PROXIMITY_USHARP60_MNCAN_DISTANCE_MIN;
}
