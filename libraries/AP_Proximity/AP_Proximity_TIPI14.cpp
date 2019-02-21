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
#include "AP_Proximity_TIPI14.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>
#include "../ArduCopter/utility.h"

extern const AP_HAL::HAL& hal;

/*
 The constructor also initializes the proximity sensor. Note that this
 constructor is not called until detect() returns true, so we
 already know that we should setup the proximity sensor
 */
AP_Proximity_TIPI14::AP_Proximity_TIPI14(AP_Proximity &_frontend,
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
		_distance[i] = PROXIMITY_TIPI14_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	Utility::my_fd_name = "/tipi14_";
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_TIPI14::detect(AP_SerialManager &serial_manager) {
	//
	return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

/* update the state of the sensor */
void AP_Proximity_TIPI14::update(void) {
	if (can == nullptr) {
		return;
	}
	/* read uSharp Hub */
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_TIPI14::get_reading(void) {
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
			//hal.console->printf("Target count: %d\n", cur_data.numObjOut);
			for (int i = 0; i < cur_data.numObjOut; i++) {
				len = sprintf(buf, "%d\t%d\t%d\t%f\t%f\t%f\n",
						cur_data.objs[i].rangeIdx, cur_data.objs[i].dopplerIdx,
						cur_data.objs[i].peakVal, cur_data.objs[i].x,
						cur_data.objs[i].y, cur_data.objs[i].z);
				for (int j=0; j<len; j++) {
					uart->write(buf[j]);
				}
			}
			// write tail
			uint8_t tail[2] = { 0xFF, 0xFE };
			uart->write(tail, 2);
		}

		// disk
		Utility::write_my_log_str("%f@(%f,%f,%f,%d,%d,%d)\n",
				times_taken,
				Utility::my_roll,
				Utility::my_pitch,
				Utility::my_yaw,
				Utility::my_latitude,
				Utility::my_longitude,
				Utility::my_inv_alt);
		for (int i = 0; i < cur_data.numObjOut; i++) {
			Utility::write_my_log_str("%d\t%d\t%d\t%f\t%f\t%f\n",
					cur_data.objs[i].rangeIdx,
					cur_data.objs[i].dopplerIdx,
					cur_data.objs[i].peakVal,
					cur_data.objs[i].x,
					cur_data.objs[i].y,
					cur_data.objs[i].z);
		}
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_distance[i] = PROXIMITY_TIPI14_DISTANCE_MAX;
	}
	return true ;
}

bool AP_Proximity_TIPI14::get_can_data(void) {
	//
	cur_step = Header_MagicWord;
	uint8_t data[8];
	uint32_t id;
	while (can->can_read(data, 8, &id) > 0) {
		if (memcmp(data, Magic_Word, 8) == 0) {
			cur_step = Header_Version;
			continue;
		}
		switch (cur_step) {
		case Header_Version:
			memcpy(&cur_data.version, data, 4);
			memcpy(&cur_data.totalPacketLen, data + 4, 4);
			cur_step = Header_Platform;
			break;
		case Header_TotalPacketLen:
			break;
		case Header_Platform:
			memcpy(&cur_data.platform, data, 4);
			memcpy(&cur_data.frameNumber, data + 4, 4);
			cur_step = Header_TimeCpuCycles;
			break;
		case Header_FrameNumber:
			break;
		case Header_TimeCpuCycles:
			memcpy(&cur_data.timeCpuCycles, data, 4);
			memcpy(&cur_data.numDetectedObj, data + 4, 4);
			cur_step = Header_NumTLVs;
			break;
		case Header_NumDetectedObj:
			break;
		case Header_NumTLVs: {
			memcpy(&cur_data.numTLVs, data, 4);
			if (cur_data.numDetectedObj > 0) {
				return get_obj_data();
			} else {
				return true;
			}
		}
		default:
			break;
		}
	}
	return false ;
}

bool AP_Proximity_TIPI14::get_obj_data(void) {
	uint8_t data[8];
	uint32_t id, tlvType, tlvLen;
	while (can->can_read(data, 8, &id) > 0) {
		memcpy(&tlvType, data, 4);
		memcpy(&tlvLen, data + 4, 4);
		if (tlvType == TLV_DETECTED_POINTS) {
			// obj data
			// numObjOut xyzQFormat
			if (can->can_read(data, 8, &id) <= 0) {
				return false ;
			}
			memcpy(&cur_data.numObjOut, data, 2);
			memcpy(&cur_data.xyzQFormat, data + 2, 2);
			uint32_t temp = 1 << cur_data.xyzQFormat;
			for (int i = 0; i < cur_data.numObjOut; i++) {
				// rangeIdx dopplerIdx
				if (can->can_read(data, 8, &id) <= 0) {
					return false ;
				}
				memcpy(&cur_data.objs[i].rangeIdx, data, 2);
				memcpy(&cur_data.objs[i].dopplerIdx, data + 2, 2);
				// rangeIdx dopplerIdx
				if (can->can_read(data, 8, &id) <= 0) {
					return false ;
				}
				memcpy(&cur_data.objs[i].peakVal, data, 2);
				uint16_t x, y, z;
				memcpy(&x, data + 2, 2);
				memcpy(&y, data + 4, 2);
				memcpy(&z, data + 6, 2);
				cur_data.objs[i].x = x > 32767 ? x - 65535 : x;
				if (cur_data.objs[i].x > 0) {
					cur_data.objs[i].x = (cur_data.objs[i].x - 0.5) / temp;
				} else if (cur_data.objs[i].x < 0) {
					cur_data.objs[i].x = (cur_data.objs[i].x + 0.5) / temp;
				}
				cur_data.objs[i].y = y > 32767 ? y - 65535 : y;
				if (cur_data.objs[i].y > 0) {
					cur_data.objs[i].y = (cur_data.objs[i].y - 0.5) / temp;
				} else if (cur_data.objs[i].y < 0) {
					cur_data.objs[i].y = (cur_data.objs[i].y + 0.5) / temp;
				}
				cur_data.objs[i].z = z > 32767 ? z - 65535 : z;
				if (cur_data.objs[i].z > 0) {
					cur_data.objs[i].z = (cur_data.objs[i].z - 0.5) / temp;
				} else if (cur_data.objs[i].z < 0) {
					cur_data.objs[i].z = (cur_data.objs[i].z + 0.5) / temp;
				}
			}
			return true;
		}
	}
	return false;
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_TIPI14::distance_max() const {
	return PROXIMITY_TIPI14_DISTANCE_MAX;
}

float AP_Proximity_TIPI14::distance_min() const {
	return PROXIMITY_TIPI14_DISTANCE_MIN;
}
