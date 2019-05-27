/*
 * AP_Proximity_uSharp3D.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: muniu
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_uLandingInfTest.h"
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
AP_Proximity_uLandingInfTest::AP_Proximity_uLandingInfTest(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager) :
		AP_Proximity_Backend(_frontend, _state) {
	Utility::my_fn = (char*)"infineonTest";
	uart = serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0);
	if (uart != nullptr) {
		uart->begin(
				serial_manager.find_baudrate(
						AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0));
	}

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_INFTEST_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	memset(buf, 0, INFTEST_BUF_LEN);
	idx = 0;
	xadc_coef = 28 / 4096.0f;

}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_uLandingInfTest::detect(AP_SerialManager &serial_manager) {
	//
	return serial_manager.find_serial(
			AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

/* update the state of the sensor */
void AP_Proximity_uLandingInfTest::update(void) {
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_uLandingInfTest::get_reading(void) {
	if (uart == nullptr) {
		return false;
	}


	vbat = -1;
	xadc_fd = fopen("/sys/bus/iio/devices/iio:device0/in_voltage8_raw", "r");
	if (xadc_fd != NULL) {
		if (fscanf(xadc_fd, "%d", vbuf) >= 0) {	
			vbat = vbuf[0] * xadc_coef + 0.2;
		}
		fclose(xadc_fd);
	}

	// read any available lines from the uLanding
	uint32_t nbytes = uart->available();
	hal.console->printf("infineon read %d bytes\n", nbytes);
	while (nbytes-- > 0) {
		uint8_t d = uart->read();
		if (idx == 0) {
			if (d == 0x55) {
				buf[idx++] = d;
			}
		} else {
			buf[idx++] = d;
			// check tail
			if (idx == INFTEST_BUF_LEN) {
				hal.console->printf("********************* one frame \n");
				// save
				Utility::write_my_log_byte(buf, INFTEST_BUF_LEN);
				Utility::write_my_log_byte(&vbat, 4);
			} else if (idx > INFTEST_BUF_LEN) {
				idx = 0;
			}
		}
	}
	
	for (uint8_t i = 0; i < _num_sectors; i++) {
		_distance[i] = PROXIMITY_INFTEST_DISTANCE_MAX;
	}
	return true ;
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_uLandingInfTest::distance_max() const {
	return PROXIMITY_INFTEST_DISTANCE_MAX;
}

float AP_Proximity_uLandingInfTest::distance_min() const {
	return PROXIMITY_INFTEST_DISTANCE_MIN;
}


