/*
 * AP_Proximity_uSharp3D.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: muniu
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_vol.h"
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
AP_Proximity_vol::AP_Proximity_vol(AP_Proximity &_frontend,
		AP_Proximity::Proximity_State &_state) :
		AP_Proximity_Backend(_frontend, _state) {
			
	Utility::my_fn = (char*)"vol";

	for (uint8_t i = 0; i < _num_sectors; i++) {
		_angle[i] = _sector_middle_deg[i];
		_distance[i] = PROXIMITY_VOL_DISTANCE_MAX;
		_distance_valid[i] = true;
	}

	xadc_coef = 28 / 4096.0f;
}

/* detect if a Aerotenna proximity sensor is connected by looking for a configured serial port */
bool AP_Proximity_vol::detect(AP_SerialManager &serial_manager) {
	//
	return true;
}

/* update the state of the sensor */
void AP_Proximity_vol::update(void) {
	if (get_reading()) {
		set_status(AP_Proximity::Proximity_Good);
	} else {
		set_status(AP_Proximity::Proximity_NoData);
	}
}

bool AP_Proximity_vol::get_reading(void) {

	vbat = -1;
	xadc_fd = fopen("/sys/bus/iio/devices/iio:device0/in_voltage8_raw", "r");
	if (xadc_fd != NULL) {
		if (fscanf(xadc_fd, "%d", vbuf) >= 0) {	
			vbat = vbuf[0] * xadc_coef + 0.2;
		}
		fclose(xadc_fd);
	}
	Utility::write_my_log_str("%f\n", vbat);

	
	for (uint8_t i = 0; i < _num_sectors; i++) {
		_distance[i] = PROXIMITY_VOL_DISTANCE_MAX;
	}
	return true ;
}

/* get maximum and minimum distances (in meters) of primary sensor */
float AP_Proximity_vol::distance_max() const {
	return PROXIMITY_VOL_DISTANCE_MAX;
}

float AP_Proximity_vol::distance_min() const {
	return PROXIMITY_VOL_DISTANCE_MIN;
}


