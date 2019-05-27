/*
 * AP_Proximity_uLandingInfTest.h
 *
 *  Created on: Jun 13, 2018
 *      Author: muniu
 */
#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <stdio.h>


#define PROXIMITY_VOL_DISTANCE_MAX      999.0f
#define PROXIMITY_VOL_DISTANCE_MIN      0.5f

class AP_Proximity_vol  : public AP_Proximity_Backend
{
public:
    /* constructor */
	AP_Proximity_vol(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    /* static detection function */
    static bool detect(AP_SerialManager &serial_manager);

    /* update state */
    void update(void);

    /* get maximum and minimum distances (in meters) of sensor */
    float distance_max() const;
    float distance_min() const;

private:
    /* read data from sensor */
    bool get_reading(void);

	float xadc_coef;
	float vbat;
	uint32_t vbuf[1];
	FILE* xadc_fd;
};

