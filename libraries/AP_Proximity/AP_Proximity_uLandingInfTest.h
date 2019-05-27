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


#define PROXIMITY_INFTEST_DISTANCE_MAX      999.0f
#define PROXIMITY_INFTEST_DISTANCE_MIN      0.5f

#define INFTEST_BUF_LEN 41

class AP_Proximity_uLandingInfTest  : public AP_Proximity_Backend
{
public:
    /* constructor */
	AP_Proximity_uLandingInfTest(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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

    AP_HAL::UARTDriver *uart = nullptr;
    AP_HAL::UARTDriver *uart_wifi = nullptr;

    uint16_t idx;
    uint8_t buf[INFTEST_BUF_LEN];

	float xadc_coef;
	float vbat;
	uint32_t vbuf[1];
	FILE* xadc_fd;
};

