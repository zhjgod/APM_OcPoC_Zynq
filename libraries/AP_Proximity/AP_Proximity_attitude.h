/*
 * AP_Proximity_attitude.h
 *
 *  Created on: Jun 13, 2018
 *      Author: muniu
 */
#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_ATTITUDE_DISTANCE_MAX      600.0f
#define PROXIMITY_ATTITUDE_DISTANCE_MIN      0.5f


class AP_Proximity_Attitude  : public AP_Proximity_Backend
{
public:
    /* constructor */
	AP_Proximity_Attitude(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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

    void uart_send_int32(int32_t val);

    AP_HAL::UARTDriver *uart = nullptr;
	uint8_t* buf;
    int32_t idx;
};

