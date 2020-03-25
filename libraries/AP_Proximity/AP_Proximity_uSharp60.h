/*
 * AP_Proximity_uSharp60.h
 *
 *  Created on: March 20, 2020
 *      Author: yc
 */
#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_USHARP60_DISTANCE_MAX      999.0f
#define PROXIMITY_USHARP60_DISTANCE_MIN      0.5f

struct target_60 {
    float snr;
    float dis;
    float vel;
    float agl_h;
    float agl_v;
};

class AP_Proximity_uSharp60  : public AP_Proximity_Backend
{
public:
    /* constructor */
	AP_Proximity_uSharp60(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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

	/* send data to sensor */
	bool put_sending(void);

    AP_HAL::UARTDriver *uart = nullptr;
    AP_HAL::UARTDriver *uart_wifi = nullptr;

    uint16_t buf_idx;
    uint8_t buf[1024];
    uint32_t time_begin = 0;
	uint8_t target_num = 0;
	uint16_t farme_len = 0;
	uint8_t msgNumb = 0;

    target_60 targets[64];

    float avoid_dis;
};

