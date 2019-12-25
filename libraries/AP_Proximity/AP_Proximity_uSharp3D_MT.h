/*
 * AP_Proximity_uSharp3D.h
 *
 *  Created on: Jun 13, 2018
 *      Author: muniu
 */
#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_USHARP3D_MT_DISTANCE_MAX      999.0f
#define PROXIMITY_USHARP3D_MT_DISTANCE_MIN      0.5f

struct target_3d_mt {
    float snr;
    float dis;
    float vel;
    float agl_h;
    float agl_v;
};

class AP_Proximity_uSharp3D_MT  : public AP_Proximity_Backend
{
public:
    /* constructor */
	AP_Proximity_uSharp3D_MT(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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
    uint8_t buf[512];
    uint32_t time_begin = 0;
	uint8_t target_num = 0;
	uint16_t farme_len = 0;

    target_3d_mt targets[32];

    float avoid_dis;
};

