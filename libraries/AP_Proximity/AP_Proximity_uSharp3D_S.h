/*
 * AP_Proximity_uSharp3D.h
 *
 *  Created on: Jun 13, 2018
 *      Author: muniu
 */
#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

#define PROXIMITY_USHARP3D_S_DISTANCE_MAX      999.0f
#define PROXIMITY_USHARP3D_S_DISTANCE_MIN      0.5f

struct target_3d_s {
    float snr;
    float dis;
    float vel;
    float agl_h;
	float pow;
    float agl_v;
	float oid;
};

class AP_Proximity_uSharp3D_S  : public AP_Proximity_Backend
{
public:
    /* constructor */
	AP_Proximity_uSharp3D_S(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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
    uint8_t buf[256];
    uint32_t time_begin = 0;

    target_3d_s targets[32];

    double roll_FK;
    double pitch_FK;
    double yaw_FK;
    double range_data[32];
    int range_size[2];
    double angleV_data[32];
    int angleV_size[2];
    double angleH_data[32];
    int angleH_size[2];

    float avoid_dis;
};

