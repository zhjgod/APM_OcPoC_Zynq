#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_HAL_Linux/CANDriver.h>

#define PROXIMITY_USHARP60_BY_DISTANCE_MAX      50.0f
#define PROXIMITY_USHARP60_BY_DISTANCE_MIN      0.5f
#define CAN_FRAME_LEN	8

typedef struct{
	uint16_t target1x;
	int16_t  target1y;
	uint16_t target2x;
	int16_t  target2y;
	uint16_t target3x;
	int16_t  target3y;
}canTargets_t;



class AP_Proximity_usharp60_by : public AP_Proximity_Backend
{

public:
    /* constructor */
	AP_Proximity_usharp60_by(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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
    bool get_can_data(void);
    bool send_can_data(void);
    uint16_t BoYingCRC16(uint8_t *fp_data, uint8_t f_len);

    Linux::CANDriver *can = nullptr;


    uint32_t time_begin = 0;
};
