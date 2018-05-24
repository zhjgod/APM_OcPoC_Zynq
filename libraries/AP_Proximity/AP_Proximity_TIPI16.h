#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_HAL_Linux/CANDriver.h>

#define PROXIMITY_TIPI16_DISTANCE_MAX      50.0f
#define PROXIMITY_TIPI16_DISTANCE_MIN      0.5f
#define CAN_FRAME_LEN	8

struct TIPI16_Obj_Data {
	uint8_t id;
	uint8_t snr;
	uint16_t dis;
	int16_t vel;
	int16_t agl;
};

struct TIPI16_Data {
	uint32_t counter;
	int16_t number_of_targets;
	TIPI16_Obj_Data objs[64];
};

class AP_Proximity_TIPI16 : public AP_Proximity_Backend
{

public:
    /* constructor */
	AP_Proximity_TIPI16(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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

    bool is_started = false;

    void start_radar(void);

    TIPI16_Data  cur_data;

    Linux::CANDriver *can = nullptr;
    AP_HAL::UARTDriver *uart = nullptr;

    uint32_t time_begin = 0;
};
