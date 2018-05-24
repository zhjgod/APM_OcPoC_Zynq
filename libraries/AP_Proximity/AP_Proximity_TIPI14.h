#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_HAL_Linux/CANDriver.h>

#define PROXIMITY_TIPI14_DISTANCE_MAX      50.0f
#define PROXIMITY_TIPI14_DISTANCE_MIN      0.5f
#define CAN_FRAME_LEN	8

enum Header_Step {
	Header_MagicWord,
	Header_Version,
	Header_TotalPacketLen,
	Header_Platform,
	Header_FrameNumber,
	Header_TimeCpuCycles,
	Header_NumDetectedObj,
	Header_NumTLVs
};

enum TLV_Types {
	TLV = 0,
	TLV_DETECTED_POINTS,
	TLV_RANGE_PROFILE,
	TLV_NOISE_PROFILE,
	TLV_AZIMUTH_STATIC_HEAT_MAP,
	TLV_RANGE_DOPPLER_HEAT_MAP,
	TLV_STATS,
	TLV_MAX
};

struct TIPI14_Obj_Data {
	uint16_t rangeIdx;
	uint16_t dopplerIdx;
	uint16_t peakVal;
	float x;
	float y;
	float z;
};

struct TIPI14_Data {
	uint32_t version;
	uint32_t totalPacketLen;
	uint32_t platform;
	uint32_t frameNumber;
	uint32_t timeCpuCycles;
	uint32_t numDetectedObj;
	uint32_t numTLVs;
	uint16_t numObjOut;
	uint16_t xyzQFormat;
	TIPI14_Obj_Data objs[64];
};

class AP_Proximity_TIPI14 : public AP_Proximity_Backend
{

public:
    /* constructor */
	AP_Proximity_TIPI14(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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
    bool get_obj_data(void);

    uint8_t Magic_Word[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
    int cur_step;
    TIPI14_Data  cur_data;

    Linux::CANDriver *can = nullptr;
    AP_HAL::UARTDriver *uart = nullptr;

    uint32_t time_begin = 0;
};
