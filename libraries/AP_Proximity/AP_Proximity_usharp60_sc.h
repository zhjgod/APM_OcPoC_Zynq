#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_HAL_Linux/CANDriver.h>

#define PROXIMITY_USHARP60_SC_DISTANCE_MAX      50.0f
#define PROXIMITY_USHARP60_SC_DISTANCE_MIN      0.5f
#define SC_CAN_FRAME_LEN	8

#define SC_CAN_ID_HEAD   0x1f
#define SC_CAN_DEV_ID		0x01
#define SC_CAN_DATA_LEN  1024
#define SC_CAN_RADAR_ID	0x81

typedef union{
	uint32_t id;
	struct{
		uint32_t subSeq:8;
		uint32_t seq:8;
		uint32_t deviceID:8;
		uint32_t head:5;
		uint32_t reserved:3;
	}idBits;
}scCanID_t;

typedef struct{
	uint16_t dataLen;
	uint16_t dataCrc;
	scCanID_t id;
	uint8_t destDev;
	uint8_t reserved;
	uint8_t crc[2];
	uint8_t cmd;
	uint8_t subCmd;
	uint8_t len[2];
	uint8_t data[SC_CAN_DATA_LEN];
}__attribute__((packed)) scCanData_t;

typedef struct{
	uint8_t flag;		//0=未接受到数据包头，1=标识数据包有效，等待后续数据，2=所有数据接收完成，而且该包数据未使用
	uint8_t devID;		//期望的设备的ID
	uint8_t pkgSeq;		//期望的数据包的序号
	uint8_t frameSeq;	//期望的帧的序号
	uint16_t allLen;	//期望的数据的长度
	uint16_t recvLen;	//接收到的数据的长度
	uint32_t lastTime; 	//上一帧的时间
}scCanPkgInfo_t;



typedef struct{
	scCanPkgInfo_t pkgInfo;
	scCanData_t data;
}scCanDataPkg_t;

struct target_60_sc_1{
    float snr;
    float dis;
    float vel;
    float agl_h;
    float agl_v;
};

struct target_60_sc{ 
	float x;
	float y;
	float z;
	uint8_t snr;
};

class AP_Proximity_usharp60_sc : public AP_Proximity_Backend
{

public:
    /* constructor */
	AP_Proximity_usharp60_sc(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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
    uint16_t SUSum16(uint8_t *fp_data, uint16_t f_len);
	
    Linux::CANDriver *can = nullptr;

	uint8_t target_num = 0;
    uint32_t time_begin = 0;
	float avoid_dis;
	target_60_sc_1 targets[64];
	target_60_sc targets_sc[64];
	scCanDataPkg_t canPkg;
};
