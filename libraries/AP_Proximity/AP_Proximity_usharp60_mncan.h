#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_HAL_Linux/CANDriver.h>

#define PROXIMITY_USHARP60_MNCAN_DISTANCE_MAX      50.0f
#define PROXIMITY_USHARP60_MNCAN_DISTANCE_MIN      0.5f
#define CAN_FRAME_LEN	8

#define MN_CAN_ID_HEAD   0x1f
#define MN_CAN_DEV_ID		0x01
#define MU_NIU_CAN_DATA_LEN  1024
#define MN_CAN_RADAR_ID	0x81

typedef union{
	uint32_t id;
	struct{
		uint32_t subSeq:8;
		uint32_t seq:8;
		uint32_t deviceID:8;
		uint32_t head:5;
		uint32_t reserved:3;
	}idBits;
}mnCanID_t;

typedef struct{
	uint16_t dataLen;
	uint16_t dataCrc;
	mnCanID_t id;
	uint8_t destDev;
	uint8_t reserved;
	uint8_t crc[2];
	uint8_t cmd;
	uint8_t subCmd;
	uint8_t len[2];
	uint8_t data[MU_NIU_CAN_DATA_LEN];
}__attribute__((packed)) mnCanData_t;

typedef struct{
	uint8_t flag;		//0=未接受到数据包头，1=标识数据包有效，等待后续数据，2=所有数据接收完成，而且该包数据未使用
	uint8_t devID;		//期望的设备的ID
	uint8_t pkgSeq;		//期望的数据包的序号
	uint8_t frameSeq;	//期望的帧的序号
	uint16_t allLen;	//期望的数据的长度
	uint16_t recvLen;	//接收到的数据的长度
	uint32_t lastTime; 	//上一帧的时间
}mnCanPkgInfo_t;



typedef struct{
	mnCanPkgInfo_t pkgInfo;
	mnCanData_t data;
}mnCanDataPkg_t;

struct target_60_mncan {
    float snr;
    float dis;
    float vel;
    float agl_h;
    float agl_v;
};

class AP_Proximity_usharp60_mncan : public AP_Proximity_Backend
{

public:
    /* constructor */
	AP_Proximity_usharp60_mncan(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_SerialManager &serial_manager);

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
    uint16_t MNSum16(uint8_t *fp_data, uint8_t f_len);
	uint16_t BoYingCRC16(uint8_t *fp_data, uint8_t f_len);
	
    Linux::CANDriver *can = nullptr;

	uint8_t target_num = 0;
    uint32_t time_begin = 0;
	float avoid_dis;
target_60_mncan targets[64];
	mnCanDataPkg_t canPkg;
};
