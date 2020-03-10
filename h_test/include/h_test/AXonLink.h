#ifndef __AXON_LINK_H_
#define __AXON_LINK_H_

#include "OniPlatform.h"

#define AXON_LINK_SERIALNUMBER_SIZE 32
#define AXON_LINK_SUPPORTED_PARAMETERS 10
typedef enum{
	AXON_LINK_SENDFILE_STATUS_STOP	= 0,
	AXON_LINK_SENDFILE_STATUS_READY	= 1,
	AXON_LINK_SENDFILE_STATUS_RECVING = 2,
	AXON_LINK_SENDFILE_STATUS_WRITING = 3,
	AXON_LINK_SENDFILE_STATUS_SUCCESS = 4,
	AXON_LINK_SENDFILE_STATUS_FAILED	= 5,
} AXonLinkSendFileStatus;

enum
{
    AXONLINK_DEVICE_PROPERTY_GET_SOFTWARE_VERSION       = 0x60000001, // AXonLinkSWVersion
    AXONLINK_DEVICE_PROPERTY_GET_CAMERA_PARAMETERS      = 0x60000010, // AXonLink Camera paramters

	AXONLINK_DEVICE_INVOKE_GET_UPGRADE_STATUS			= 0x60000020, // AXonLinkSendFileStatus
	AXONLINK_DEVICE_INVOKE_SET_UPLOADFILE				= 0x60000030, // char[]
	AXONLINK_DEVICE_INVOKE_SET_UPGRADE_ENABLE			= 0x60000040, // uchar
	AXONLINK_DEVICE_INVOKE_SET_REBOOT					= 0x60000080, //NULL
	AXONLINK_DEVICE_INVOKE_GET_REGE2_STATUS					= 0x60000090, //short
	AXONLINK_DEVICE_INVOKE_SET_REGE2FILE				= 0x60000100, //uchar;
	AXONLINK_DEVICE_INVOKE_SET_REGE2ENABLE		= 0x60000110, //uchar;
	AXONLINK_DEVICE_INVOKE_SET_DSP_DPINTERFACE		= 0x60000120,//uint32
	AXONLINK_DEVICE_INVOKE_GET_DSP_DPINTERFACE		= 0x60000130, //uint32

	AXONLINK_DEVICE_INVOKE_GET_FWVERSION			= 0x60000160, //AxonLink_DeviceVersion
	AXONLINK_DEVICE_INVOKE_GET_E2PROM_ITEM     = 0x60000170,//AXonLinkCamParam

	AXONLINK_DEVICE_INVOKE_SET_CAMERA_TRIGER_SYNCSIGNAL = 0x60000180, //SyncTrigerSignal

	AXONLINK_DEVICE_COLOR_SENSOR_I2C				= 0x60000050, //unsigned short
	AXONLINK_DEVICE_DEPTH_SENSOR_I2C				= 0x60000060, //unsigned short
	AXONLINK_DEVICE_E2PROM							= 0x60000070, //unsigned short

    AXONLINK_STREAM_PROPERTY_FLIP                   = 0x61000001, // FLIP
	AXONLINK_STREAM_PROPERTY_CROPPING				= 0x62000001, //AXonCropping
	AXONLINK_STREAM_PROPERTY_CALIBRATION			= 0x63000001, // AXonCalibration
	AXONLINK_STREAM_PROPERTY_MOTIONTHRESHOLD			= 0x64000001, //AXonMotionThreshold
	AXONLINK_STREAM_PROPERTY_EXPOSURE_LEVEL				= 0x65000001, //AXonLinkGetExposureLevel   //AXonLinkSetExposureLevel

};
enum
{
	AXONLINK_CAMERA_COLOR_ID						= 0x01,
	AXONLINK_CAMERA_DEPTH_ID						= 0x02,
	AXONLINK_E2PROM									= 0x05,
};                                           //used for write/read I2C

#pragma pack (push, 1)

typedef struct{
	uint32_t m_nhwType;
	uint32_t m_nMajor;
	uint32_t m_nMinor;
	uint32_t m_nmaintenance;
	uint32_t m_nbuild;
	uint8_t m_nyear;
	uint8_t m_nmonth;
	uint8_t m_nday;
	uint8_t m_nhour;
	uint8_t m_nmin;
	uint8_t m_nsec;
	uint8_t reserved[34];
}AXonLinkFWVersion;

typedef struct{
    uint32_t version;
    uint32_t m_nyear: 6;
    uint32_t m_nmonth: 4;
    uint32_t m_nday: 5;
    uint32_t m_nhour: 5;
    uint32_t m_nmin: 6;
    uint32_t m_nsec: 6;
}AXonLinkSWVersion;

typedef struct {
    int32_t ResolutionX;
    int32_t ResolutionY;
    float fx;
    float fy;
    float cx;
    float cy;
	float k1;
	float k2;
	float k3;
	float p1;
	float p2;
	float k4;
	float k5;
	float k6;
}CamIntrinsicParam;

typedef struct {
    float R_Param[9];
    float T_Param[3];
}CamExtrinsicParam;

typedef struct {
    CamExtrinsicParam stExtParam;
    CamIntrinsicParam astDepthParam[AXON_LINK_SUPPORTED_PARAMETERS];
    CamIntrinsicParam astColorParam[AXON_LINK_SUPPORTED_PARAMETERS];
}AXonLinkCamParam;
typedef struct {
	uint16_t regaddress;
	uint16_t i2cvalue;
}I2cValue;
typedef struct {
	short tpye;
	short length;
	short crc;
	short* data;
}E2Reg;
typedef struct{
	uint32_t UNDISTORT:1;
	uint32_t MASK:1;
	uint32_t NR3:1;
	uint32_t NR2:1;
	uint32_t GAMMA:1;
	uint32_t FLYING:1;
	uint32_t FLYING_2:1;
	uint32_t R2Z:1;
	uint32_t reserved:24;
}AXonDSPInterface;
typedef struct
{
	uint8_t curLevel;
	uint8_t write2E2flag; // value could be 0 (don't save) and 1 (save). All other values should not be used.
	uint8_t reserved[2];
}AXonLinkSetExposureLevel;
typedef struct
{
	uint8_t customID;
	uint8_t maxLevel;
	uint8_t curLevel;
	uint8_t reserved;
}AXonLinkGetExposureLevel;
typedef struct
{
	char filename[256];
	unsigned int hwType;

	struct
	{
		unsigned int major;
		unsigned int minor;
		unsigned int maintenance;
		unsigned int build;
	}swVersion;
}AxonLinkFirmWarePacketVersion;
typedef struct
{
	uint16_t originX;
	uint16_t originY;
	uint16_t width;
	uint16_t height;
	uint16_t gx;
	uint16_t gy;
}AXonCropping;
typedef struct
{
	uint16_t type;
	uint16_t Length;
	uint8_t data[500];
}AXonLinkReadE2OnType;
typedef struct
{
	uint8_t enable;
	uint8_t reserved[3];
}AXonCalibration;
typedef struct
{
	uint16_t thresHold;
	uint32_t count;
	uint16_t remain;
}AXonMotionThreshold;
typedef struct
{
	uint16_t len;
	char serialNumber[30];
}AXonBoard_SN;

#pragma pack (pop)

#endif
