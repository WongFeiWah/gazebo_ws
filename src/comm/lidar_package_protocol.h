
#ifndef __LIDAR_PACKAGE_PROTOCOL__
#define __LIDAR_PACKAGE_PROTOCOL__

#include <inttypes.h>
#include <comm/LidarProtocol.h>

#define ANGLE_PER_PACK		12

#pragma pack(push, 1) 

typedef struct
{
	uint8_t header;
	uint8_t version_and_length;
	uint16_t speed;
	uint16_t start_angle;
	Lidar2DPoint points[ANGLE_PER_PACK];
	uint16_t end_angle;
	uint16_t timestamp;
	uint8_t crc8;
}LidarFrameSeg;


#pragma pack(pop)


class LidarPackageProtocol
{
public:
	LidarPackageProtocol();
	~LidarPackageProtocol();

	int PushByte(uint8_t points);
	inline const uint8_t *GetRxPackageBuffer() { return mRxBuffer; }
	inline int32_t GetPackageSize() { return mPackageSize; }
private:
	int32_t mPackageSize;
	uint8_t *mRxBuffer;
	uint8_t mCrcCheckNum;
	uint8_t mFrameHead;
	uint8_t mOneFramePointNum;
	bool mIsRecvHeader;
	bool mIsRecvLen;
	int32_t mRevOffset;
};


#endif
