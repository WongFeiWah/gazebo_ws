
#include "comm/lidar_package_protocol.h"

#define LIDAR_DATA_STATISTICS 0
#if LIDAR_DATA_STATISTICS
static uint32_t errPackages = 0;
static uint32_t okPackages = 0;
static uint32_t totalPackages = 0;
#endif // LIDAR_DATA_STATISTICS

LidarPackageProtocol::LidarPackageProtocol()
{
	mCrcCheckNum = 0;
	mRevOffset = 0;
	mFrameHead = 0x54;
	mOneFramePointNum = 12;
	mIsRecvHeader = false;
	mIsRecvLen = false;
	mRxBuffer = new uint8_t[2048];
}

LidarPackageProtocol::~LidarPackageProtocol()
{
	delete[]mRxBuffer;
}

int LidarPackageProtocol::PushByte(uint8_t points)
{
	if (!mIsRecvHeader) {
		if (points == mFrameHead) {
			mCrcCheckNum = 0;
			mRevOffset = 0;
			mIsRecvHeader = true;
			mRxBuffer[mRevOffset++] = points;
			mCrcCheckNum = crc_table[(mCrcCheckNum ^ points) & 0xff];
		}
	}
	else if (!mIsRecvLen) {
		if (points == mOneFramePointNum) {
			mIsRecvLen = true;
			mRxBuffer[mRevOffset++] = points;
			mCrcCheckNum = crc_table[(mCrcCheckNum ^ points) & 0xff];
		}
		else if ((points & 0x1F) == mOneFramePointNum) {
			mIsRecvLen = true;
			mRxBuffer[mRevOffset++] = points;
			mCrcCheckNum = crc_table[(mCrcCheckNum ^ points) & 0xff];

		}
		else if (points == mFrameHead) {
			mRevOffset = 0;
			mIsRecvHeader = true;
			mRxBuffer[mRevOffset++] = points;
			mCrcCheckNum = 0;
			mCrcCheckNum = crc_table[(mCrcCheckNum ^ points) & 0xff];
		}
		else {
			mIsRecvHeader = false;
		}
	}
	else {
		mRxBuffer[mRevOffset++] = points;
		if (mRevOffset == sizeof(LidarFrameSeg)) {
			if (points == mCrcCheckNum) {
				//check sum success
				mPackageSize = mRevOffset;
				mIsRecvHeader = false;
				mIsRecvLen = false;
#if LIDAR_DATA_STATISTICS
                ++okPackages;
                totalPackages = errPackages + okPackages;
                if (!(totalPackages % 1000)) {
                    float errRate = errPackages * 100.0 / totalPackages;
                    LOGD("Lidar Package error statistics:\nerr/ok/total = %d/%d/%d, err rate:%.02f%%\n",
                        errPackages, okPackages, totalPackages, errRate);
                }
#endif // LIDAR_DATA_STATISTICS
				return true;
			}
			else {
				mIsRecvHeader = false;
				mIsRecvLen = false;
#if LIDAR_DATA_STATISTICS
                ++errPackages;
                totalPackages = errPackages + okPackages;
                if (!(totalPackages % 1000)) {
                    float errRate = errPackages * 100.0 / totalPackages;
                    LOGD("Lidar Package error statistics:\nerr/ok/total = %d/%d/%d, err rate:%.02f%%\n",
                        errPackages, okPackages, totalPackages, errRate);
                }
#endif // LIDAR_DATA_STATISTICS
			}
		}
		else {
			mCrcCheckNum = crc_table[(mCrcCheckNum ^ points) & 0xff];
		}
	}

	return false;
}
