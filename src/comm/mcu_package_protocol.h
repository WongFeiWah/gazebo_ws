
#ifndef __MCU_PACKAGE_PROTOCOL__
#define __MCU_PACKAGE_PROTOCOL__

#include <inttypes.h>
#include <mutex>
using namespace std;

typedef void(*McuPackageCallBack)(void *param, const uint8_t *buffer, int32_t len);

class McuPackageProtocol
{
public:
	McuPackageProtocol(uint32_t maxPackageSize);
	~McuPackageProtocol();

	int PushByte(uint8_t points);
	const uint8_t *PackageBuffer(const uint8_t *buf, int32_t in_len, int32_t *out_len);
	inline const uint8_t *GetRxPackageBuffer() { return mRxBuffer; }
	inline int32_t GetPackageSize() { return mPackageSize; }

private:
	uint8_t mLastByte;
	uint32_t mBeginFlag;
	uint32_t mCtrlFlag;
	int32_t mRevOffset;
	uint8_t mCheckSum;

	int32_t mMaxPackageSize;
	int32_t mErrorPackageCnt;
	int32_t mPackageSize;

	uint8_t *mRxBuffer;
	uint8_t *mTxBuffer;

	uint8_t mFrameHead, mFrameTail, mFrameCtrl;

	mutex mMutex;
};

#endif
