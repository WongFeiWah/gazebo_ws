
#include "comm/mcu_package_protocol.h"
#include <string.h>

#define MCU_CHECK_STASTICS 0
#if MCU_CHECK_STASTICS
static uint32_t errPackages = 0;
static uint32_t okPackages = 0;
static uint32_t totalPackages = 0;

static int32_t intervalCnt = 0;
static uint64_t preDataTime = 0;
static uint32_t minInterval = 1000000;
static uint32_t maxInterval = 0;
static uint32_t avgInterval = 0;
static uint64_t totalInterval = 0;
#endif // MCU_CHECK_STASTICS


McuPackageProtocol::McuPackageProtocol(uint32_t maxPackageSize)
{
	mLastByte = 0;
	mBeginFlag = 0;
	mCtrlFlag = 0;
	mRevOffset = 0;
	mCheckSum = 0;

	mFrameHead = 0xAA;
	mFrameTail = 0x55;
	mFrameCtrl = 0xA5;

	mMaxPackageSize = maxPackageSize;
	mErrorPackageCnt = 0;
	mPackageSize = 0;

	mRxBuffer = new uint8_t[maxPackageSize];
	mTxBuffer = new uint8_t[maxPackageSize];
}

McuPackageProtocol::~McuPackageProtocol()
{
	delete[] mRxBuffer;
	delete[] mTxBuffer;
}

int McuPackageProtocol::PushByte(uint8_t points)
{
	if (((points == mFrameHead) && (mLastByte == mFrameHead)) || (mRevOffset > mMaxPackageSize)) {
		//RESET
		mRevOffset = 0;
		if (mBeginFlag) {
			mErrorPackageCnt++;
		}
		mBeginFlag = 1;
		mLastByte = points;
		return 0;
	}
	if ((points == mFrameTail) && (mLastByte == mFrameTail) && mBeginFlag) {
		mRevOffset--;
		mPackageSize = mRevOffset - 1;
		mCheckSum -= mFrameTail;
		mCheckSum -= mRxBuffer[mPackageSize];
		mLastByte = points;
		mBeginFlag = 0;
		if (mCheckSum == mRxBuffer[mPackageSize]) {
			mCheckSum = 0;
#if MCU_CHECK_STASTICS
            ++okPackages;
            totalPackages = errPackages + okPackages;
            if (!(totalPackages % 1000)) {
                float errRate = errPackages * 100.0 / totalPackages;
                LOGD("MCU Package error statistics:\nerr/ok/total = %d/%d/%d, err rate:%.02f%%\n",
                    errPackages, okPackages, totalPackages, errRate);
            }

            uint64_t now = GetCurrentTick();

            if (preDataTime != 0) {
                uint32_t nowInterval = (uint32_t)(now - preDataTime);

                preDataTime = now;
                totalInterval += nowInterval;
                ++intervalCnt;
                avgInterval = totalInterval / intervalCnt;
                if (nowInterval < minInterval) {
                    minInterval = nowInterval;
                }
                if (nowInterval > maxInterval) {
                    maxInterval = nowInterval;
                }
                if (!(intervalCnt % 1000)) {
                    LOGD("MCU OOOJBK interval statistics:\nmin/avg/max = %d/%d/%d us in %d packages",
                        minInterval, avgInterval, maxInterval, intervalCnt);
                    intervalCnt = 0;
                    totalInterval = 0;
                    minInterval = 1000000;
                    maxInterval = 0;
                    avgInterval = 0;
                    preDataTime = 0;
                }
            } else {
                preDataTime = now;
            }
#endif // MCU_CHECK_STASTICS
			return 1;
		}
		mErrorPackageCnt++;
		mCheckSum = 0;
#if MCU_CHECK_STASTICS
        ++errPackages;
        totalPackages = errPackages + okPackages;
        if (!(totalPackages % 1000)) {
            float errRate = errPackages * 100.0 / totalPackages;
            LOGD("MCU Package error statistics:\nerr/ok/total = %d/%d/%d, err rate:%.02f%%\n",
                errPackages, okPackages, totalPackages, errRate);
        }
#endif // MCU_CHECK_STASTICS
		return -1;
	}
	mLastByte = points;
	if (mBeginFlag) {
		if (mCtrlFlag) {
			mRxBuffer[mRevOffset++] = points;
			mCheckSum += points;
			mCtrlFlag = 0;
			mLastByte = mFrameCtrl;
		}
		else if (points == mFrameCtrl) {
			mCtrlFlag = 1;
		}
		else {
			mRxBuffer[mRevOffset++] = points;
			mCheckSum += points;
		}
	}
	return 0;
}

const uint8_t *McuPackageProtocol::PackageBuffer(const uint8_t *buf, int32_t in_len, int32_t *out_len)
{

	int32_t i;
	uint8_t *pBuf;
	uint8_t check_sum = 0;

	if (buf == NULL)
		return NULL;

	lock_guard<mutex> lck(mMutex);

	pBuf = mTxBuffer;
	*pBuf++ = mFrameHead;
	*pBuf++ = mFrameHead;

	for (i = 0; i < in_len; i++) {
		if ((buf[i] == mFrameCtrl) || (buf[i] == mFrameHead) || (buf[i] == mFrameTail))
			*pBuf++ = mFrameCtrl;

		*pBuf++ = buf[i];
		check_sum += buf[i];

		if ((pBuf - mTxBuffer) > (mMaxPackageSize - 4))
			return NULL;
	}

	if ((check_sum == mFrameCtrl) || (check_sum == mFrameHead) || (check_sum == mFrameTail))
		*pBuf++ = mFrameCtrl;

	*pBuf++ = check_sum;
	*pBuf++ = mFrameTail;
	*pBuf++ = mFrameTail;

	*out_len = (uint32_t)(pBuf - mTxBuffer);

	return mTxBuffer;
}
