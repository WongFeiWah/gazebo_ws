//
// Created by huang on 8/6/18.
//

#ifndef PROJECT_CARRIER_H
#define PROJECT_CARRIER_H

#include <thread>
#include <mutex>
#include <chrono>
#include <comm/mcu_protocol.h>
#include <comm/mcu_package_protocol.h>
#include <comm/zmq_port.h>

enum CarrierStatus
{
  CAR_SPEED_MODE = 0,
  CAR_STEP_MODE,
  CAR_CHARGE_MODE,
  CAR_REFLEX_BACK_MODE,
  CAR_REFLEX_SPIN_MODE,
  CAR_WALLFOLLOW_MODE,
  CAR_DIRECT_MODE,
  CAR_STEPEND_MODE,
  CAR_IDLE_MODE,
};

enum AlongWallStatus
{
  WAL_ALIGN,
  WAL_ALIGN_WAIT,
  WAL_ALONG,
};

namespace Carrier{
  class CarrierCore
  {
  protected:
    std::thread *mMoveModeThread;
    std::thread *mDeltaMoveThread;

    std::thread *mBridgePublisherThread;
    std::thread *mBridgeSubscriberThread;

    void *sensor_handler;

    CarrierStatus mModeStatus;
    AlongWallStatus mWallFollowStatus;

    float move_v;
    float move_w;

    float move_set_v;
    float move_set_w;

    char mBridgePortName[50];

  public:
    CarrierCore(void *sensor_handler, char *bridgeName);
    ~CarrierCore();

    // Move control
    static void OnMoveMode(void *param);
    void setSettingMove(float w, float v) { this->move_set_w = w, this->move_set_v = v; }
    void setMove(float w, float v) { this->move_w = w, this->move_v = v; }
    void StopMove();
    void Move();
    void Move(float w,float v, bool isAcc = true);
    void SetCtlData(ChassisControlRegister mCtrlPackage);
    uint64_t mLastMoveControlTime;

    // status
    bool is_steping;
    bool is_along_wall;

    // DeltaMove
    static void OnDeltaMove(void *param);
    void ClearDeltaMove();
    s32 mDeltaX, mDeltaW;

    // Along Wall
    void AlongWallLoop();
    float wallfollowAlignDstW;
    char last_collier;

    // step
    s32 mStep_s, mStep_phi;

    // protocol
    // mcu protocol
    ChassisControlRegister mCtrlPackage;
    McuPackageProtocol *mMcuPackageProtocol;
    McuFastPackage mcuFastPackage_;
    McuSlowPackage mcuSlowPackage_;
    int SendMcuData(uint8_t *data, uint32_t len);
    void ProcessMcuPackage(const uint8_t *data, int32_t len);

    // Bridge
    CommunicationPort *mBridge;
    static void OnBridgePublisher(void *param);
    static void OnBridgeSubscriber(void *param, const uint8_t *data, uint32_t len);

  };
}

#endif //PROJECT_CARRIER_H
