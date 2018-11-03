//
// Created by huang on 7/25/18.
//

#include <chrono>
#include <ctime>
#include <ratio>
#include <math.h>
#include <unistd.h>
#include <CarrierSensor.h>
#include <comm/mcu_protocol.h>
#include "comm/cleanpack_struct.h"

using namespace Carrier;
using namespace std::chrono;

Sensor* Sensor::instance = NULL;

uint64_t Carrier::NowTime() {
  static long start_time = 0;
  uint64_t time_;
  if(start_time == 0){
    start_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    time_ = start_time;
  }else {
    time_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  }
  time_ = time_ - start_time;

  return time_;
}

Sensor::Sensor()
{

  // paramer init
  pickupStatus = 0;
  feedbackStatus = 0;
  this->mChargerIr = 0;
  this->collider_state = 0;
  this->alongWall = 0.0f;
  this->infrareder = 0;
  this->edge_sensor = 0;
  cmdVel_data.v = 0.0;
  cmdVel_data.w = 0.0;
  cmdVel_data.isAcc = false;

  mChargeStatus = 0;
  robot_angle = 0;
  this->deltaX = 0.0f;
  this->deltaW = 0.0f;

  isStop = true;

  tick = 0;
  mControlInterface = new ZmqInterface(ZMQ_PP_TYPE::SEND, SID_CONTROL);

  isStop = false;
}

Sensor::~Sensor() {
  isStop = true;
}


void Sensor::setOdom(const ODOM &msg) {

  float dx = msg.x - robot_pose.x;
  float dy = msg.y - robot_pose.y;
  this->deltaX += sqrt(dx*dx+dy*dy);

  this->encoderL = robot_pose.encoder_left - robot_start_pose.encoder_left;
  this->encoderR = robot_pose.encoder_right - robot_start_pose.encoder_right;

  memcpy(&robot_pose, &msg, sizeof(ODOM));
}

void Sensor::setImu(const IMU &msg) {
  static long last_tick_ms = 0;
  long cur_tick_ms = Carrier::NowTime();

  float diff_angle = msg.yaw - robot_imu.yaw;
  deltaW += fabs(diff_angle);

  if(last_tick_ms == 0) last_tick_ms = cur_tick_ms;
  float dt = cur_tick_ms/1000000.0f - last_tick_ms/1000000.0f;
  robot_angle += (int)(msg.gyro_z*100.0f)/100.0f *fabs(dt);

  memcpy(&robot_imu, &msg, sizeof(IMU));
  last_tick_ms = cur_tick_ms;
  printf("angle : %.3f %.3f\n", robot_angle, (int)(msg.gyro_z*100.0f)/100.0f);
}

void Sensor::setChargeLeft(const char &msg) {
  if(msg){
    mChargeStatus |= 0x01 << 1;
  }else{
    mChargeStatus &= ~(0x01 << 1);
  }
}

void Sensor::setChargeRight(const char &msg) {
  if(msg){
    mChargeStatus |= 0x01 << 0;
  }else{
    mChargeStatus &= ~(0x01 << 0);
  }
}

void Sensor::setCollider(const float &msg) {
  if(fabs(msg) >= DEG2RAD(88)){
    this->collider_state = 0;
  }else if(msg > DEG2RAD(15)){
    this->collider_state = 1;
  }else if(msg < DEG2RAD(-15)){
    this->collider_state = 2;
  }else{
    this->collider_state = 3;
  }
}

void Sensor::setLidarCollider(const char &msg) {
  if( msg == 0 ){
    this->infrareder &= 0x7FFF;
  }else{
    this->infrareder |= 0x8000;
  }
}

void Sensor::setAlongWall(const float &msg) {
    this->alongWall = msg;
}

void Sensor::setEdgeLeft(const bool &msg) {
  if(!msg){
    this->edge_sensor |= 0x01<<1;
  }else{
    this->edge_sensor &= ~(0x01<<1);
  }
}

void Sensor::setEdgeMid(const bool &msg) {
  if(!msg){
    this->edge_sensor |= 0x01<<0;
  }else{
    this->edge_sensor &= ~(0x01<<0);
  }
}

void Sensor::setEdgeRight(const bool &msg) {
  if(!msg){
    this->edge_sensor |= 0x01<<2;
  }else{
    this->edge_sensor &= ~(0x01<<2);
  }
}

void Sensor::setChargerIr(const unsigned short &msg){
  this->mChargerIr = (u32)msg;
}


void Sensor::setChargerInfrarederMidLeft(const unsigned short &msg){
  this->mChargerIrMidLeft = (u32)msg;
}
void Sensor::setChargerInfrarederMidRight(const unsigned short &msg){
  this->mChargerIrMidRight = (u32)msg;
}
void Sensor::setChargerInfrarederLeft(const unsigned short &msg){
  this->mChargerIrLeft = (u32)msg;
}
void Sensor::setChargerInfrarederRight(const unsigned short &msg){
  this->mChargerIrRight = (u32)msg;
}

void Sensor::setInfrareders(const float *msg) {
  int i = 0;
  for (; i < 5; ++i) {
      if(msg[4-i] > INFRAREDER_TRIG_DISTANCE)
      {
        this->infrareder &= ~(0x0001<<i);
      }
      else
      {
        this->infrareder |= 0x0001<<i;
      }
  }
  for (; i < 9; ++i) {
    if(msg[8-i+5] > INFRAREDER_TRIG_DISTANCE)
    {
      this->infrareder &= ~(0x0001<<i);
    }
    else
    {
      this->infrareder |= 0x0001<<i;
    }
  }
}


#define INSET_FEEDBACK(sta,bit) (sta==0 ? ~bit : bit)
void Sensor::setRosControl(const char *msg){

  /*
  int size = msg.data.size();
  for(int i = 0; i< size; i++){
    switch (i){
      case 0:
        feedbackStatus &= ~STATUS_KEY1;// clear bit
        if(msg.data[i] != 0) feedbackStatus |= STATUS_KEY1;
        break;
      case 1:
        feedbackStatus &= ~STATUS_KEY2;// clear bit
        if(msg.data[i] != 0) feedbackStatus |= STATUS_KEY2;
        break;
      case 2:
        if(msg.data[i] == 0) pickupStatus = 0;
            else pickupStatus = 1;
        break;
      case 3:
        if(msg.data[i] != 0) TestTurn(0.0f, 1.5f, 180.0f);
        break;
      default:
        break;
    }
  }
*/
}

void Sensor::PubControl(float w, float v, bool isAcc) {
    cmdVel_data.v = v;
    cmdVel_data.w = w;
    cmdVel_data.isAcc = isAcc;
    uint8_t buffer[80] = {0};
    CP_HEADER *header = (CP_HEADER *)buffer;
    CP_CMDVEL *cmd = (CP_CMDVEL *)(buffer+sizeof(CP_HEADER));
    header->header = HEADER_XX;
    header->len = sizeof(CP_CMDVEL);
    header->type = CP_TYPE::TYPE_CMD_VEL;
    cmd->isAcc = isAcc;
    cmd->v = v*1000;
    cmd->w = w*1000;
    mControlInterface->send(buffer, sizeof(CP_HEADER)+sizeof(CP_CMDVEL));
}

void Sensor::updateMcuSlowPacket(McuSlowPackage &package) {
  //package.reg.ultrasound = 0;
  package.reg.collisionSensor = this->collider_state;
  if(pickupStatus){
    package.reg.dropSensor = 0x03;
  }else{
    package.reg.dropSensor = this->edge_sensor;
  }
  
  
  package.reg.irSensor = this->infrareder;
  package.reg.batteryVoltage = 12000;
  package.reg.systickMs = tick;
  package.reg.errorState = 0;
  package.reg.wallDistance = (u16)(this->alongWall*1000);
  package.reg.waterPumpCurrent = 0;
  package.reg.wheelCurrentL = 0;
  package.reg.wheelCurrentR = 0;
  package.reg.sideBroomCurrentL = 0;
  package.reg.sideBroomCurrentR = 0;
  package.reg.midBroomCurrent = 0;
  package.reg.temperature = 26;
  package.reg.fanSpeed = 0;

  package.reg.realBatteryLevel = 90;
  package.reg.batteryStatus = 90;
  package.reg.motorControlStatus = 0;
  if(mChargeStatus){
    package.reg.chargeStatus = 1;
  }else{
    package.reg.chargeStatus = 0;
  }
  package.reg.motionStatus = 0;
  package.reg.pickupStatus = pickupStatus;
  package.reg.garbageFreqLevel = 0;
  //package.reg.isAlongingWall = mCarrierMove->is_along_wall;
  package.reg.alongWallStatus = 0;

  package.reg.fanValue = 0;
  package.reg.midBroomValue = 0;
  package.reg.sideBroomValue = 0;
  
  feedbackStatus |= STATUS_GARBAGE_BOX_PLUGIN;
  /*
  if(mCarrierMove->is_steping){
    feedbackStatus |= STATUS_IS_STEPING;
  }else{
    feedbackStatus &= ~STATUS_IS_STEPING;
  }
  */
  package.reg.feedbackStatus = feedbackStatus;
  package.reg.lPwm = 0;
  package.reg.lSpeed = 0;
  package.reg.rPwm = 0;
  package.reg.rSpeed = 0;

  package.reg.sensorSwitchStatus = 0;
  package.reg.lastBeepInfo = 0;
  package.reg.curLedInfo = 0;

  package.reg.debugInfo1 = this->mChargerIr;
  package.reg.debugInfo2 = 0;
  
}

void Sensor::updateMcuFastPacket(McuFastPackage &package) {
  package.reg.systickMs = tick;
  package.reg.reserve = -2;
  package.reg.skidError = 0;
  package.reg.rightEncoderPos = this->encoderR;
  package.reg.leftEncoderPos = this->encoderL;
  package.reg.angularPos = robot_angle*1000;
  package.reg.angularVelocity = this->robot_pose.v*1000.0f;
  package.reg.lineVelocity = this->robot_pose.w*1000.0f;

  package.reg.imuData.accelPitch = robot_imu.acc_x;//*1000;
  package.reg.imuData.accelRoll = robot_imu.acc_y;//*1000;
  package.reg.imuData.accelYaw = robot_imu.acc_z;//*1000;

  package.reg.imuData.gyroPitch = robot_imu.gyro_x;//*1000;
  package.reg.imuData.gyroRoll = robot_imu.gyro_y;//*1000;
  package.reg.imuData.gyroYaw = robot_imu.gyro_z;//*1000;

}

void Sensor::ClearDeltaOdom(){
  deltaX = 0.0f;
  deltaW = 0.0f;
}

bool Sensor::CheckPickUp(){
  if(0!=pickupStatus){
    return true;
  }
  return false;
}

void Sensor::TestTurn(float v, float w, float angle){
  
  uint8_t buffer[256] = {0};
  McuProtocolHeader *head = (McuProtocolHeader *)buffer;
  ChassisControlRegister *reg = (ChassisControlRegister *)(buffer + sizeof(McuProtocolHeader));
  
  
  head->deviceAddr = 18;
  head->functionCode = 2;
  head->len = 23;
  head->offset = 0;
  
  reg->lineVelocity = v*1000;
  reg->angularVelocity = w*1000;
  reg->mode = 5;
  reg->vAcc = 0;
  reg->wAcc = 0;
  reg->stepS = 150000;
  reg->stepPhi = M_PI/180.0f*angle*1000;
  //this->mCarrierMove->ProcessMcuPackage(buffer,sizeof(McuProtocolHeader)+head->len);
  reg->mode = 3;
  //this->mCarrierMove->ProcessMcuPackage(buffer,sizeof(McuProtocolHeader)+head->len);
}

void Sensor::Run() {
  while(1){
      tick = Carrier::NowTime()/1000;
      //printf("tick : %d\n",tick);
      usleep(1000);
  }

  isStop = true;
}










