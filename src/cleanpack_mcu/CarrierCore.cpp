//
// Created by huang on 8/6/18.
//

#include <CarrierCore.h>
#include <CarrierSensor.h>

using namespace Carrier;

CarrierCore::CarrierCore(void *sensor_handler, char *bridgeName)
{
  mMcuPackageProtocol = new McuPackageProtocol(4096);

  this->sensor_handler = sensor_handler;
  this->mModeStatus = CarrierStatus::CAR_IDLE_MODE;

  if(bridgeName != NULL){
    memcpy(mBridgePortName, bridgeName, strlen(bridgeName));
  } else {
    memcpy(mBridgePortName, "127.0.0.1", strlen("127.0.0.1"));
  }

  this->mBridge = (CommunicationPort*)(new ZmqPort(bridgeName,SID_ROS_SIMULATION_MCU_TO_CARRIER, SID_ROS_SIMULATION_CARRIER_TO_MCU));
  ((ZmqPort*)mBridge)->setCallBack(&Carrier::CarrierCore::OnBridgeSubscriber, this);
  ((ZmqPort*)mBridge)->Start();

  memset(&mCtrlPackage, 0, sizeof(ChassisControlRegister));
  this->last_collier = 0;

  mMoveModeThread = new std::thread(CarrierCore::OnMoveMode, this);
  mDeltaMoveThread = new std::thread(CarrierCore::OnDeltaMove, this);
  mBridgePublisherThread = new std::thread(CarrierCore::OnBridgePublisher, this);
}

CarrierCore::~CarrierCore() {
  if(mMoveModeThread)
    delete mMoveModeThread;

  if(mDeltaMoveThread)
    delete mDeltaMoveThread;

  if(mBridgePublisherThread)
    delete mBridgePublisherThread;

  ((ZmqPort*)mBridge)->Stop();
  this->sensor_handler = NULL;
}

void CarrierCore::StopMove() {
  Carrier::Sensor *sensor_ptr = (Carrier::Sensor *)sensor_handler;
  this->setMove(0.0f,0.0f);
  sensor_ptr->PubControl(this->move_w, this->move_v);
}

void CarrierCore::Move() {
  Carrier::Sensor *sensor_ptr = (Carrier::Sensor *)sensor_handler;
  sensor_ptr->PubControl(this->move_w, this->move_v);
}

void CarrierCore::Move(float w, float v, bool isAcc) {
  Carrier::Sensor *sensor_ptr = (Carrier::Sensor *)sensor_handler;
  this->setMove(w,v);
  if(abs(v) > CAR_MAX_V){
    v = (v>0.0f ? CAR_MAX_V:-CAR_MAX_V);
  }
  if(abs(w) > CAR_MAX_W){
    w = (w>0.0f ? CAR_MAX_W:-CAR_MAX_W);
  }
  //ROS_INFO_STREAM("move : " << w << " " << v );
  sensor_ptr->PubControl(this->move_w, this->move_v, isAcc);
}

void CarrierCore::SetCtlData(ChassisControlRegister mCtrlPackage) {

  this->move_set_v = mCtrlPackage.lineVelocity/1000.0f;
  this->move_set_w = mCtrlPackage.angularVelocity/1000.0f;

  //printf("mode %d  co:%d\n",mCtrlPackage.mode,this->last_collier);
  //ROS_INFO_STREAM(" set speed : " << this->move_set_v << "   " << this->move_set_w);
  switch (mCtrlPackage.mode){
    case 0:
    {
      this->is_steping = false;
      this->mModeStatus = CarrierStatus::CAR_SPEED_MODE;
      this->mLastMoveControlTime = Carrier::NowTime();
      //ROS_INFO_STREAM("Speed Mode! ");
    }break;
    case 1:
    {
      this->is_steping = false;
      this->mModeStatus = CarrierStatus::CAR_WALLFOLLOW_MODE;
      this->mLastMoveControlTime = Carrier::NowTime();
    }break;
    case 2:
    {
      this->is_steping = false;
      this->mModeStatus = CarrierStatus::CAR_CHARGE_MODE;
      this->mLastMoveControlTime = Carrier::NowTime();
    }break;
    case 3:
    {
      this->mLastMoveControlTime = Carrier::NowTime();
      if( (this->mModeStatus != CarrierStatus::CAR_STEP_MODE) && (this->mModeStatus != CarrierStatus::CAR_STEPEND_MODE) )
      {
        this->mStep_s = mCtrlPackage.stepS;
        this->mStep_phi = mCtrlPackage.stepPhi;
        this->ClearDeltaMove();
        this->is_steping = true;
        this->mModeStatus = CarrierStatus::CAR_STEP_MODE;
        //ROS_DEBUG_STREAM("dst "<< this->mStep_s << " " << this->mStep_phi);
        //ROS_DEBUG_STREAM("speed "<< this->move_set_v << " " << this->move_set_w);
        //ROS_DEBUG_STREAM("start "<< this->mDeltaX << " " << this->mDeltaW);
        //ROS_INFO_STREAM("Step Mode! " << this->mStep_s << " " << this->mStep_phi);
      }
    }break;
    case 4:
    {
      this->is_steping = false;
      this->mModeStatus = CarrierStatus::CAR_DIRECT_MODE;
      this->mLastMoveControlTime = Carrier::NowTime();
    }break;
    case 5:
    {
      this->is_steping = false;
      this->mModeStatus = CarrierStatus::CAR_IDLE_MODE;
      this->mLastMoveControlTime = Carrier::NowTime();
    }break;
    default:
      break;
  }
}

void CarrierCore::OnMoveMode(void *param) {
  CarrierCore *this_ = (CarrierCore*)param;
  Carrier::Sensor *sensor = (Carrier::Sensor *)this_->sensor_handler;
  CarrierStatus last_MoveStatus = this_->mModeStatus;

  while(!sensor->isStop){
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    //********************************************
    //** step 0: Idle handle Code Block
    
    if( MOVE_LAST_TIME < (Carrier::NowTime() - this_->mLastMoveControlTime) ){
      if( !this_->is_steping ){
        this_->move_set_w = 0.0f;
        this_->move_set_v = 0.0f;
        this_->StopMove();
      }
    }
    
    //********************************************
    //** step 0: Drop handle Code Block
    
    if(sensor->CheckPickUp()) {
      this_->StopMove();
      continue;
    }
  
    if(sensor->edge_sensor != 0){
        // back move 0.1m
        float turn_angle = 0.0f;
        float turn_speed = 0.0f;
        int count = 0;
        u8 last_status = sensor->edge_sensor;
        this_->ClearDeltaMove();
        this_->Move(0.0f, -0.2f, false);
        while(count < 5000 && (this_->mDeltaX < 100 || sensor->edge_sensor != 0)){
            usleep(200);
            count++;

            this_->Move(0.0f, -0.2f, false);
        }
  
        this_->ClearDeltaMove();
        if(last_status & (0x01<<0)) {
            turn_speed = 1.0f;
            turn_angle = 90.0f;
        }else if(last_status & (0x01<<1)) {
            turn_speed = 1.0f;
            turn_angle = 120.0f;
        }else if(last_status & (0x01<<2)) {
            turn_speed = 1.0f;
            turn_angle = 30.0f;
        }
        
        this_->Move(turn_speed, 0.0f);
        while(this_->mDeltaW < (M_PI/180.0f*turn_angle)*1000){
            usleep(1000);
            this_->Move(turn_speed, 0.0f);
        }
        this_->StopMove();
        this_->mModeStatus = CarrierStatus::CAR_IDLE_MODE;
    }
    //********************************************
    //** step 1: collide handle Code Block
    if(sensor->collider_state != 0){
      // back move 0.1m
      this_->ClearDeltaMove();
      this_->mWallFollowStatus = WAL_ALIGN;
      this_->last_collier = sensor->collider_state;
      this_->Move(0.0f, -0.2f, false);
      while(this_->mDeltaX < 15){
        usleep(200);
        this_->Move(0.0f, -0.2f, false);
      }
      if(sensor->collider_state != 0) {
        // turn around 180C
        this_->ClearDeltaMove();
        this_->Move(1.5f, 0.0f);
        while(this_->mDeltaW < M_PI*1000){
          usleep(1000);
          if(sensor->collider_state == 0){
            break;
          }else{
            this_->Move(1.5f, 0.0f);
          }
        }
      }
      this_->move_set_w = 0.0f;
      this_->move_set_v = 0.0f;
      this_->StopMove();
      this_->mModeStatus = CarrierStatus::CAR_IDLE_MODE;
    }
    //********************************************
    //** step 2: update last MoveStatus
    if(this_->mModeStatus != last_MoveStatus){
      last_MoveStatus = this_->mModeStatus;
      //ROS_INFO_STREAM("move status: " << last_MoveStatus);
    }

    if(this_->mModeStatus != CarrierStatus::CAR_STEP_MODE){
      this_->is_steping = false;
    }

    //********************************************
    //** step 3: car move status handle Code Block
    switch(this_->mModeStatus){
      case CarrierStatus::CAR_WALLFOLLOW_MODE:
      {
        this_->AlongWallLoop();
      }break;
      case CarrierStatus::CAR_SPEED_MODE :
      {
        this_->Move(this_->move_set_w, this_->move_set_v);
        this_->last_collier = 0;
      }break;
      case CarrierStatus::CAR_STEP_MODE :
      {
        static float factory_w = 25.0f;
        static float factory_v = 0.0f;
        this_->Move(this_->move_set_w, this_->move_set_v);
        this_->is_steping = true;
        if (    abs(this_->mDeltaX) + (factory_v*abs(this_->move_v)) > abs(this_->mStep_s)
             || abs(this_->mDeltaW) + (factory_w*abs(this_->move_w)) > abs(this_->mStep_phi) ) {
          this_->is_steping = false;
          this_->mModeStatus = CarrierStatus::CAR_STEPEND_MODE;
          this_->StopMove();
          //ROS_DEBUG_STREAM("end "<< this_->mDeltaX << " " << this_->mDeltaW);
        }
      }break;
      default:
        break;
    }
  }
}

void CarrierCore::AlongWallLoop() {
  Carrier::Sensor *sensor = (Carrier::Sensor *)this->sensor_handler;
  float psdDis = sensor->alongWall;
  static float psdDisLast = 0;
  //ROS_INFO_STREAM("dis: " << psdDis);
  switch (this->mWallFollowStatus){
    //**********
    //   ALIGN
    case AlongWallStatus::WAL_ALIGN :
    {
      this->ClearDeltaMove();
      this->Move(-CAR_WALLFOLLOW_AL_W, 0.0f);

      if(last_collier == 1){
        this->wallfollowAlignDstW = M_PI / 180.0f*120.0f;
      }else if(last_collier == 2){
        this->wallfollowAlignDstW = M_PI / 180.0f*15.0f;
      }else if(last_collier == 3){
        this->wallfollowAlignDstW = M_PI / 180.0f*90.0f;
      }

      this->last_collier = 0;

      //ROS_INFO_STREAM("WAL_ALIGN " << this->wallfollowAlignDstW);
      psdDisLast = psdDis;
      this->mWallFollowStatus = AlongWallStatus::WAL_ALIGN_WAIT;
    }break;
    //**********
    //  ALIGN_WAIT
    case AlongWallStatus::WAL_ALIGN_WAIT :
    {
      //ROS_INFO_STREAM("WAL_ALIGN_WAIT");
      this->Move(-CAR_WALLFOLLOW_AL_W, 0.0f);
      if ( abs(this->mDeltaW/1000.0f) > (this->wallfollowAlignDstW - M_PI / 180.0f*10.0f) ) {
        if ( (psdDis - psdDisLast) > 0 && psdDis < 0.12f && last_collier > 1) {
          this->mWallFollowStatus = AlongWallStatus::WAL_ALONG;
          //ROS_DEBUG_STREAM("1WAL_ALIGN END " << this->mDeltaW);
          this->last_collier = 0;
        }

        if (abs(this->mDeltaW/1000.0f) > this->wallfollowAlignDstW) {
          this->mWallFollowStatus = AlongWallStatus::WAL_ALONG;
          //ROS_DEBUG_STREAM("2WAL_ALIGN END " << this->mDeltaW);
          this->last_collier = 0;
        }

      }
    }break;
    //**********
    //  ALONG
    case AlongWallStatus::WAL_ALONG :
    {
      float tv = 0;
      float tw = 0;
      //ROS_INFO_STREAM("WAL_ALONG");
      if (psdDis > 0.12f || psdDis == 0) {
        tv = CAR_WALLFOLLOW_AL_V;
        tw = CAR_WALLFOLLOW_AL_W;
        this->is_along_wall = false;
        //car->is_along_wall = 0;
        //tv += 1;
      } else {
        float err = psdDis - PSD_TAR_DIS;
        static float lasterr = 0;

        //ROS_INFO_STREAM("err= " << err);

        tv = CAR_WALLFOLLOW_V;
        tw = err * CAR_WALLFOLLOW_PID_P;
        tw += (err - lasterr) * CAR_WALLFOLLOW_PID_I;

        //ROS_INFO_STREAM("tw= " << tw);


        if (tw > CAR_MAX_W) {
          tw = CAR_MAX_W;
        }

        if (tw < -CAR_MAX_W) {
          tw = -CAR_MAX_W;
        }

        lasterr = err;
        this->is_along_wall = true;
        //car->is_along_wall = 1;
      }
      this->Move(tw,tv);
    }break;
    default:
      break;
  }
}

void CarrierCore::OnDeltaMove(void *param) {
  CarrierCore *this_ = (CarrierCore*)param;
  Carrier::Sensor *sensor = (Carrier::Sensor *)this_->sensor_handler;

  while(!sensor->isStop){

    this_->mDeltaX = sensor->deltaX*1000;
    this_->mDeltaW = sensor->deltaW*1000;
    //this_->last_pose = this_->curr_pose;
    usleep(10000);
  }
}

void CarrierCore::ClearDeltaMove() {
  ((Carrier::Sensor*)sensor_handler)->ClearDeltaOdom();
  //this->last_pose = ((Carrier::Sensor*)sensor_handler)->robot_pose;
  this->mDeltaX = 0.0f;
  this->mDeltaW = 0.0f;
}

void CarrierCore::OnBridgePublisher(void *param) {
  CarrierCore *this_ = (CarrierCore*)param;
  Carrier::Sensor *sensor = (Carrier::Sensor *)this_->sensor_handler;
  int timer_count = 0;
  while(!sensor->isStop && this_->mBridge) {
    timer_count++;
    if(timer_count >= 6){
      timer_count = 0;
      sensor->updateMcuSlowPacket(this_->mcuSlowPackage_);
      this_->SendMcuData((uint8_t *) &this_->mcuSlowPackage_, sizeof(McuSlowPackage));
    }
    sensor->updateMcuFastPacket(this_->mcuFastPackage_);
    this_->SendMcuData((uint8_t *) &this_->mcuFastPackage_, sizeof(McuFastPackage));
    std::this_thread::sleep_for(std::chrono::milliseconds(6));
  }
}

void CarrierCore::OnBridgeSubscriber(void *param, const uint8_t *data, uint32_t len) {
  for(uint32_t i = 0; i<len;i++){
    if (this->mMcuPackageProtocol->PushByte(data[i]) > 0) {
      this->ProcessMcuPackage(this->mMcuPackageProtocol->GetRxPackageBuffer(),
                        this->mMcuPackageProtocol->GetPackageSize());
    }
  }
}

int CarrierCore::SendMcuData(uint8_t *data, uint32_t len) {
  int32_t txLen = 0;
  const uint8_t *txBuf = mMcuPackageProtocol->PackageBuffer(data, len, &txLen);
  return this->mBridge->Send(txBuf, txLen);
}

void CarrierCore::ProcessMcuPackage(const uint8_t *data, int32_t len) {
  uint32_t read_len = len;
  McuProtocolHeader *head = (McuProtocolHeader *)data;
  if(read_len < sizeof(McuProtocolHeader)){
    return;
  }
  if (head->deviceAddr == MCU_ADDRESS){
    switch (head->functionCode){
      case MCU_CONTROL_REG:
      {
        if(head->len + head->offset >= sizeof(ChassisControlRegister)) break;
        memcpy((uint8_t *)&mCtrlPackage + head->offset, data + sizeof(McuProtocolHeader), head->len);
        this->SetCtlData(mCtrlPackage);
      }break;
      default:
        break;
    }
  }

}







