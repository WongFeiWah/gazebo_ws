//
// Created by huang on 7/25/18.
//

#ifndef PROJECT_SENSORNODE_H
#define PROJECT_SENSORNODE_H

#include <string>
#include <mutex>
#include <zmq.h>
#include <comm/mcu_protocol.h>

#define GETDISTANCE(x1,y1,x2,y2) ( sqrt( ( (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) )) )
#define INFRAREDER_TRIG_DISTANCE (0.03+0.175)
#define DEG2RAD(x) ((x) * M_PI / 180.0f)
#define CHECKERROR(x, L, err) ( ( x > (L+err) || x < (L-err) ) ? false : true  )
#define LIMITANGLE(angle) (angle < 0.0f ? (M_PI*2.0f + angle) : angle)

namespace Carrier{
  typedef struct cmd_vel_struct{
    bool isAcc;
    float v;
    float w;
  }CMD_VEL;

  typedef struct imu_struct{
    float x;
    float y;
    float z;
    float w;

    float acc_x;
    float acc_y;
    float acc_z;

    float gyro_x;
    float gyro_y;
    float gyro_z;
  }IMU;

  typedef struct odom_struct{
    int encoder_left;
    int encoder_right;

    float x;
    float y;
    float z;

    float v;
    float w;
  }ODOM;


  class Sensor
  {
  protected:
  private:
    Sensor();
    
    CMD_VEL cmdVel_data;
    IMU robot_imu;
    ODOM robot_pose;


    std::mutex mtx;           // locks access to scan
    void *zmq_context;
    void *zmq_publisher;

    // ros param
    std::string mBridgePortName;
    uint64_t tick;
    uint32_t feedbackStatus;
    uint8_t pickupStatus;

    static std::mutex create_mtx;
    static Sensor *instance;

  public:
    static Sensor* CreateSensor(){
      if (instance == NULL)
      {
        create_mtx.lock();
        if (instance == NULL)
          instance = new Sensor();
        create_mtx.unlock();
        return instance;
      }
    }

    ~Sensor();

    void setOdom(const ODOM &msg);
    void setImu(const IMU &msg);

    void setChargeLeft(const char &msg);
    void setChargeRight(const char &msg);

    void setLidarCollider(const char &msg);
    void setCollider(const float &msg);
    void setAlongWall(const float &msg);

    void setEdgeLeft(const char &msg);
    void setEdgeMid(const char &msg);
    void setEdgeRight(const char &msg);

    void setChargerIr(const unsigned short &msg);
    void setChargerInfrarederMidLeft(const unsigned short &msg);
    void setChargerInfrarederMidRight(const unsigned short &msg);
    void setChargerInfrarederLeft(const unsigned short &msg);
    void setChargerInfrarederRight(const unsigned short &msg);
    void setInfrareders(const float *msg);
    void Run();

    void setRosControl(const char *msg);
    void PubControl(float w, float v, bool isAcc = 0.0f);
    void updateMcuSlowPacket(McuSlowPackage &package);
    void updateMcuFastPacket(McuFastPackage &package);

    void ClearDeltaOdom();
    bool CheckPickUp();
    void TestTurn(float v, float w, float angle);

    char collider_state;  // 0 no collided ; 1 right collided; 2 left collided; 3 all collided;
    float alongWall;      // 0~0.15m
    uint16_t infrareder;   //  true has tryg; false no trig;
    uint8_t edge_sensor;
    double robot_angle;
    
    bool isStop;
    char mChargeStatus;
    uint32_t mChargerIr;
    uint32_t mChargerIrMidLeft;
    uint32_t mChargerIrMidRight;
    uint32_t mChargerIrLeft;
    uint32_t mChargerIrRight;
    long start_encoderL;
    long start_encoderR;
    long encoderL;
    long encoderR;

    float deltaX;
    float deltaW;
  };

  uint64_t NowTime();
}
#endif //PROJECT_SENSORNODE_H
