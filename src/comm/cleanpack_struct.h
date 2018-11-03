//
// Created by huang on 18-9-18.
//

#ifndef PROJECT_CLEANPACK_STRUCT_H
#define PROJECT_CLEANPACK_STRUCT_H

#define HEADER_XX 0xAA


enum CP_TYPE{
    NOP = 0,
    TYPE_CMD_VEL,
    TYPE_ODOM,
    TYPE_IMU,
    TYPE_COLLIDER,
    TYPE_ALONGWALL,
    TYPE_KEY,



    TYPE_END
};

#pragma pack(push, 1)

typedef struct
{
    uint8_t header;//0xAA
    uint8_t len;
    uint8_t type;
}CP_HEADER;

typedef struct
{
   uint8_t isAcc;
   int16_t v;
   int16_t w;
}CP_CMDVEL;

typedef struct
{
    int encoder_left;
    int encoder_right;

    float x;
    float y;
    float z;

    float v;
    float w;
}T_ODOM;

typedef struct{
  float x;
  float y;
  float z;
  float w;

  float yaw;

  float acc_x;
  float acc_y;
  float acc_z;

  float gyro_x;
  float gyro_y;
  float gyro_z;
}T_IMU;

typedef struct _CP_ODOM
{
   CP_HEADER header;
   T_ODOM odom;
   _CP_ODOM(){
       header.header = HEADER_XX;
       header.type = CP_TYPE::TYPE_ODOM;
       header.len = sizeof(struct _CP_ODOM);
   }
}CP_ODOM;

typedef struct _CP_IMU
{
   CP_HEADER header;
   T_IMU imu;
   _CP_IMU(){
       header.header = HEADER_XX;
       header.type = CP_TYPE::TYPE_IMU;
       header.len = sizeof(struct _CP_IMU);
   }
}CP_IMU;

typedef struct _CP_COLLIDER
{
   CP_HEADER header;
   int16_t collider_angle;// mdeg
   _CP_COLLIDER(){
       header.header = HEADER_XX;
       header.type = CP_TYPE::TYPE_COLLIDER;
       header.len = sizeof(struct _CP_COLLIDER);
   }
}CP_COLLIDER;

typedef struct _CP_ALONGWALL
{
   CP_HEADER header;
   int16_t distance;
   _CP_ALONGWALL(){
       header.header = HEADER_XX;
       header.type = CP_TYPE::TYPE_ALONGWALL;
       header.len = sizeof(struct _CP_ALONGWALL);
       distance = 0;
   }
}CP_ALONGWALL;

typedef struct _CP_KEY
{
   CP_HEADER header;
   bool key1;
   bool key2;
   _CP_KEY(){
       header.header = HEADER_XX;
       header.type = CP_TYPE::TYPE_KEY;
       header.len = sizeof(struct _CP_KEY);
       key1 = key2 = false;
   }
}CP_KEY;

#pragma pack(pop)




#endif //PROJECT_CLEANPACCK_STRUCT_H
