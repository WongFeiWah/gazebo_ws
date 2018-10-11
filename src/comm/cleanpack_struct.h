//
// Created by huang on 18-9-18.
//

#ifndef PROJECT_CLEANPACCK_STRUCT_H
#define PROJECT_CLEANPACCK_STRUCT_H

#define HEADER_XX 0xAA


enum CP_TYPE{
    NOP = 0,
    TYPE_CMD_VEL,
    TYPE_ODOM,
    TYPE_IMU,
    TYPE_COLLIDER,
    TYPE_ALLONGWALL,


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
   int16_t x;
   int16_t y;
   int16_t z;

   int16_t v;
   int16_t w;

   int32_t encoder_left;
   int32_t encoder_right;
}CP_ODOM;

typedef struct _CP_COLLIDER
{
   CP_HEADER header;
   int16_t collider_angle;
   _CP_COLLIDER(){
       header.header = HEADER_XX;
       header.type = CP_TYPE::TYPE_COLLIDER;
       header.len = sizeof(struct _CP_COLLIDER);
   }
}CP_COLLIDER;

typedef struct _CP_ALLONGWALL
{
   CP_HEADER header;
   int16_t distance;
   _CP_ALLONGWALL(){
       header.header = HEADER_XX;
       header.type = CP_TYPE::TYPE_COLLIDER;
       header.len = sizeof(struct _CP_ALLONGWALL);
       distance = 0;
   }
}CP_ALLONGWALL;

#pragma pack(pop)




#endif //PROJECT_CLEANPACCK_STRUCT_H
