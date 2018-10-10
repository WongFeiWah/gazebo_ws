//
// Created by huang on 18-9-18.
//

#ifndef PROJECT_CLEANPACCK_STRUCT_H
#define PROJECT_CLEANPACCK_STRUCT_H

#define HENDER_XX 0xAA


enum CP_TYPE{
    NOP = 0,
    TYPE_CMD_VEL,
    TYPE_ODOM,
    TYPE_IMU,
    TYPE_COLLIDER,



    TYPE_END
};

#pragma pack(push, 1)

typedef struct
{
    uint8_t hender;//0xAA
    uint8_t len;
    uint8_t type;
}CP_HENDER;

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

#pragma pack(pop)




#endif //PROJECT_CLEANPACCK_STRUCT_H
