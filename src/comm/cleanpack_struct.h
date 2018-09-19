//
// Created by huang on 18-9-18.
//

#ifndef PROJECT_CLEANPACCK_STRUCT_H
#define PROJECT_CLEANPACCK_STRUCT_H

#define HENDER_XX 0xAA


enum CP_TYPE{
    NOP = 0,
    TYPE_CMD_VEL,
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
   uint16_t x;
   uint16_t z;
}CP_CMDVEL;

#pragma pack(pop)




#endif //PROJECT_CLEANPACCK_STRUCT_H
