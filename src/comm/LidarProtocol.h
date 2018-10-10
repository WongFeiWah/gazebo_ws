/**
 * @file        pro.h
 * @author      陈维
 * @version     V01
 * @date        2016.09.21
 * @brief       协议定义
 * @note
 *
 * @attention   COYPRIGHT INMOTION ROBOT
 **/

#ifndef _PRO_H_
#define _PRO_H_

#define ANGLE_PER_PACK 12
#define LIDAR_RESOLUTION 360

#pragma pack(push, 1)

typedef struct {
  uint16_t distance;
  uint8_t confidence;
}Lidar2DPoint;

typedef struct {
  uint8_t newRawData;
  uint8_t newPackage;
  //uint16_t temperature;   // temperature
  int16_t curSpeed;       // current speed
  int16_t setSpeed;       // set speed
}LidarStatus;

typedef struct {
  LidarStatus status;
  uint64_t sysTimeStamp;   // timestamp from system
  uint64_t lidarTimestamp; // timestamp from lidar
  Lidar2DPoint points[LIDAR_RESOLUTION];
}LidarDataInfo;

#pragma pack()

#endif



/************************ (C) COPYRIGHT INMOTION ROBOT *****END OF FILE****/
