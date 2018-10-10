//
// Created by huang on 7/25/18.
//

#ifndef GAZEBO2SENSOR_H
#define GAZEBO2SENSOR_H

#include <string>
#include "comm/ZmqInterface.h"

namespace Carrier{
  class Gazebo2Sensor
  {
    private:
        ZmqInterface *mSensorInterface;
    
    public:
        Gazebo2Sensor(void *sensor_handle);
        ~Gazebo2Sensor();
        void callback(void *param, const uint8_t *data, uint32_t len);
  };

}
#endif 