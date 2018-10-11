//
// Created by huang on 7/25/18.
//

#ifndef GAZEBOTOSENSOR_H
#define GAZEBOTOSENSOR_H

#include <string>
#include "comm/ZmqInterface.h"

namespace Carrier{
  class GazeboToSensor
  {
    private:
        ZmqInterface *mSensorInterface;
        ZmqInterface *mControlInterface;
    
    public:
        GazeboToSensor(void *sensor_handle);
        ~GazeboToSensor();
        void callback(void *param, const uint8_t *data, uint32_t len);
  };

}
#endif 