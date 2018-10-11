#include <GazeboToSensor.h>
#include <CarrierSensor.h>
#include <cleanpack_struct.h>

using namespace Carrier;

GazeboToSensor::GazeboToSensor(void *sensor_handle){
    mSensorInterface = new ZmqInterface(ZMQ_PP_TYPE::RECV, SID_SENSOR);
    mSensorInterface->setCallBack(&GazeboToSensor::callback, this, sensor_handle);
    mSensorInterface->Start();
}

GazeboToSensor::~GazeboToSensor(){
    delete mSensorInterface;
}

void GazeboToSensor::callback(void *param, const uint8_t *data, uint32_t len){
    CP_HEADER *header = (CP_HEADER *)data;
    switch(header->type){
        case CP_TYPE::TYPE_COLLIDER:{
            CP_COLLIDER *PData = (CP_COLLIDER *)data;
            printf("collider %d\n",PData->collider_angle);
            break;
        }
        default:break;
    }

    
}