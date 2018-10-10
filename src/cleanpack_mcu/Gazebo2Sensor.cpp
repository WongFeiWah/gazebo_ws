#include <Gazebo2Sensor.h>
#include <CarrierSensor.h>

Gazebo2Sensor::Gazebo2Sensor(void *sensor_handle){
    this->keyPower = false;
    this->keyHome = false;
    this->isPickUp = false;
    mSensorInterface = new ZmqInterface(ZMQ_PP_TYPE::RECV, SID_SENSOR);
    mSensorInterface->setCallBack(&Gazebo2Sensor::callback, this, sensor_handle);
    mSensorInterface->Start();
}

Gazebo2Sensor::~Gazebo2Sensor(){
    delete mSensorInterface;
}

void Gazebo2Sensor::callback(void *param, const uint8_t *data, uint32_t len){
    gzmsg << "Gazebo to Sensor callback\n";
}