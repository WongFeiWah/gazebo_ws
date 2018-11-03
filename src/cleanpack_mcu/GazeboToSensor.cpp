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
    Carrier::Sensor *sensor=(Carrier::Sensor *)param;
    switch(header->type){
        case CP_TYPE::TYPE_ODOM:{
            CP_ODOM *PData = (CP_ODOM *)data;
            ODOM odom;
            memcpy(&odom, &PData->odom, sizeof(ODOM));
            //printf("position %f %f\n",PData->odom.x, PData->odom.y);
            sensor->setOdom(odom);
            break;
        }
        case CP_TYPE::TYPE_IMU:{
            CP_IMU *PData = (CP_IMU *)data;
            IMU imu;
            memcpy(&imu, &PData->imu, sizeof(IMU));
            //printf("imu %.3f %.3f %.3f %.3f\n",PData->imu.x, PData->imu.y, PData->imu.z, PData->imu.w);
            sensor->setImu(imu);
            break;
        }
        case CP_TYPE::TYPE_COLLIDER:{
            CP_COLLIDER *PData = (CP_COLLIDER *)data;
            float angle = PData->collider_angle/1000.0;
            //printf("collider %d\n",PData->collider_angle);
            sensor->setCollider(angle);
            break;
        }
        case CP_TYPE::TYPE_ALONGWALL:{
            CP_ALONGWALL *PData = (CP_ALONGWALL *)data;
            float distance = PData->distance;
            //printf("collider %d\n",PData->collider_angle);
            sensor->setAlongWall(distance);
            break;
        }
        default:break;
    }

    
}