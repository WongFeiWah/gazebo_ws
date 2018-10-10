//
// Created by huang on 18-9-18.
//

#include <CarrierCore.h>
#include <CarrierSensor.h>
#include <Gazebo2Sensor.h>

using namespace Carrier;

int main(int argc, char *argv[])
{
    Carrier::Sensor* sensor = Carrier::Sensor::CreateSensor();
    Carrier::CarrierCore core(sensor, "127.0.0.1");
    Carrier::Gazebo2Sensor gazebo_to_sensor(sensor);
    sensor->Run();
    return 0;
}
