//
// Created by huang on 18-9-18.
//

#include <CarrierCore.h>
#include <CarrierSensor.h>
#include <GazeboToSensor.h>

using namespace Carrier;

int main(int argc, char *argv[])
{
    Carrier::Sensor* sensor = new Carrier::Sensor();
    Carrier::CarrierCore core(sensor, "127.0.0.1");
    Carrier::GazeboToSensor gazebo_to_sensor(sensor);
    sensor->Run();
    return 0;
}
