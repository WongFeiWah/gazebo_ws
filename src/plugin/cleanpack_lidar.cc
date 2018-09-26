/*
  * Copyright 2013 Open Source Robotics Foundation
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
 */

/*
 * Desc: Ros Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 */

#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Console.hh>

#include <cleanpack_lidar.hh>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(CleanpackLidar)
  // Constructor
  CleanpackLidar::CleanpackLidar()
  {

  }

  // Destructor
  CleanpackLidar::~CleanpackLidar()
  {
  }

  // Load the controller
  void CleanpackLidar::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Get the world name.
    std::string worldName = _parent->WorldName();
    this->world_ = physics::get_world(worldName);
    // save pointers
    this->sdf = _sdf;
    sensor_ = _parent;
    mLidarFrequency = 6;
    mPort = NULL;

    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);
    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(this->sensor_->GetTopic(),
                                  &CleanpackLidar::OnScan, this);
    // connect Update function

    //mPort = new ZmqPort();
    char *pathvar = getenv("CARRIER_IP");
    if(pathvar != NULL) {
      carrier_ip = pathvar;
    }else{
      carrier_ip = "127.0.0.1";
    }
    printf("carrier lidar IP is : %s\n", pathvar);
    this->sensor_->SetActive(true);
    printf("Lidar Load.\n");

  }


  bool CleanpackLidar::SendLidarData(ConstLaserScanStampedPtr &_msg){
    LidarFrameSeg frame;
    uint8_t *ptr = (uint8_t*)&frame;
    uint8_t crc = 0;
    int sleep_time = mLidarFrequency;
    frame.header = 0x54;
    frame.speed = this->mLidarFrequency*360;
    frame.version_and_length = 12;
    if(_msg->scan().count() < LIDAR_RESOLUTION) {
      return false;
    }

    for (int j = 0; j < LIDAR_RESOLUTION;) {
      frame.start_angle = j*100;
      for (int i = 0; i < ANGLE_PER_PACK; ++i) {
        frame.points[i].distance = (unsigned short)(_msg->scan().ranges(j)*1000);
        if(_msg->scan().intensities(j) <= 1.0f || _msg->scan().intensities(j) >= 255.0f){
          frame.points[i].confidence = 255;
        }else{
          frame.points[i].confidence = _msg->scan().intensities(j);
        }
        ++j;
      }

      frame.end_angle = (j-1)*100;//( j==LIDAR_RESOLUTION ? 0 : (j-1)*100);
      //frame.timestamp = NowTime();

      crc = 0;
      for (uint64_t i = 0; i < (sizeof(LidarFrameSeg)-1); ++i) {
        crc = crc_table[(crc ^ ptr[i]) & 0xff];
      }
      frame.crc8 = crc;

      if(mPort != NULL && mPort->IsOpened() && sleep_time != 0){
        mPort->Send((const uint8_t *) &frame, sizeof(LidarFrameSeg));
        usleep(27778/sleep_time);

      }else{
        return false;
      }
    }
    return true;
  }

  void CleanpackLidar::OnScan(ConstLaserScanStampedPtr &_msg)
  {
    //gzmsg << "laser masg." << std::endl;
    //printf("laser masg. %d\n", _msg->scan().count());
    //boost::mutex::scoped_lock scoped_lock(lock);
    if(!lock.try_lock()) return;
    SendLidarData(_msg);
    /*mScan.clear();
    mScan.resize(_msg->scan().count());
    std::copy(_msg->scan().ranges().begin()
    ,_msg->scan().ranges().end()
    ,mScan.begin());*/
    lock.unlock();
  }

}