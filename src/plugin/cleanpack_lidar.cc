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
    printf("Lidar Load .\n");
    // load plugin
    RayPlugin::Load(_parent, this->sdf);
    // Get the world name.
    std::string worldName = _parent->WorldName();
    this->world_ = physics::get_world(worldName);
    // save pointers
    this->sdf = _sdf;

    using std::dynamic_pointer_cast;
    this->parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(_parent);

    if (!this->parent_ray_sensor_)
      gzthrow("CleanpackLidar controller requires a Ray Sensor as its parent");

    this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);
    this->laser_scan_sub_ = this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                  &CleanpackLidar::OnScan, this);
    // connect Update function
   /* updateTimer.setUpdateRate(10.0);
    updateTimer.Load(world, _sdf);
    updateConnection = updateTimer.Connect(boost::bind(&CleanpackLidar::Update, this));*/
    //gzmsg("Lidar Load.");
    this->parent_ray_sensor_->SetActive(false);

  }
  void CleanpackLidar::OnScan(ConstLaserScanStampedPtr &_msg)
  {
    gzmsg << "laser masg." << std::endl;
    //printf("laser masg.\n");
  }
// Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(CleanpackLidar)
}