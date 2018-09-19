/*
  * Copyright 2012 Open Source Robotics Foundation
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

#ifndef GAZEBO_ROS_LIDAR_HH
#define GAZEBO_ROS_LIDAR_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>

namespace gazebo
{
  class CleanpackLidar : public RayPlugin
  {
  public: CleanpackLidar();

  public: ~CleanpackLidar();

  public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    // Pointer to the model
  private: std::string world_name_;
  private: physics::WorldPtr world_;
  private: sensors::RaySensorPtr parent_ray_sensor_;

    // deferred load in case ros is blocking
  private: sdf::ElementPtr sdf;

  private: gazebo::transport::NodePtr gazebo_node_;
  private: gazebo::transport::SubscriberPtr laser_scan_sub_;
  private: void OnScan(ConstLaserScanStampedPtr &_msg);

  };
}
#endif