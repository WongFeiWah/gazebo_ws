//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <cleanpack_alongwall.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <limits>

#include <gazebo/gazebo_config.h>

namespace gazebo {

CleanpackAlongWall::CleanpackAlongWall()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
CleanpackAlongWall::~CleanpackAlongWall()
{
  updateTimer.Disconnect(updateConnection);
  sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void CleanpackAlongWall::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  gzmsg << "CleanpackAlongWall::Load.\n";
  // Get then name of the parent sensor
  sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosSonar requires a Ray Sensor as its parent");
    return;
  }

  // Get the world name.
  std::string worldName = sensor_->WorldName();
  world = physics::get_world(worldName);



  sensor_model_.Load(_sdf);

  Reset();

  // connect Update function
  updateTimer.setUpdateRate(30.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&CleanpackAlongWall::Update, this));

  // activate RaySensor
  sensor_->SetActive(true);
  gzmsg << "CleanpackAlongWall::Load End.\n";
  mSensorInterface = new ZmqInterface(ZMQ_PP_TYPE::SEND, SID_SENSOR);
}

void CleanpackAlongWall::Reset()
{
  updateTimer.Reset();
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void CleanpackAlongWall::Update()
{
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);


  // find ray with minimal range
  int num_ranges = sensor_->LaserShape()->GetSampleCount() * sensor_->LaserShape()->GetVerticalSampleCount();
  float range = sensor_->LaserShape()->GetRange(0);
  //range = sensor_model_(range, dt);
  //gzmsg << "alongwall: " << range << "\n";
  alongwall_data.distance = range;
  mSensorInterface->send(&alongwall_data, alongwall_data.header.len);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CleanpackAlongWall)

} // namespace gazebo
