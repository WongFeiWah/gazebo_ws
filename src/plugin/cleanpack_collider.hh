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

#ifndef CLEANPACK_GAZEBO_PLUGINS_COLLIDER_H
#define CLEANPACK_GAZEBO_PLUGINS_COLLIDER_H

#include <gazebo/common/Plugin.hh>
#include <comm/ZmqInterface.h>
#include <update_timer.h>

namespace gazebo
{

class CleanpackCollider : public SensorPlugin
{
public:
    CleanpackCollider();
  virtual ~CleanpackCollider();

protected:
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update_collider_body();
  virtual void Update_collider_charge();
  virtual void Update_collider_lidar();
  float calcDistance(float x, float y, float x1, float y1);

private:
  /// \brief The parent World
  physics::WorldPtr world;
  physics::ModelPtr parent_;

  sensors::ContactSensorPtr sensor_;

  std::string namespace_;
  std::string topic_;
  std::string frame_id_;
  std::string mode_;
  std::string contactModel_;
  double collider_radius_;

  ZmqInterface *mSensorInterface;

  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;

};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_SONAR_H
