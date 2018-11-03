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

#include <cleanpack_imu.hh>
// c/c++ lib
// gazebo lib
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
// exten lib
// other


namespace gazebo
{


////////////////////////////////////////////////////////////////////////////////
// Constructor
CleanpackImu::CleanpackImu()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
CleanpackImu::~CleanpackImu()
{
  updateTimer.Disconnect(updateConnection);

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void CleanpackImu::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  world = _model->GetWorld();

  if (_sdf->HasElement("LinkName"))
  {
    link_name_ = _sdf->GetElement("LinkName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }
  else
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }

  // assert that the body by link_name_ exists
  if (!link)
  {
    gzerr << "CleanpackImu plugin error: LinkName: " << link_name_.c_str() << " does not exist\n";
    return;
  }

  accelModel.Load(_sdf, "accel");
  rateModel.Load(_sdf, "rate");
  yawModel.Load(_sdf, "yaw");

  // also use old configuration variables from gazebo_ros_imu
  if (_sdf->HasElement("gaussianNoise")) {
    double gaussianNoise;
    if (_sdf->GetElement("gaussianNoise")->GetValue()->Get(gaussianNoise) && gaussianNoise != 0.0) {
      accelModel.gaussian_noise = gaussianNoise;
      rateModel.gaussian_noise  = gaussianNoise;
    }
  }

  if (_sdf->HasElement("xyzOffset")) {
#if (GAZEBO_MAJOR_VERSION >= 8)
    this->offset_.Pos() = _sdf->Get<ignition::math::Vector3d>("xyzOffset");
#else
    this->offset_.pos = _sdf->Get<math::Vector3>("xyzOffset");
#endif
  } else {
#if (GAZEBO_MAJOR_VERSION >= 8)
    this->offset_.Pos() = ignition::math::Vector3d(0, 0, 0);
#else
    this->offset_.pos = math::Vector3(0, 0, 0);
#endif
  }

  if (_sdf->HasElement("rpyOffset")) {
#if (GAZEBO_MAJOR_VERSION >= 8)
    this->offset_.Rot() = _sdf->Get<ignition::math::Quaterniond>("rpyOffset");
#else
    this->offset_.rot = _sdf->Get<math::Vector3>("rpyOffset");
#endif
  } else {
#if (GAZEBO_MAJOR_VERSION >= 8)
    this->offset_.Rot() = ignition::math::Quaterniond(0, 0, 0);
#else
    this->offset_.rot = math::Vector3(0, 0, 0);
#endif
  }

  mSensorInterface = new ZmqInterface(ZMQ_PP_TYPE::SEND, SID_SENSOR);

  Reset();

  // connect Update function
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&CleanpackImu::Update, this));
}

void CleanpackImu::Reset()
{
  updateTimer.Reset();

#if (GAZEBO_MAJOR_VERSION >= 8)
  orientation = ignition::math::Quaterniond();
#else
  orientation = math::Quaternion();
#endif
  velocity = 0.0;
  accel = 0.0;

  accelModel.reset();
  rateModel.reset();
  yawModel.reset();
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void CleanpackImu::Update()
{
  // Get Time Difference dt
#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time cur_time = world->SimTime();
#else
  common::Time cur_time = world->GetSimTime();
#endif
  double dt = updateTimer.getTimeSinceLastUpdate().Double();
  boost::mutex::scoped_lock scoped_lock(lock);

  // Get Pose/Orientation
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d pose = link->WorldPose();
  // ignition::math::Vector3d pos = pose.pos + this->offset_.pos;
  ignition::math::Quaterniond rot = this->offset_.Rot() * pose.Rot();
#else
  math::Pose pose = link->GetWorldPose();
  // math::Vector3 pos = pose.pos + this->offset_.pos;
  math::Quaternion rot = this->offset_.rot * pose.rot;
#endif

  // get Gravity
#if (GAZEBO_MAJOR_VERSION >= 8)
  gravity = world->Gravity();
  double gravity_length = gravity.Length();
  //ROS_DEBUG_NAMED("gazebo_ros_imu", "gravity_world = [%g %g %g]", gravity.X(), gravity.Y(), gravity.Z());
#else
  gravity = world->GetPhysicsEngine()->GetGravity();
  double gravity_length = gravity.GetLength();
  //ROS_DEBUG_NAMED("gazebo_ros_imu", "gravity_world = [%g %g %g]", gravity.x, gravity.y, gravity.z);
#endif

  // get Acceleration and Angular Rates
  // the result of GetRelativeLinearAccel() seems to be unreliable (sum of forces added during the current simulation step)?
  //accel = myBody->GetRelativeLinearAccel(); // get acceleration in body frame
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d temp = link->WorldLinearVel(); // get velocity in world frame
#else
  math::Vector3 temp = link->GetWorldLinearVel(); // get velocity in world frame
#endif
  if (dt > 0.0) accel = rot.RotateVectorReverse((temp - velocity) / dt - gravity);
  velocity = temp;

  // calculate angular velocity from delta quaternion
  // note: link->GetRelativeAngularVel() sometimes return nan?
  // rate  = link->GetRelativeAngularVel(); // get angular rate in body frame
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Quaterniond delta = this->orientation.Inverse() * rot;
#else
  math::Quaternion delta = this->orientation.GetInverse() * rot;
#endif
  this->orientation = rot;
  if (dt > 0.0) {
#if (GAZEBO_MAJOR_VERSION >= 8)
    rate = this->offset_.Rot().Inverse()
           * (2.0 * acos(std::max(std::min(delta.W(), 1.0), -1.0)) * ignition::math::Vector3d(delta.X(), delta.Y(), delta.Z()).Normalize() / dt);
#else
    rate = this->offset_.rot.GetInverse()
           * (2.0 * acos(std::max(std::min(delta.w, 1.0), -1.0)) * math::Vector3(delta.x, delta.y, delta.z).Normalize() / dt);
#endif
  }

  // update sensor models
  accel = accelModel(accel, dt);
  rate  = rateModel(rate, dt);
  yawModel.update(dt);

  //printf("\e[1;30m" "q:\nx %.4f\ny %.4f\nz %.4f\nw %.4f\n", rot.x, rot.y, rot.z, rot.w);
  //printf("\e[1;31m" "acc:\nx %.4f\ny %.4f\nz %.4f\n", accel.x, accel.y, accel.z);
  //printf("\e[1;32m" "rate:\nx %.4f\ny %.4f\nz %.4f\n", rate.x, rate.y, rate.z);
  //printf("\e[0m");
  
  imu_data.imu.acc_x = accel.x;
  imu_data.imu.acc_y = accel.y;
  imu_data.imu.acc_z = accel.z;

  imu_data.imu.gyro_x = rate.x;
  imu_data.imu.gyro_y = rate.y;
  imu_data.imu.gyro_z = rate.z;

  imu_data.imu.x = rot.x;
  imu_data.imu.y = rot.y;
  imu_data.imu.z = rot.z;
  imu_data.imu.w = rot.w;

  imu_data.imu.yaw = rot.GetYaw();
  
  mSensorInterface->send(&imu_data, sizeof(CP_IMU)); 
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(CleanpackImu)

} // namespace gazebo