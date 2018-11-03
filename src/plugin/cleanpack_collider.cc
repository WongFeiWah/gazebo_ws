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

#include <cleanpack_collider.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/ContactSensor.hh>

#include <limits>
#include <math.h>
#include <string>

#include <gazebo/gazebo_config.h>


#define CHECK_EQUAL(num, e_num, error) (num < (e_num+error) && num > (e_num-error) ? true : false )

namespace gazebo {

CleanpackCollider::CleanpackCollider()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
CleanpackCollider::~CleanpackCollider()
{
  updateTimer.Disconnect(updateConnection);
  sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void CleanpackCollider::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  gzmsg << "CleanpackCollider::Load.\n";
  // Get then name of the parent sensor
  sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosSonar requires a Ray Sensor as its parent");
    return;
  }

  // Get the world name.
  std::string worldName = sensor_->WorldName();
  world = physics::get_world(worldName);

  // default parameters
  topic_ = "collider";
  frame_id_ = "/collider_link";

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("mode"))
    mode_ = _sdf->GetElement("mode")->GetValue()->GetAsString();
  else
    mode_ = "collide";

  if (_sdf->HasElement("contactModel"))
    contactModel_ = _sdf->GetElement("contactModel")->GetValue()->GetAsString();

  if (_sdf->HasElement("collider_radius"))
    _sdf->GetElement("collider_radius")->GetValue()->Get(collider_radius_);
  else
    collider_radius_= 0.175;

  Reset();

  if( mode_ == "body" )
  {
    this->updateConnection = this->sensor_->ConnectUpdated(
        std::bind(&CleanpackCollider::Update_collider_body, this));
  } else if( mode_ == "charge" ){
    this->updateConnection = this->sensor_->ConnectUpdated(
        std::bind(&CleanpackCollider::Update_collider_charge, this));
  } else{
    this->updateConnection = this->sensor_->ConnectUpdated(
        std::bind(&CleanpackCollider::Update_collider_lidar, this));
  }

  mSensorInterface = new ZmqInterface(ZMQ_PP_TYPE::SEND, SID_SENSOR);
  //collider_data.header.len = sizeof(CP_COLLIDER);
  // activate RaySensor
  sensor_->SetActive(true);
}

void CleanpackCollider::Reset()
{
  updateTimer.Reset();
}

float CleanpackCollider::calcDistance(float x, float y, float x1, float y1)
{
  return sqrt( pow(x-x1,2)+pow(y-y1,2) );
}


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void CleanpackCollider::Update_collider_body()
{
  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  // Get Robot Pose.
  parent_ = world->GetModel("cleanpack");
  math::Pose robot_pose = this->parent_->GetWorldPose();
  float robot_yaw = robot_pose.rot.GetYaw();

  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->sensor_->Contacts();
  // variable
  unsigned long depth = 0;
  double contact_angle = 0.0;
  float robot2contact_distance = 0.0;
  float contact2world_angle = 0.0f;
  unsigned int j = 0;

  //collider_angle_.data = M_PI;
  collider_data.collider_angle = M_PI*1000;
  contact_angle = M_PI;

  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    depth = 0;
    contact_angle = 0.0;
    j = 0;
    for (j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      //calculated distance with robot centrer to contact.
      robot2contact_distance = calcDistance( robot_pose.pos.x,
                                             robot_pose.pos.y,
                                             contacts.contact(i).position(j).x(),
                                             contacts.contact(i).position(j).y() );
      // not robot frond +-90degree
      if( CHECK_EQUAL(robot2contact_distance, collider_radius_, 0.003) == false )
      {
        continue;
      }

      contact2world_angle = atan( ( contacts.contact(i).position(j).y() - robot_pose.pos.y )/( contacts.contact(i).position(j).x() - robot_pose.pos.x ) );
      if( ( contacts.contact(i).position(j).x() - robot_pose.pos.x )<0.0f )
      {
        if(contact2world_angle < 0.0f)contact2world_angle+=M_PI;
        else contact2world_angle-=M_PI;
      }
      contact_angle += contact2world_angle;
      depth += contacts.contact(i).depth(j)*10000*10000*100;

    }
    contact_angle = contact_angle/j;
    //ROS_INFO_STREAM("   pos:" << robot_pose.pos.x << " , " << robot_pose.pos.y << "\n");
    //ROS_INFO_STREAM("   Depth:" << depth/j  << "  angle: " << contact_angle/0.0175  << "  robotangle: " << robot_yaw/0.0175 << "  aa:" << (contact_angle - robot_yaw)/0.0175);
    contact_angle = contact_angle - robot_yaw;
    if(abs(contact_angle) > M_PI){
      if(contact_angle < 0){
        contact_angle += (2.0f*M_PI);
      }else{
        contact_angle -= (2.0f*M_PI);
      }
    }
    depth=depth/j;
    if(depth < 1.0f)
    {
      continue;
    }
    collider_data.collider_angle = contact_angle*1000;
    //collider_angle_.data = contact_angle;
  }

  mSensorInterface->send(&collider_data, sizeof(CP_COLLIDER));
  //printf("collider : %.3f\n",contact_angle);
}

void CleanpackCollider::Update_collider_charge()
{
  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  // Get Robot Pose.
  parent_ = world->GetModel("cleanpack");
  math::Pose robot_pose = this->parent_->GetWorldPose();
  float robot_yaw = robot_pose.rot.GetYaw();

  parent_ = world->GetModel("charger");
  if(parent_ == nullptr){
    return;
  }
  math::Pose charge_pose = this->parent_->GetWorldPose();
  float charger_yaw = charge_pose.rot.GetYaw();

  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->sensor_->Contacts();
  float distance = calcDistance( robot_pose.pos.x,
                    robot_pose.pos.y,
                    charge_pose.pos.x,
                    charge_pose.pos.y);
  if( CHECK_EQUAL(distance, collider_radius_, 0.02) == false )
  {
    //collider_state_.data = 0;
  }else if(CHECK_EQUAL(abs(charger_yaw - robot_yaw), 0.0f, M_PI/180.0f*5.0f) ){
    //collider_state_.data = 1;
  }
  //ROS_INFO_STREAM("   yaw " << charger_yaw  << " " << robot_yaw);
  //ROS_INFO_STREAM("   collision ");

  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    //ROS_INFO_STREAM("   collision2:" << contacts.contact(i).collision2() << "  ==  " << contactModel_);
    /*if( strstr(contacts.contact(i).collision1().c_str(),this->contactModel_.c_str()) ) {
      collider_state_.data = 1;
      break;
    }*/
  }

  //publisher_.publish(collider_state_);
}

void CleanpackCollider::Update_collider_lidar()
{
  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  // Get Robot Pose.
  parent_ = world->GetModel("cleanpack");
  math::Pose robot_pose = this->parent_->GetWorldPose();
  float robot_yaw = robot_pose.rot.GetYaw();

  parent_ = world->GetModel("charger");
  if(parent_ == nullptr){
    return;
  }
  math::Pose charge_pose = this->parent_->GetWorldPose();
  float charger_yaw = charge_pose.rot.GetYaw();

  // Get all the contacts.
  msgs::Contacts contacts;
  
  double lidar_x = robot_pose.pos.x - 0.105f*cos(robot_yaw);
  double lidar_y = robot_pose.pos.y - 0.105f*sin(robot_yaw);
  
  contacts = this->sensor_->Contacts();
  //ROS_INFO_STREAM(" lidar pose "<<lidar_x << " " << lidar_y);
  //collider_state_.data = 0;
  
  if(contacts.contact_size() > 0){
    //collider_state_.data = 1;
  
    int depth = 0;
    double contact_angle = 0.0;
    int j = 0;
    for (j = 0; j < contacts.contact(0).position_size(); ++j)
    {
      //calculated distance with robot centrer to contact.
      double contact2world_angle = atan( ( contacts.contact(0).position(j).y() - lidar_y )/( contacts.contact(0).position(j).x() - lidar_x ) );
      if( ( contacts.contact(0).position(j).x() - lidar_x )<0.0f )
      {
        if(contact2world_angle < 0.0f)contact2world_angle+=M_PI;
        else contact2world_angle-=M_PI;
      }
      contact_angle += contact2world_angle;
    
    }
    contact_angle = contact_angle/j - robot_yaw;
    if(abs(contact_angle) > M_PI){
      if(contact_angle < 0){
        contact_angle += (2.0f*M_PI);
      }else{
        contact_angle -= (2.0f*M_PI);
      }
    }
    //ROS_INFO_STREAM("angle "<<contact_angle);
    float distance = calcDistance( robot_pose.pos.x,
                                   robot_pose.pos.y,
                                   contacts.contact(0).position(0).x(),
                                   contacts.contact(0).position(0).y() );
    if( fabs(contact_angle)>M_PI_2*0.5f || distance>collider_radius_){
      //collider_state_.data = 0;
    }
  }

  //publisher_.publish(collider_state_);
}


// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(CleanpackCollider)

} // namespace gazebo
