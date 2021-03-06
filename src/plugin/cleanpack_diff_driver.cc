
#include <algorithm>
#include <assert.h>

#include "cleanpack_diff_driver.hh"
#include "cleanpack_struct.h"

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>



namespace gazebo
{
    
    enum {
        RIGHT,
        LEFT,
    };
    
    CleanpackDiffDriver::CleanpackDiffDriver() {}

// Destructor
    CleanpackDiffDriver::~CleanpackDiffDriver() {FiniChild();}

// Load the controller
    void CleanpackDiffDriver::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
        
        this->parent = _parent;
      printf("CleanpackDiffDriver Load.\n");
        
        wheel_separation_ = 0.24;
        wheel_diameter_ = 0.07;
        wheel_accel = 1.2;
        wheel_torque = 24;
        odom_source_ = ENCODER;
        update_rate_ = 50;
        
        
        joints_.resize ( 2 );
        joints_[LEFT] = parent->GetJoint("left_wheel_joint");
        joints_[RIGHT] = parent->GetJoint("right_wheel_joint");
#if GAZEBO_MAJOR_VERSION > 2
        joints_[LEFT]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT]->SetParam ( "fmax", 0, wheel_torque );
#else
        joints_[LEFT]->SetMaxForce ( 0, wheel_torque );
        joints_[RIGHT]->SetMaxForce ( 0, wheel_torque );
#endif
        
        
        
        // Initialize update rate stuff
        if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;
        last_update_time_ = parent->GetWorld()->GetSimTime();
        
        // Initialize velocity stuff
        wheel_speed_[RIGHT] = 0;
        wheel_speed_[LEFT] = 0;
        
        // Initialize velocity support stuff
        wheel_speed_instr_[RIGHT] = 0;
        wheel_speed_instr_[LEFT] = 0;
        
        x_ = 0;
        rot_ = 0;
        isAcc = true;
        alive_ = true;
        
        // listen to the update event (broadcast every simulation iteration)
        this->update_connection_ =
                event::Events::ConnectWorldUpdateBegin ( boost::bind ( &CleanpackDiffDriver::UpdateChild, this ) );


        mControlInterface = new ZmqInterface(ZMQ_PP_TYPE::RECV, SID_CONTROL);
        mControlInterface->setCallBack(&CleanpackDiffDriver::cmdVelCallback, this);
        mControlInterface->Start();
        mSensorInterface = new ZmqInterface(ZMQ_PP_TYPE::SEND, SID_SENSOR);
    }
    
    void CleanpackDiffDriver::Reset()
    {
        last_update_time_ = parent->GetWorld()->GetSimTime();
        
        x_ = 0;
        rot_ = 0;
#if GAZEBO_MAJOR_VERSION > 2
        joints_[LEFT]->SetParam ( "fmax", 0, wheel_torque );
  joints_[RIGHT]->SetParam ( "fmax", 0, wheel_torque );
#else
        joints_[LEFT]->SetMaxForce ( 0, wheel_torque );
        joints_[RIGHT]->SetMaxForce ( 0, wheel_torque );
#endif
    }

// Update the controller
    void CleanpackDiffDriver::UpdateChild()
    {
        
        /* force reset SetParam("fmax") since Joint::Reset reset MaxForce to zero at
           https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
           (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
           and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than GazeboRosDiffDrive::Reset
           (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
        */
        for ( int i = 0; i < 2; i++ ) {
#if GAZEBO_MAJOR_VERSION > 2
            if ( fabs(wheel_torque -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
                joints_[i]->SetParam ( "fmax", 0, wheel_torque );
#else
            if ( fabs(wheel_torque -joints_[i]->GetMaxForce ( 0 )) > 1e-6 ) {
                joints_[i]->SetMaxForce ( 0, wheel_torque );
#endif
            }
        }
        
        
        if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
        common::Time current_time = parent->GetWorld()->GetSimTime();
        double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
        
        if ( seconds_since_last_update > update_period_ ) {
            publicOdom();
            
            // Update robot in case new velocities have been requested
            getWheelVelocities();
            
            double current_speed[2];
            
            current_speed[LEFT] = joints_[LEFT]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
            current_speed[RIGHT] = joints_[RIGHT]->GetVelocity ( 0 ) * ( wheel_diameter_ / 2.0 );
            
            if ( wheel_accel == 0 ||
                 ( fabs ( wheel_speed_[LEFT] - current_speed[LEFT] ) < 0.01 ) ||
                 ( fabs ( wheel_speed_[RIGHT] - current_speed[RIGHT] ) < 0.01 ) ) {
                //if max_accel == 0, or target speed is reached
#if GAZEBO_MAJOR_VERSION > 2
                joints_[LEFT]->SetParam ( "vel", 0, wheel_speed_[LEFT]/ ( wheel_diameter_ / 2.0 ) );
            joints_[RIGHT]->SetParam ( "vel", 0, wheel_speed_[RIGHT]/ ( wheel_diameter_ / 2.0 ) );
#else
                joints_[LEFT]->SetVelocity ( 0, wheel_speed_[LEFT]/ ( wheel_diameter_ / 2.0 ) );
                joints_[RIGHT]->SetVelocity ( 0, wheel_speed_[RIGHT]/ ( wheel_diameter_ / 2.0 ) );
#endif
            } else {
                
                double acc = 0.0f;
                if(isAcc) {
                    acc = wheel_accel;
                }else{
                    acc = 10.0f;
                }
                
                
                
                if ( wheel_speed_[LEFT]>=current_speed[LEFT] )
                    wheel_speed_instr_[LEFT]+=fmin ( wheel_speed_[LEFT]-current_speed[LEFT],  acc * seconds_since_last_update );
                else
                    wheel_speed_instr_[LEFT]+=fmax ( wheel_speed_[LEFT]-current_speed[LEFT], -acc * seconds_since_last_update );
                
                if ( wheel_speed_[RIGHT]>current_speed[RIGHT] )
                    wheel_speed_instr_[RIGHT]+=fmin ( wheel_speed_[RIGHT]-current_speed[RIGHT], acc * seconds_since_last_update );
                else
                    wheel_speed_instr_[RIGHT]+=fmax ( wheel_speed_[RIGHT]-current_speed[RIGHT], -acc * seconds_since_last_update );
                
                // ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], wheel_speed_[LEFT]);
                // ROS_INFO("actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],wheel_speed_[RIGHT]);

#if GAZEBO_MAJOR_VERSION > 2
                
                joints_[LEFT]->SetParam ( "vel", 0, wheel_speed_instr_[LEFT] / ( wheel_diameter_ / 2.0 ) );
        joints_[RIGHT]->SetParam ( "vel", 0, wheel_speed_instr_[RIGHT] / ( wheel_diameter_ / 2.0 ) );

#else
                joints_[LEFT]->SetVelocity ( 0,wheel_speed_instr_[LEFT] / ( wheel_diameter_ / 2.0 ) );
                joints_[RIGHT]->SetVelocity ( 0,wheel_speed_instr_[RIGHT] / ( wheel_diameter_ / 2.0 ) );
#endif
            }
            last_update_time_+= common::Time ( update_period_ );
        }
    }

// Finalize the controller
    void CleanpackDiffDriver::FiniChild()
    {
        alive_ = false;
    }
    
    void CleanpackDiffDriver::getWheelVelocities()
    {
        //boost::mutex::scoped_lock scoped_lock ( lock );
        lock.lock();
        double vr = x_;
        double va = rot_;
        
        if(legacy_mode_)
        {
            wheel_speed_[LEFT] = vr + va * wheel_separation_ / 2.0;
            wheel_speed_[RIGHT] = vr - va * wheel_separation_ / 2.0;
        }
        else
        {
            wheel_speed_[LEFT] = vr - va * wheel_separation_ / 2.0;
            wheel_speed_[RIGHT] = vr + va * wheel_separation_ / 2.0;
        }
        lock.unlock();
    }
    
    void CleanpackDiffDriver::cmdVelCallback ( void *param, const uint8_t *data, uint32_t len)
    {
        boost::mutex::scoped_lock scoped_lock ( lock );
        CP_HEADER *header = (CP_HEADER*)data;
        assert(data != NULL);

        if( header->header != HEADER_XX || header->type != CP_TYPE::TYPE_CMD_VEL){
            return;
        }

        CP_CMDVEL *cmd = (CP_CMDVEL*)(data + sizeof(CP_HEADER));
        x_ = cmd->v/1000.0f;
        rot_ = cmd->w/1000.0f;

        //printf("cmd:%.2f  %.2f\n", x_, rot_);

        if(cmd->isAcc == 0){
            isAcc = false;
        }else{
            isAcc = true;
        }
    }
    /*
    void CleanpackDiffDriver::QueueThread()
    {
        static const double timeout = 0.01;
        
        while ( alive_ && gazebo_ros_->node()->ok() ) {
            queue_.callAvailable ( ros::WallDuration ( timeout ) );
        }
    }
    */
    void CleanpackDiffDriver::UpdateOdometryEncoder()
    {
        double vl = joints_[LEFT]->GetVelocity ( 0 );
        double vr = joints_[RIGHT]->GetVelocity ( 0 );
        common::Time current_time = parent->GetWorld()->GetSimTime();
        double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
        last_odom_update_ = current_time;
        
        double b = wheel_separation_;
        
        // Book: Sigwart 2011 Autonompus Mobile Robots page:337
        double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
        double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
        
        odom_data.odom.encoder_left += sl*1000.0f;
        odom_data.odom.encoder_right += sr*1000.0f;
    }

    void CleanpackDiffDriver::publicOdom(){
        math::Pose pose = parent->GetWorldPose();
        math::Vector3 linear;
        float yaw = pose.rot.GetYaw();
        linear = parent->GetWorldLinearVel();
        
        odom_data.odom.x = pose.pos.x;
        odom_data.odom.y = pose.pos.y;
        odom_data.odom.z = pose.pos.z;
        odom_data.odom.v = cosf ( yaw ) * linear.x + sinf ( yaw ) * linear.y;
        odom_data.odom.w = parent->GetWorldAngularVel().z;

        //gzmsg << "odom " << odom_data.odom.x << " " << odom_data.odom.y << "\n";
 
        mSensorInterface->send(&odom_data, sizeof(CP_ODOM)); 
    }
    
    
    GZ_REGISTER_MODEL_PLUGIN ( CleanpackDiffDriver )
}
