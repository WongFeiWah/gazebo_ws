
#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {
    
    class Joint;
    class Entity;
    
    class CleanpackDiffDriver : public ModelPlugin {
        
        enum OdomSource
        {
            ENCODER = 0,
            WORLD = 1,
        };
    public:
        CleanpackDiffDriver();
        ~CleanpackDiffDriver();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void Reset();
    
    protected:
        virtual void UpdateChild();
        virtual void FiniChild();
    
    private:
        void getWheelVelocities();
        void UpdateOdometryEncoder();
        void cmdVelCallback ( const void *data );
        
        
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection_;
        
        double wheel_separation_;
        double wheel_diameter_;
        double wheel_torque;
        double wheel_speed_[2];
        double wheel_accel;
        double wheel_speed_instr_[2];
        
        std::vector<physics::JointPtr> joints_;
        
        std::string tf_prefix_;
        
        boost::mutex lock;
        
        std::string robot_namespace_;
        std::string command_topic_;
        std::string odometry_topic_;
        std::string odometry_frame_;
        std::string robot_base_frame_;
        bool publish_tf_;
        bool legacy_mode_;
        // Custom Callback Queue
        boost::thread callback_queue_thread_;
        void QueueThread();
        
        // DiffDrive stuff
        
        double x_;
        double rot_;
        bool alive_;
        bool isAcc;
        
        // Update Rate
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;
        
        OdomSource odom_source_;
        common::Time last_odom_update_;
    
        math::Pose odom_;
        double encoder_x;
        double encoder_y;
        double encoder_theta;
        
        // Flags
        bool publishWheelTF_;
        bool publishOdomTF_;
        bool publishWheelJointState_;
        
    };
    
}

#endif
