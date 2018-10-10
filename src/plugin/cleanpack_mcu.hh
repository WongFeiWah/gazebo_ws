#ifndef CLENPACK_MCU_HH
#define CLENPACK_MCU_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <comm/ZmqInterface.h>

namespace gazebo {
    
    class Joint;
    class Entity;
    
    class CleanpackMcu : public ModelPlugin {
        
        enum OdomSource
        {
            ENCODER = 0,
            WORLD = 1,
        };
    public:
        CleanpackMcu();
        ~CleanpackMcu();
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void Reset();
    
    protected:
    
    private:
        
        physics::ModelPtr parent;
        event::ConnectionPtr update_connection_;
        
        boost::mutex lock;
        ZmqInterface *mControlInterface;
        ZmqInterface *mSensorInterface;
        
        // Update Rate
        double update_rate_;
        double update_period_;
        common::Time last_update_time_;
        
        OdomSource odom_source_;
        common::Time last_odom_update_;
    
        math::Pose odom_;
        
    };
    
}

#endif
