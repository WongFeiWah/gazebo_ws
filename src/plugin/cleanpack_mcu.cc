
#include <algorithm>
#include <assert.h>

#include "cleanpack_diff_driver.hh"
#include "cleanpack_struct.h"

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>
#include <comm/cleanpack_struct.h>


namespace gazebo
{
    
    enum {
        RIGHT,
        LEFT,
    };
    
    CleanpackMcu::CleanpackMcu() {}

// Destructor
    CleanpackMcu::~CleanpackMcu() {}

// Load the controller
    void CleanpackMcu::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
        
        this->parent = _parent;
        gzmsg << "CleanpackMcu Load.\n";
        
        
        
        
        
        // Initialize update rate stuff
        if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
        else this->update_period_ = 0.0;

        mControlInterface = new ZmqInterface(ZMQ_PP_TYPE::SEND, SID_CONTROL);

        mSensorInterface = new ZmqInterface(ZMQ_PP_TYPE::RECV, SID_SENSOR);
        mSensorInterface->setCallBack(&CleanpackMcu::cmdVelCallback, this);
        mSensorInterface->Start();
        
    }
    
    void CleanpackMcu::Reset()
    {
    }

    
    
    GZ_REGISTER_MODEL_PLUGIN ( CleanpackMcu )
}
