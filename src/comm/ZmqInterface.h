//
// Created by huang on 18-9-18.
//

#ifndef PROJECT_ZMQINTERFACE_H
#define PROJECT_ZMQINTERFACE_H

#include <thread>
#include <cstring>
#include <mutex>
#include <zmq.h>
#include <string>
#include <sstream>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

enum ZMQ_PP_TYPE{
    SEND=0,
    RECV
};

typedef boost::function<void(void *, const uint8_t *, uint32_t)>  MsgProcFunc;

class ZmqInterface{
private:
    MsgProcFunc mProcFunc;
    boost::mutex lock;
public:
    ZmqInterface(ZMQ_PP_TYPE type, int port);
    ~ZmqInterface();
    bool isOpened();
    int send(void *data, uint32_t len);
    bool Start();
    
    template<class T>
    bool setCallBack( void(T::*fp)(void *, const uint8_t *, uint32_t), T* obj ){
        if( this->mType != ZMQ_PP_TYPE::RECV ){
            return false;
        }
        this->mProcFunc = boost::bind(fp, obj, _1, _2, _3);
        return true;
    };
private:
    ZMQ_PP_TYPE mType;
    std::thread *mProcessThread;
    
    
    void *mZmqSend;
    void *mZmqRecv;
    static void *mZmqContext;
    
    int mPort;
    bool isStop;
    
    bool InitSendSock();
    bool InitRecsSock();
    
    
};

#endif //PROJECT_ZMQINTERFACE_H
