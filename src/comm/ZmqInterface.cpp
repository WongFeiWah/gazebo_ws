//
// Created by huang on 18-9-18.
//
#include "ZmqInterface.h"

void *ZmqInterface::mZmqContext = nullptr;

ZmqInterface::ZmqInterface(ZMQ_PP_TYPE type, int port){
    this->mPort = port;
    this->mType = type;
    mProcessThread = NULL;
    isStop = true;
    if(ZmqInterface::mZmqContext == nullptr){
        ZmqInterface::mZmqContext = zmq_ctx_new();
        if( ZmqInterface::mZmqContext == NULL) {
            perror("ZmqInterface:zmq_ctx_new fail.");
            return;
        }
        printf("context new.\n");
    }
    
    if( type == ZMQ_PP_TYPE::SEND ){
        if( this->InitSendSock() == false) return;
    }else{
        if( this->InitRecsSock() == false) return;
    }
    callbackParam = NULL;
    isStop = false;
}

ZmqInterface::~ZmqInterface(){
    isStop = true;
    if(mProcessThread!=NULL){
        mProcessThread->join();
        delete  mProcessThread;
    }
    
}

bool ZmqInterface::InitSendSock(){
    //PUSH
    if( this->mPort <= 7800 ) return false;
    std::string str = "";
    std::stringstream str_s;
    str_s.str("");
    str_s << "tcp://" << "127.0.0.1" << ":" << this->mPort;
    str = str_s.str();
    
    if( (mZmqSend = zmq_socket ( ZmqInterface::mZmqContext, ZMQ_PUSH)) == NULL )
    {
        perror("ZmqInterface: zmq_socket fail.");
        return false;
    }
    int hwm = 8;
    zmq_setsockopt(mZmqSend, ZMQ_SNDHWM, &hwm, sizeof(int));
    zmq_setsockopt(mZmqSend, ZMQ_RCVHWM, &hwm, sizeof(int));
    int timeout = 10;
    zmq_setsockopt(mZmqSend, ZMQ_SNDTIMEO, &timeout, sizeof(int));
    zmq_setsockopt(mZmqSend, ZMQ_RCVTIMEO, &timeout, sizeof(int));
    if(zmq_connect(mZmqSend, str.c_str()) != 0){
        perror("ZmqInterface:send handle connect fail. ");
        return false;
    }
    printf("===========================\n");
    printf("=== Send.  %s\n",str.c_str());
    return true;
}
bool ZmqInterface::InitRecsSock(){
    //POLL
    if( this->mPort <= 7800 ) return false;
    std::string str = "";
    std::stringstream str_s;
    
    if( (mZmqRecv = zmq_socket( ZmqInterface::mZmqContext, ZMQ_PULL)) == NULL )
    {
        perror("error: zmq_socket fail.");
        return false;
    }
    int hwm = 8;
    zmq_setsockopt(mZmqRecv, ZMQ_SNDHWM, &hwm, sizeof(int));
    zmq_setsockopt(mZmqRecv, ZMQ_RCVHWM, &hwm, sizeof(int));
    int timeout = 10;
    zmq_setsockopt(mZmqRecv, ZMQ_SNDTIMEO, &timeout, sizeof(int));
    zmq_setsockopt(mZmqRecv, ZMQ_RCVTIMEO, &timeout, sizeof(int));
    // bind addr.
    str_s.str("");
    str_s << "tcp://" << "127.0.0.1" << ":" << this->mPort;
    str = str_s.str();
    if(zmq_bind(mZmqRecv, str.c_str()) != 0){
        perror("ZmqInterface:recv handle bind fail. ");
        return false;
    }
    
    
    printf("===========================\n");
    printf("=== Recv.  %s\n",str.c_str());
    return true;
}

int ZmqInterface::send(void *data, uint32_t len) {
    if(this->mType != ZMQ_PP_TYPE::SEND){
        return -1;
    }
    boost::mutex::scoped_lock scoped_lock(lock);
    return zmq_send(mZmqSend, data, len, 0);
}

bool ZmqInterface::isOpened(){
    if( this->mZmqContext == nullptr ) return false;
    if(isStop) return false;
    return true;
}

bool ZmqInterface::Start(){
    if(this->mType != ZMQ_PP_TYPE::RECV){
        return false;
    }
    mProcessThread = new std::thread([](void *param)
    {
        ZmqInterface *this_ = (ZmqInterface *)param;
        uint8_t buffer[1024] = {0};
        printf("mZmqPortProcessThread\n");
        while (!this_->isStop)
        {
            int len = zmq_recv(this_->mZmqRecv, buffer, 1024, 0);
            if(len > 0 && this_->mProcFunc != nullptr){
                this_->mProcFunc(this_->callbackParam, buffer, len);
            }
        }
    },this);
    return true;
}
