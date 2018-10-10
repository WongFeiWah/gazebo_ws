//
// Created by huang on 8/22/18.
//

#ifndef CLEANPACK3_ZMQ_PORT_H
#define CLEANPACK3_ZMQ_PORT_H

#include <thread>
#include <communication_port.h>
#include <zmq_defines.h>
#include <cstring>
#include <zmq.h>
#include <string>
#include <sstream>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>

typedef boost::function<void(void *, const uint8_t *, uint32_t)>  MsgProcFunc;

class ZmqPort:public CommunicationPort
{
protected:
  void InitZmq( char *send_ip, uint32_t send_id, uint32_t recv_id );
  MsgProcFunc mProcFunc;
  uint8_t send_buffer[1024];
  
  void *mZmqSend;
  void *mZmqRecv;
  void *mCallbackParam;
  std::thread *mZmqPortProcessThread;
  boost::mutex lock;
	bool mStopProcessFlag;
public:
  ZmqPort(char *send_ip, uint32_t send_id, uint32_t recv_id);
  ~ZmqPort();
  

  template<class T>
  bool setCallBack( void(T::*fp)(void *, const uint8_t *, uint32_t), T* obj ){
    this->mProcFunc = boost::bind(fp, obj, _1, _2, _3);
    this->mCallbackParam = (void*)obj;
    return true;
  };
  bool IsOpened();

  void Start();
  void Stop();
  static void *mZmqContext;
  int32_t Send(const uint8_t *data, int32_t len);
};

#endif //CLEANPACK3_ZMQ_PORT_H
