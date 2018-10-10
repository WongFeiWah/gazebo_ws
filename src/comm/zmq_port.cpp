//
// Created by huang on 8/22/18.
//
#include <zmq_port.h>

void *ZmqPort::mZmqContext = nullptr;

ZmqPort::ZmqPort(char *send_ip, uint32_t send_id, uint32_t recv_id):CommunicationPort() {

  this->mType = CommunicationPortType::ZMQ_PORT;
  mStopProcessFlag = true;
  mZmqPortProcessThread = NULL;
  InitZmq(send_ip,send_id,recv_id);
}

void ZmqPort::InitZmq( char *send_ip, uint32_t send_id, uint32_t recv_id ){
  std::string str = "";
  std::stringstream str_s;
  //**************************************************
  // Context Init
  if(ZmqPort::mZmqContext == nullptr){
    ZmqPort::mZmqContext = zmq_ctx_new();
    if( ZmqPort::mZmqContext == NULL) {
      perror("ZmqPort:zmq_ctx_new fail.");
      return;
    }
  }

  // Publisher Init
  str_s.str("");
  str_s << "tcp://" << send_ip << ":" << send_id;
  str = str_s.str();
  mZmqSend = zmq_socket ( ZmqPort::mZmqContext, ZMQ_PUSH);
  int hwm = 8;
  zmq_setsockopt(mZmqSend, ZMQ_SNDHWM, &hwm, sizeof(int));
  zmq_setsockopt(mZmqSend, ZMQ_RCVHWM, &hwm, sizeof(int));
  int timeout = 1;
  zmq_setsockopt(mZmqSend, ZMQ_SNDTIMEO, &timeout, sizeof(int));
  zmq_setsockopt(mZmqSend, ZMQ_RCVTIMEO, &timeout, sizeof(int));
  if(zmq_connect(mZmqSend, str.c_str()) != 0){
    perror("ZmqPort:send handle bind fail. ");
    return;
  }

  //**************************************************
  // Pull Init
  // socket init.
  if( (mZmqRecv = zmq_socket( ZmqPort::mZmqContext, ZMQ_SUB)) == NULL )
  {
    perror("error: zmq_socket fail.");
    return ;
  }
  hwm = 8;
  zmq_setsockopt(mZmqRecv, ZMQ_SNDHWM, &hwm, sizeof(int));
  zmq_setsockopt(mZmqRecv, ZMQ_RCVHWM, &hwm, sizeof(int));
  timeout = 10;
  zmq_setsockopt(mZmqRecv, ZMQ_SNDTIMEO, &timeout, sizeof(int));
  zmq_setsockopt(mZmqRecv, ZMQ_RCVTIMEO, &timeout, sizeof(int));
  // bind addr.
  str_s.str("");
  str_s << "tcp://" << send_ip << ":" << recv_id;
  str = str_s.str();
  if(zmq_connect(mZmqRecv, str.c_str()) != 0){
    perror("ZmqPort:recv handle bind fail. ");
    return;
  }

  if(0 != zmq_setsockopt(mZmqRecv, ZMQ_SUBSCRIBE, "", 0)){
    perror("ZmqPort:zmq_setsockopt fail. ");
    return;
  }
  printf("===========================\n");
  printf("Zmq Init.\n");
  printf("send : %s:%d\n",send_ip, send_id);
  printf("recv : %s:%d\n",send_ip, recv_id);
  printf("===========================\n");
}

ZmqPort::~ZmqPort() {

}

bool ZmqPort::IsOpened() {
  if(mStopProcessFlag){
    return false;
  }
  return true;
}

void ZmqPort::Start(){
  mStopProcessFlag = false;
  mZmqPortProcessThread = new std::thread([](void *param)
	{
    ZmqPort *this_ = (ZmqPort *)param;
    uint8_t buffer[1024] = {0};
    uint8_t *readData = &buffer[0];
		while (!this_->mStopProcessFlag)
		{
			int len = zmq_recv(this_->mZmqRecv, buffer, 1024, 0);
      if(len > 0){
        this_->mProcFunc(this_->mCallbackParam, buffer, len);
      }
		}
	},this);
  mZmqPortProcessThread->detach();
}

void ZmqPort::Stop(){
  mStopProcessFlag = true;
	
  if(mZmqPortProcessThread){
    printf("ZmqPort[%s:%d]\n",__FILE__,__LINE__);
    if (mZmqPortProcessThread->joinable())
      mZmqPortProcessThread->join();
    printf("ZmqPort[%s:%d]\n",__FILE__,__LINE__);
  }
}

int32_t ZmqPort::Send(const uint8_t *data, int32_t len) {
  if(!IsOpened()){
    return 0;
  }
  int slen = len;
  boost::mutex::scoped_lock scoped_lock(lock);
  memcpy(send_buffer, data, len);
  //printf("send %d\n",len);
  return zmq_send(mZmqSend, send_buffer, slen, 0);
}

