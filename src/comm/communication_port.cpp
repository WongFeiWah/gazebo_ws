//
// Created by huang on 8/22/18.
//
#include <comm/communication_port.h>

CommunicationPort::CommunicationPort() {
  this->mType = CommunicationPortType::NOP;
}

CommunicationPort::~CommunicationPort() {

}

int32_t CommunicationPort::Send(const uint8_t *data, int32_t len){
  return 0;
}
