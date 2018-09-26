//
// Created by huang on 8/22/18.
//

#ifndef CLEANPACK3_BASE_PORT_H
#define CLEANPACK3_BASE_PORT_H

#include <stdint.h>
#include <string>

typedef void(*CommunicationPortRxCallBackDef)(const uint8_t *, int32_t);

enum CommunicationPortType{
  NOP = 0,
  SERIAL_PORT,
  ZMQ_PORT,
  END
};

using namespace std;
class CommunicationPort
{
public:
  CommunicationPort();
  virtual ~CommunicationPort();

  virtual bool IsOpened() = 0;
  virtual int32_t Send(const uint8_t *data, int32_t len) = 0;

  CommunicationPortType mType;
};

#endif //CLEANPACK3_BASE_PORT_H
