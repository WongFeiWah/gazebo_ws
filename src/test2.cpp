//
// Created by huang on 18-9-18.
//

#include "comm/ZmqInterface.h"

class C_TEST{
public:
    C_TEST(){}
    ~C_TEST(){}
    void Proc(void *param, const uint8_t *data, uint32_t len){
        //data[len] = 0;
        if( param == NULL && len != 0)
            printf("recv  %c%c%c%c\n",data[0],data[1],data[2],data[3]);
    }
    
};


int main(int argc, char *argv[])
{
    C_TEST cc;
    ZmqInterface recv(ZMQ_PP_TYPE::RECV, 7888);
    recv.setCallBack(&C_TEST::Proc, &cc);
    recv.Start();
    while(1){
        sleep(1);
    }
    
    return 0;
}