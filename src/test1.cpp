//
// Created by huang on 18-9-18.
//

#include "comm/ZmqInterface.h"

int main(int argc, char *argv[])
{
    ZmqInterface send(ZMQ_PP_TYPE::SEND, 7888);
    
    
    while(1){
        send.send((void *)"aaaa", 4);
        printf("send.\n");
        sleep(1);
    }
    
    return 0;
}
