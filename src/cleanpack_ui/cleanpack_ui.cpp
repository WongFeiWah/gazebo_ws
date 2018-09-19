//
// Created by huang on 18-9-18.
//

#include "comm/ZmqInterface.h"
#include "comm/cleanpack_struct.h"
#include <termio.h>
#include <stdio.h>
#include <comm/cleanpack_struct.h>

#define REVERSE_BOOL(n) (n ? n = false : n = true)

class CleanpackUINode
{
private:
    bool keyPower;
    bool keyHome;
    bool isPickUp;
    bool isTest_Turn_PI;
    ZmqInterface *mControlInterface;
    
public:
    CleanpackUINode();
    ~CleanpackUINode();
    void DisplayHelp();
    void ModeSel(char key);
    void SendControl(float v, float w, bool isAcc);
    char GetKey();
    void Run();
};

CleanpackUINode::CleanpackUINode(){
    this->keyPower = false;
    this->keyHome = false;
    this->isPickUp = false;
    mControlInterface = new ZmqInterface(ZMQ_PP_TYPE::SEND, 7888);
}
CleanpackUINode::~CleanpackUINode(){
    
}

char CleanpackUINode::GetKey(){
    char key;
    fd_set rfds;
    struct timeval tv;
    
    FD_ZERO(&rfds);
    FD_SET(0, &rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 10; //设置等待超时时间
    
    
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_lflag &=~ECHO;
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);
    
    
    
    //检测键盘是否有输入
    if (select(1, &rfds, NULL, NULL, &tv) > 0) {
        key = getchar();
        tcsetattr(0,TCSANOW,&stored_settings);
    }else{
        return 0;
    }
    return key;
}

void CleanpackUINode::ModeSel(char key) {
    switch (key){
        case '1':
            REVERSE_BOOL(keyPower);
            break;
        case '2':
            if(keyHome)keyHome = false;
            else keyHome = true;
            break;
        case '3':
            if(isPickUp)isPickUp = false;
            else isPickUp = true;
            break;
        case '4':
            if(isTest_Turn_PI)isTest_Turn_PI = false;
            else isTest_Turn_PI = true;
            break;
        case 'i':
            this->SendControl(0.3f,0.0f,false);
            break;
        case ',':
            this->SendControl(-0.3f,0.0f,false);
            break;
        case 'j':
            this->SendControl(0.0f,1.0f,false);
            break;
        case 'l':
            this->SendControl(0.0f,-1.0f,false);
            break;
        case 'k':
            this->SendControl(0.0f,0.0f,false);
            break;
        default:
            break;
    }
    
    
}

void CleanpackUINode::SendControl(float v, float w, bool isAcc){
    uint8_t buffer[80] = {0};
    CP_HENDER *header = (CP_HENDER *)buffer;
    CP_CMDVEL *cmd = (CP_CMDVEL *)(buffer+sizeof(CP_HENDER));
    header->hender = HENDER_XX;
    header->len = sizeof(CP_CMDVEL);
    header->type = CP_TYPE::TYPE_CMD_VEL;
    cmd->isAcc = isAcc;
    cmd->x = v*1000;
    cmd->z = w*1000;
    mControlInterface->send(buffer, sizeof(CP_HENDER)+sizeof(CP_CMDVEL));
}

void CleanpackUINode::DisplayHelp(){
    printf("==========================================\n");
    printf("select action:\n");
    printf("1 - key power      %s\n",(this->keyPower ? "Down":"Up"));
    printf("2 - key home       %s\n",(this->keyHome ? "Down":"Up"));
    printf("3 - pick up        %s\n",(this->isPickUp ? "Down":"Up"));
    printf("4 - test turn pi   %s\n",(this->isTest_Turn_PI ? "Down":"Up"));
    printf("==========================================\n");
}

void CleanpackUINode::Run(){
    char key = 0;
    int dis_count = 0;
    DisplayHelp();
    while(true){
        dis_count++;
        if(dis_count>100000){
            dis_count = 0;
            DisplayHelp();
        }
        
        
        if(key != 0) {
            ModeSel(key);
            DisplayHelp();
            //printf("input %02X\n",key);
            isTest_Turn_PI = false;
        }
        
        key = GetKey();
    }
}

int main(int argc, char *argv[])
{
    CleanpackUINode node;
    node.Run();
    return 0;
}