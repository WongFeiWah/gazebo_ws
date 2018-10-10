
#include <inttypes.h>

#ifndef __MCU_PROTOCOL_H__
#define __MCU_PROTOCOL_H__

// Carrier Control mode
#define SPEED_MODE         0
#define ALONG_WALL_MODE    1
#define CHARGE_MODE        2
#define STEP_MODE		   3
#define GO_DIRECT_USE_GYRO 4

#define MAX_MCU_BLOCK_SIZE 50
#define MAX_LIDAR_BLOCK_SIZE 50
#define MAX_NET_BLOCK_SIZE 412
#define IO_INTERFACE_TIME_INTERVAL 300

typedef float f32;
typedef double f64;

typedef signed long long s64;
typedef signed int  s32;
typedef signed short s16;
typedef signed char  s8;

typedef signed long long const sc64;
typedef signed int  const sc32;  /* Read Only */
typedef signed short const sc16;  /* Read Only */
typedef signed char  const sc8;   /* Read Only */

typedef volatile signed long long vs64;
typedef volatile signed int  vs32;
typedef volatile signed short vs16;
typedef volatile signed char  vs8;

typedef volatile signed long long  const vsc64;
typedef volatile signed int  const vsc32;  /* Read Only */
typedef volatile signed short const vsc16;  /* Read Only */
typedef volatile signed char  const vsc8;   /* Read Only */

typedef unsigned long long u64;
typedef unsigned int  u32;
typedef unsigned short u16;
typedef unsigned char  u8;

typedef unsigned long long const uc64;
typedef unsigned int  const uc32;  /* Read Only */
typedef unsigned short const uc16;  /* Read Only */
typedef unsigned char  const uc8;   /* Read Only */

typedef volatile unsigned long long  vu64;
typedef volatile unsigned int  vu32;
typedef volatile unsigned short vu16;
typedef volatile unsigned char  vu8;

typedef volatile unsigned long long const vuc64;  /* Read Only */
typedef volatile unsigned int  const vuc32;  /* Read Only */
typedef volatile unsigned short const vuc16;  /* Read Only */
typedef volatile unsigned char  const vuc8;   /* Read Only */


#define REMOTE_CTRL 0xFD		//远程控制
#define REMOTE_FUNC_SIG 0x00	//远程控制的信号控制
#define MCU_ADDRESS 0x12
#define FACTORY_ADDRESS 0xFE
#define _USE_IMU_DATA_ 1



#define PSD_TAR_DIS 0.02f

#define CAR_MAX_V 0.3f//*SPEED_RATIO
#define CAR_MAX_W 2.0f//*SPEED_RATIO

#define CAR_WALLFOLLOW_V 0.2f//*SPEED_RATIO
#define CAR_WALLFOLLOW_AL_V 0.2f//*SPEED_RATIO
#define CAR_WALLFOLLOW_AL_W (-0.9f)//*SPEED_RATIO

#define CAR_REFLEX_SPIN_W 1.5f

#define CAR_WALLFOLLOW_PID_P   (-0.2f) //(40.0f/sqrt(max(SPEED_RATIO,1.0f)))
#define CAR_WALLFOLLOW_PID_I   (-1.0f) //(800.0f/max(SPEED_RATIO,1.0f))

#define COLLISION_LAST_TIME    ((int64_t)(600000/3.0f)) //Last 600ms
#define MOVE_LAST_TIME    ((int64_t)(600000/3.0f)) //Last 600ms

#pragma pack(push, 1)


typedef struct _McuProtocolHeader {
  u8 deviceAddr;	//  cmd
  u8 functionCode;	// sub cmd
  u16 offset;		//sub sub cmd
  u32 len;

  _McuProtocolHeader(u8 addr, u8 funcCode, u16 start, u32 len)
  {
    deviceAddr = addr;
    functionCode = funcCode;
    offset = start;
    this->len = len;
  }

  _McuProtocolHeader()
  {
    deviceAddr = 0;
    functionCode = 0;
    offset = 0;
    this->len = 0;
  }
}McuProtocolHeader;

typedef enum {
  MS_IDLE,
  MS_CONDITIONED_REFLEX,
  MS_STEP,
  MS_WALL_FOLLOW,
  MS_COMMON_SPEED_LOOP,
  MS_CONDITIONED_REFLEX_TIMEOUT,
  MS_TEST,
  MS_AUTO_STOP_AT_CHARGE,
  MS_GO_DIRECT_USE_GYRO,
  MS_AUTO_RECHARGE,
}MotionStatus_Type;

typedef struct {
  s32 angularPos;
  s32 leftEncoderPos;      ///<当前左边里程计的积分位置
  s32 rightEncoderPos;     ///<当前右边里程计的积分位置
}CarrierEncoder;

typedef struct {
  s16 gyroPitch;
  s16 gyroRoll;
  s16 gyroYaw;
  s16 accelPitch;
  s16 accelRoll;
  s16 accelYaw;
}ImuData;

typedef struct {
  u16 ultrasound[8];
  u8 dropSensor;
  u16 irSensor;
  u8 collisionSensor;
  s32 angularPos;
  s32 leftEncoderPos;      ///<当前左边里程计的积分位置
  s32 rightEncoderPos;     ///<当前右边里程计的积分位置
  s32 lineVelocity;
  s32 angularVelocity;
  u8 chargeStatus;
  u8 batteryStatus;
  u8 pickupStatus;		//抱起
  u16 errorState;
  u16 wallDistance;
  s32 feedbackStatus;
  ImuData imuData;
  u8 garbageFreqLevel;
  u16 wheelCurrentL;
  u16 wheelCurrentR;
  u16 sideBroomCurrentL;
  u16 sideBroomCurrentR;
  u16 midBroomCurrent;
  u16 fanSpeed;
  s16 temperature;
  u16 waterPumpCurrent;
  u8 isAlongingWall;

  // Gavin Added
  s32 lSpeed;
  s32 rSpeed;
  s32 lPwm;
  s32 rPwm;
  u8 motorControlStatus;
  u8 motionStatus;
  u8 alongWallStatus;
  u8 realBatteryLevel;
  u16 batteryVoltage;
  s16 skidError;
  s32 debugInfo1;
  s32 debugInfo2;

}ChassisStatusRegister;

typedef struct {
  u32 systickMs;				//s16
  s32 angularPos;
  s32 leftEncoderPos;			///<当前左边里程计的积分位置
  s32 rightEncoderPos;		///<当前右边里程计的积分位置
  s32 lineVelocity;			//s16
  s32 angularVelocity;		//s16
  s16 skidError;
  s16 reserve;				//-2

  ImuData imuData;
}FastChassisStatusRegister; //28 * 200 + 12 *100 = 6800

typedef struct     //
{
  u16 ultrasound[8];
  u32 systickMs;				//u16
  u8 collisionSensor;
  u8 dropSensor;
  u16 irSensor;
  u16 batteryVoltage;
  u16 errorState;
  u16 wallDistance;
  u16 wheelCurrentL;		//?
  u16 wheelCurrentR;		//?
  u16 sideBroomCurrentL;	//?
  u16 sideBroomCurrentR;	//?
  u16 midBroomCurrent;
  u16 fanSpeed;
  s16 temperature;		//?
  u16 waterPumpCurrent;

  u8 chargeStatus;
  u8 batteryStatus;
  u8 realBatteryLevel;
  u8 pickupStatus;
  u8 garbageFreqLevel;
  u8 isAlongingWall;        // 当前是否在沿墙
  u8 motorControlStatus;    // mcu 内部状态机值
  u8 motionStatus;           // mcu 内部状态机值
  u8 alongWallStatus;       // mcu 内部状态机值

  u8 fanValue;
  u8 midBroomValue;
  u8 sideBroomValue;

  s32 lSpeed;			//s16
  s32 rSpeed;			//s16
  s32 lPwm;				//s16
  s32 rPwm;				//s16
  u32 feedbackStatus;

  u32 curLedInfo;
  u32 lastBeepInfo;
  u32 sensorSwitchStatus;

  s32 debugInfo1;
  s32 debugInfo2;
  //u32	beepBeginTime;
}SlowChassisStatusRegister; //(98-16)*20 = 1800

typedef struct {
  u32 systickMs;        //s16
  s32 angularPos;
  s32 leftEncoderPos;      ///<当前左边里程计的积分位置
  s32 rightEncoderPos;    ///<当前右边里程计的积分位置
  s16 lineVelocity;      //s16
  s16 angularVelocity;    //s16
  s16 skidError;       //            ===0   ===20 ----20        0x00 0x14

  ImuData imuData;

  u8 collisionSensor;
  u8 dropSensor;
  u16 irSensor;
  u16 batteryVoltage;
  u16 errorState;
  u16 wallDistance;
  u16 wheelCurrentL;    //?
  u16 wheelCurrentR;    //?             ===32  ----14           0x20  0x0e

  u16 sideBroomCurrentL;  //?
  u16 sideBroomCurrentR;  //?
  u16 midBroomCurrent;
  u16 fanSpeed;
  s16 temperature;    //?
  u16 waterPumpCurrent;//                ===46   ----12        0x2e  0x0C

  u8 chargeStatus;
  u8 batteryStatus;
  u8 realBatteryLevel;
  u8 pickupStatus;
  u8 garbageFreqLevel;
  u8 isAlongingWall;        // 当前是否在沿墙
  u8 motorControlStatus;    // mcu 内部状态机值
  u8 motionStatus;           // mcu 内部状态机值
  u8 alongWallStatus;       // mcu 内部状态机值

  u8 fanValue;
  u8 midBroomValue;
  u8 sideBroomValue; //             ===58     ----12          0x3A  0x0C

  s16 lSpeed;      //s16
  s16 rSpeed;      //s16
  s16 lPwm;        //s16
  s16 rPwm;        //s16
  u32 feedbackStatus;          // ===70       // ----12       0x46  0X0C

  u32 curLedInfo;
  u32 lastBeepInfo;
  u32 sensorSwitchStatus;//      ===82          ---12        0x52  0x0C

  s32 debugInfo1;
  s32 debugInfo2;      //老版本协议的慢包  ===94    ----8   0x5e 0x08

}ChassisStatusRegisterInterval;

struct ChassisControlRegister {
  s32 lineVelocity;
  s32 angularVelocity;
  u8 mode;
  s32 vAcc;
  s32 wAcc;
  s32 stepS;
  s16 stepPhi;
  u32 switchControl;
  u8 fanLevel;

  u32 beep;
  u32 led;

  u8 sideBroomLevel;
  u8 midBroomLevel;

  u8 lidarWallDis;
  u8 cecMode;
  u8 virtualCollision;
  u8 waterPumpLevel;
  u8 wifiLedControl;
  u8 rev[11];
};

typedef struct {
  u32 version;
  u32 hardwareVersion;
  u32 subHardwareVersion;
}ChassisParamRegister;

typedef struct {
  s32 vectorTargetDistance;
  s32 vectorTargetAngle;
  s32 relativeTargetPosX;
  s32 relativeTargetPosY;
  s32 setLineVelocity;
  s32 setAngularVelocity;
  u8 backUint8Tge;
  u8 backUint8TgeThreshold;
  s32 setBackUint8TgePosX;
  s32 setBackUint8TgePosY;
  u8 startMapping;
  u8 stopMapping;
  u8 setDefaultMap;
  u8 emergencyStop;
}AlgControlRegister;

typedef struct {
  u8 workMode;
  s32 lineVelocity;
  s32 angularVelocity;
  s32 posX;
  s32 posY;
  s32 posSita;
  u16 errorState;
}AlgStatusRegister;

typedef struct {
  McuProtocolHeader* header;
  char radom[4];
  char cpuId[12];
}McuIDReg;

typedef enum {
  SIG_TO_CHARGE = 3000,       // 回充
  SIG_SPOT,                   // 定点扫
  SIG_CLEAN_OR_PAUSE,         // 清扫/暂停，二合一的清扫暂停，如果启动和暂停是分开的，请不要使用此code
  SIG_CHANGE_FAN_SPEED,       //控制风机速度
  SIG_RESET_WIFI,             //重置wifi
  SIG_FORWARD,                //前进
  SIG_BACK,                   //后退
  SIG_TURN_LEFT,              //左转
  SIG_TURN_RIGHT,             //右转
  SIG_CHECK_WIFI_STATUS,      //查询wifi状态
  SIG_FIND_ROBOT_START,       //开始查找机器人
  SIG_FIND_ROBOT_STOP,        //停止查找机器人
  SIG_FACTORY_RESET,          //恢复出厂设置
  SIG_SET_SPEED,              //设置遥控
  SIG_RC_CONNECT,             //遥控器配对成功
  SIG_MAX_NUM,                //最后一个信号边界
}RfCtrlCode;

typedef struct _RfCtrlDef {
  McuProtocolHeader header;
  uint32_t sigCode;	//RfCtrl 定义

  _RfCtrlDef()
  {
    header.deviceAddr = REMOTE_CTRL;		//0xFD
    header.functionCode = REMOTE_FUNC_SIG;	//0x00	//远程控制的信号控制
    header.offset = 0;
    header.len = 0;
    sigCode = 0;
  }
}RfCtrlDef;

#pragma pack(pop)

// MCU
#define MCU_CONTROL_REG_READ   0x01
#define MCU_CONTROL_REG        0x02
#define MCU_STATUS_REG         0x03
#define MCU_PARAM_REG          0x04
#define MCU_USER_REG_READ      0x05
#define MCU_USER_REG_WRITE     0x06
#define MCU_UID_REG_READ       0x08
#define MCU_FIRMWARE_WRITE     0x09
#define MCU_FIRMWARE_RESPOND   0x0A
#define MCU_FACTORY_CMD		   0x0B									//function code 表示是工厂时候的指令
#define MCU_SRNSORS_ADJUST	   0x0C
#define MCU_SENSOR_READ        0x0D
#define MCU_SENSOR_WRITE       0x0E
#define MCU_WIFI_SN_WRITE      0x0F		//写SN
#define MCU_WIFI_SN_REG        0x10		//读SN


#define MCU_SLOW_STATUS_REG     0X11
#define MCU_FAST_STATUS_REG     0x12

#define MCU_RTC_SET_CLOCK      0x13	//写入RTC闹钟
#define MCU_RTC_GET_TIME       0x14	//get RTC时间
#define MCU_RTC_SET_TIME	   0x15	//向RTC发送时间
#define MCU_STATUS_REG_NEW_PROTOCOL	   0x16	//新组包
#define MCU_RESTART_CMD			0x32



#define MCU_FACTORY_SUBCMD_READY			0x01								//startAddr 表示是准备就绪
#define MCU_FACTORY_SUBCMD_CHECK			0x02								//表示是开始检测的指令
#define MCU_FACTORY_SUBCMD_RESULT			0x03								//表示检测结果
#define MCU_FACTORY_SUBCMD_CAN_GET_SN		0x04								//通知上层可以获取SN
#define MCU_FACTORY_SUBCMD_GET_SN			0x05
#define MCU_FACTORY_SUBCMD_INTOFACTORYMODE	0x06								//通知MCU进入检测模式



#define FACTORY_FUNC_GETSN          0x00    //获取SN
#define FACTORY_FUNC_CLEANPACK_INFO 0x01    //CleanPack的系统信息
#define FACTORY_FUNC_ONLINE         0x02	//告知设备在线
#define FACTORY_FUNC_SETSN          0x03	//设置SN

//MCU 升级相关
// Firmware respond
#define FW_ACK              0
#define FW_DOWNLOAD_ERR     1
#define FW_CHECK_ERR        2

//MCU 状态位相关
#define SW_MID_BROOM                        (1<<0)
#define SW_SIDE_BROOM                       (1<<1)
#define SW_POWER_DOWN                       (1<<2)
#define SW_WATER_PUMP		                (1<<3)
#define SW_DISABLE_DROP_PROTECT	            (1<<4)
#define SW_DISABLE_WALL_PROTECT		        (1<<5)
#define SW_DISABLE_COLLISION_STUCK_PROTECT  (1 << 6)  // 关闭碰撞超时处理，当碰撞超时后，将会禁用碰撞
#define SW_LIDAR_CLOSE_OBSTACLE_FLAG        (1 << 7)  // 雷达探测到近处障碍物(< 2cm)标志
#define SW_ENABLE_AUTO_STOP_AT_CHARGE       (1 << 8)  // 检测到充电信号后，自动刹车停机
#define SW_RESEND_RF_KEY                    (1 << 9)  // 收到之后，将会发送一次遥控器KEY
#define SW_DISABLE_COLLISION_PROTECT        (1 << 10) // 置1后，将关闭碰撞保护
#define SW_DISABLE_AUTO_CLOSE_IR            (1 << 11) // 置1后，将关闭空闲时自动光比IR传感器
#define SW_POWEROFF_COLLISION               (1 << 12) //置1后，关闭碰撞总电源
#define SW_POWEROFF_DROP                    (1 << 13) //置1后，关闭地检总电源
#define SW_POWEROFF_WALL                    (1 << 14) //置1后，关闭强检总电源
#define SW_POWEROFF_WIFI                    (1 << 15) //置1后，关闭WIFI总电源
#define SW_POWEROFF_RF_KEY                  (1 << 16) //置1后，关闭遥控器总电源
#define SW_POWEROFF_WHEEL                   (1 << 17) //置1后，关闭轮子总电源
#define SW_POWEROFF_MCU_ALL                 (1 << 18) //置1后，关闭总电源和MCU
#define SW_POWEROFF_TOP                     (1 << 19) //置1后，关闭顶部碰撞电源
#define SW_POWEROFF_GARBAGE                 (1 << 20) //置1后，关闭垃圾总电源
#define SW_LED_LOW_POWER					(1 << 21) //置1后，按键灯光变暗
#define SW_IRNEAR_SIGNAL					(1 << 22) //置1后，忽略近位信号带来的应激反应
#define SW_VER_BIGGER_THAN_6155				(1 << 25) //置1后，表示使用的是6155以上的版本

// errorState
#define ERR_STATUS_IMU                  (1<<0)
#define ERR_STATUS_L_SIDE_I             (1<<1)
#define ERR_STATUS_R_SIDE_I             (1<<2)
#define ERR_STATUS_MID_I                (1<<3)
#define ERR_STATUS_L_WHEEL              (1<<4)
#define ERR_STATUS_R_WHEEL              (1<<5)
#define ERR_STATUS_FAN_SPEED            (1<<6)
#define ERR_STATUS_COLLISION            (1<<7)
#define ERR_STATUS_DROP                 (1<<8)
#define ERR_STATUS_WATER_PUMP_I         (1<<9)	//水泵电流过大
#define ERR_STATUS_IMU_DATA_NO_CHANGE   (1<<10)
#define ERR_STATUS_WALL_DIRTY	        (1<<11)	// wall checker dirty
#define ERR_STATUS_PSD_DIRTY	        (1<<12)	// psd sensor dirty
#define ERR_STATUS_LIDAR_EDGE           (1<<14) // lidar edge shelter
#define ERR_ARC_FAILED			        (1<<15)	// too many collision caused Backcharge failed.

// feedbackStatus
#define STATUS_IS_STEPING           (1<<0)
#define STATUS_WATER_BOX_EMPTY      (1<<1)
#define STATUS_WATER_BOX_PLUGIN     (1<<2)
#define STATUS_GARBAGE_FULL         (1<<3)
#define STATUS_GARBAGE_BOX_PLUGIN   (1<<4)
#define STATUS_STEPING_BREAK		(1<<5)
#define STATUS_KEY1					(1<<6)
#define STATUS_KEY2					(1<<7)
#define STATUS_IS_MUSIC_PLAY        (1<<9)      //is playing music
#define STATUS_LONG_PERIOD          (1<<10)     //no used now
#define STATUS_USELESS              (1<<11)     //应嵌入式要求暂时弃用此标志位
#define STATUS_IS_CHECK_CRC_FLAG1   (1<<12)     //13和14位都是1才会校验crc的包
#define STATUS_IS_CHECK_CRC_FLAG2   (1<<13)
#define STATUS_MOP_PLUGIN           (1<<14)     //拖布支架

typedef enum _WifiLedControl {
  WIFI_LED_OFF = 0,
  WIFI_LED_ON,
  WIFI_LED_FLASH_F,
  WIFI_LED_FLASH_S,
  WIFI_LED_BREATH_F,
  WIFI_LED_BREATH_S,
}WifiLedStatus;

//检查内容
typedef enum {
  CHECK_CLEAN_STATUS,
  CHECK_WIFI_STATUS
}CheckCode;

//error code
typedef enum {
  LidarSerialError = -11000,
  FileNotTotal = -11001,
}FactoryModeErrorCode;

//################################以下为九阳专用协议############################
//deivce addr
#define  MCU_CLEANER_STATUS_ADDRESS		0x20	//表示专用通讯地址
//function code
#define MCU_CLEANER_STATUS_CODE_CHECK	0x00	//表示请求扫地机状态
#define MCU_CLEANER_STATUS_CODE_STATUS	0x01	//表示状态码互通

#pragma pack(push, 1)

//关于这个状态  特别说明的是，子状态一般只在清扫中和充电中存在。如果充电中存在非NULL的子状态，表示充电完成后将会进行续扫
typedef struct {
  uint8_t cleanStatus;			//扫地机状态        CleanPackStatus
  uint8_t cleanStatusSub;			//扫地机子状态		CleanSubMode
  uint8_t wifiStatus;				//网络状态			0 表示未联网 1表示已经联网
  uint8_t rev;					//保留
}CleanCheckStatus;




// fast package
typedef struct McuFastPackage{
  McuProtocolHeader header;
  FastChassisStatusRegister reg;
  McuFastPackage() {
    header.deviceAddr = MCU_ADDRESS;
    header.functionCode = MCU_FAST_STATUS_REG|0x80;
    header.len = sizeof(reg);
    header.offset = 0;
  }
}McuFastPackage;

// slow package
typedef struct McuSlowPackage{
  McuProtocolHeader header;
  SlowChassisStatusRegister reg;
  McuSlowPackage() {
    header.deviceAddr = MCU_ADDRESS;
    header.functionCode = MCU_SLOW_STATUS_REG|0x80;
    header.len = sizeof(reg);
    header.offset = 0;
  }
}McuSlowPackage;


#pragma pack(pop)


#endif
