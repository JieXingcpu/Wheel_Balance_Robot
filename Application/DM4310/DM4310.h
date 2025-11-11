#ifndef DM4310_H
#define DM4310_H

#include "main.h"
#include "can.h"
#include "config.h"

/*DM4310电机MIT控制模式下限制值*/
#define DM4310_MAX_TORQUE 5.0f
#define DM4310_MIN_TORQUE -5.0f
#define DM4310_MAX_SPEED 30.0f
#define DM4310_MIN_SPEED -30.0f
#define DM4310_MAX_POSITION 12.5f
#define DM4310_MIN_POSITION -12.5f

/*4个关节电机ID号枚举*/
typedef enum
{
    DM4310_Left_Front = 0x01,
    DM4310_Left_Back = 0x02,
    DM4310_Right_Front = 0x03,
    DM4310_Right_Back = 0x04
}DM4310_ID;

/*4个关节电机反馈ID号枚举*/
typedef enum
{
    DM4310_Left_Front_Rx = 0xF1,
    DM4310_Left_Back_Rx = 0xF2,
    DM4310_Right_Front_Rx = 0xF3,
    DM4310_Right_Back_Rx = 0xF4
}DM4310_RxID;

/*DM4310电机状态结构体*/
typedef struct
{
    float Position;
    float Speed;
    float Torque;
}DM4310_State;

/*DM4310电机数据结构体*/
typedef struct
{
    uint8_t ID_ERR;//ID和故障位
    uint8_t POS_H;//位置高8位
    uint8_t POS_L;//位置低8位
    uint8_t SPEED_H;//速度高8位
    uint8_t SPEED_L_TORQ_H;//速度低4位和力矩高4位
    uint8_t TORQ_L;//力矩低8位
    uint8_t TEMP_MOS;//MOS温度
    uint8_t TEMP_ROTOR;//线圈温度
}DM4310_Data_Raw;

extern float Joint_Torque_Set[4];

void DM4310_Init(void);
void DM4310_DeInit(void);
void DM4310_Set_Torque(DM4310_ID id, float torque);


#endif

