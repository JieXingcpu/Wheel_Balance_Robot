#ifndef CONFIG_H
#define CONFIG_H
/*存放各种宏开关，取消注释即可使能对应功能*/

/*使能DM4310电机驱动*/
#define DM4310_USE 1

/*使能CAN2滤波器*/
// #define CAN_2_USE 1

/*使能GM6020电机*/
// #define GM6020_USE 1

/*使能FreeRTOS*/
#define __USE_FreeRTOS 1

// #define TASK_TRACK_STACK 1

#endif