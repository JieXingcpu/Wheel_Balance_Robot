#ifndef GM6020_H
#define GM6020_H


#include "main.h"
#include "can.h"
#include "PID.h"
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"

void gm6020_set_current(int16_t current);
void GM6020_Init(void);

typedef struct
{
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint16_t temperature;
} GM6020_Data_t;

/*angle值为-180到180*/
typedef struct 
{
    int16_t speed;
    float angle;
    float target_angle;
}GM6020_Behavior;

    
extern volatile GM6020_Data_t gm6020_data;
extern PID_InitTypeDef gm6020_position_pid;
extern PID_InitTypeDef gm6020_speed_pid;
extern GM6020_Behavior gm6020_behavior;

#endif // GM6020_H
