#ifndef __PID_H__
#define __PID_H__

#include "main.h"
#include <math.h>
typedef enum
{
    POSITION_PID,
    DELTA_PID
} PID_ModeTypeDef;

typedef struct
{
    PID_ModeTypeDef mode;
    float Kp;
    float Ki;
    float Kd;
    float feedback;
    float integral;
    float error[3];
    float target;
    float output;
    float integral_limit;
    float output_limit;        // Add output limiting
    float dead_zone;          // Add dead zone for small errors
    float derivative_filter;  // Add derivative filtering coefficient
    uint8_t integral_reset;   // Flag for integral reset
}PID_InitTypeDef;

void PID_Init(PID_InitTypeDef *pid, PID_ModeTypeDef mode);
float PID_Calculate(PID_InitTypeDef *pid,float set,float feedback);
void PID_SetOutputLimit(PID_InitTypeDef *pid, float limit);
void PID_SetDeadZone(PID_InitTypeDef *pid, float dead_zone);
void PID_Reset(PID_InitTypeDef *pid);

#endif
