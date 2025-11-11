#include "PID.h"
#define ABS(x) ((x) > 0 ? (x) : -(x))
void PID_Init(PID_InitTypeDef *pid, PID_ModeTypeDef mode)
{
    pid->Kp = pid->Kp;
    pid->Ki = pid->Ki;
    pid->Kd = pid->Kd;
    pid->mode = mode;
    pid->integral = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->target = 0.0f;
    pid->output = 0.0f;
    pid->integral_limit = pid->integral_limit;
    pid->output_limit = 3000.0f;  // Default GM6020 current limit
    pid->dead_zone = 0.0f;
    pid->derivative_filter = 0.8f; // Low-pass filter coefficient
    pid->integral_reset = 0;
}

float PID_Calculate(PID_InitTypeDef *pid,float set,float feedback)
{
    if(pid==NULL)   return 0.0f;
    
    pid->target = set;
    pid->feedback = feedback;
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->target - pid->feedback;

    // Dead zone check
    if(ABS(pid->error[0]) < pid->dead_zone)
    {
        pid->error[0] = 0.0f;
        pid->error[1] = pid->error[0];
        pid->error[2] = pid->error[1];
    }

    // Reset integral if target changed significantly
    if(pid->integral_reset || ABS(pid->error[0] - pid->error[1]) > 500.0f)
    {
        pid->integral = 0.0f;
        pid->integral_reset = 0;
    }

    // Conditional integration - only integrate if output is not saturated
    if(ABS(pid->output) < pid->output_limit)
    {
        pid->integral += pid->error[0];
    }
    
    // Integral limiting
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    if(pid->mode==DELTA_PID)
    {
        pid->output += pid->Kp * (pid->error[0] - pid->error[1])
                        + pid->Ki * pid->error[0]
                        + pid->Kd * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
    }
    else
    {
        // Apply derivative filtering
        float derivative = pid->error[0] - pid->error[1];
        static float last_derivative = 0.0f;
        derivative = pid->derivative_filter * last_derivative + (1.0f - pid->derivative_filter) * derivative;
        last_derivative = derivative;
        
        pid->output = pid->Kp * pid->error[0]
                        + pid->Ki * pid->integral
                        + pid->Kd * derivative;
    }

    // Output limiting
    if (pid->output > pid->output_limit)
        pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit)
        pid->output = -pid->output_limit;

    return pid->output;
}

void PID_SetOutputLimit(PID_InitTypeDef *pid, float limit)
{
    if(pid != NULL)
    {
        pid->output_limit = limit;
    }
}

void PID_SetDeadZone(PID_InitTypeDef *pid, float dead_zone)
{
    if(pid != NULL)
    {
        pid->dead_zone = dead_zone;
    }
}

void PID_Reset(PID_InitTypeDef *pid)
{
    if(pid != NULL)
    {
        pid->integral = 0.0f;
        pid->error[0] = 0.0f;
        pid->error[1] = 0.0f;
        pid->error[2] = 0.0f;
        pid->output = 0.0f;
    }
}
