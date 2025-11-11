#include "Key.h"
#include "GM6020.h"
#include "DM4310.h"
#include "RobotTask.h"


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin==KEY_Pin)
    {
        #ifdef GM6020_USE
        gm6020_behavior.target_angle += 60.0f;
        DM4310_DeInit();
        if(gm6020_behavior.target_angle>360.0f)
        {
            gm6020_behavior.target_angle -=360.0f;
        }
        #endif
    }
}
