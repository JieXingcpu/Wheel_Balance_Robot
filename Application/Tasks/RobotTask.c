#include "RobotTask.h"
#define ABS(x) ((x) > 0 ? (x) : -(x))

#ifdef __USE_FreeRTOS
/*
 * @brief 存放各种任务句柄
 */
TaskHandle_t startTaskHandle;
TaskHandle_t GM6020TaskHandle;
TaskHandle_t DM4310TaskHandle;

static void GM6020_Position_Task(void *pvParameters);
static void DM4310_Control_Task(void *pvParameters);
static void Track_Stack_Task(void *pvParameters);

float gimbal_target_position = 90.0f; // -180 to 180 degrees
float current_set=0.0f;

/*
 * @brief FreeRTOS启动函数
 * @note 创建初始任务并启动调度器
 * @param 无
 */
void FreeRTOS_Start(void)
{
    xTaskCreate((TaskFunction_t)startTask,
                (const char *)"StartTask",
                (configSTACK_DEPTH_TYPE)128,
                (void *)NULL,
                (UBaseType_t)3,
                (TaskHandle_t *)&startTaskHandle);
    vTaskStartScheduler();
}

/*
 * @brief 初始任务函数
 * @note 创建其他任务
 * @param pvParameters:任务参数
 */
void startTask(void* pvParameters)
{
    /*进入临界区*/
    taskENTER_CRITICAL();
    /*创建其他任务*/
    #ifdef GM6020_USE
    xTaskCreate((TaskFunction_t)GM6020_Position_Task,
                (const char *)"GimbalSpeedTask",
                (configSTACK_DEPTH_TYPE)128,
                (void *)NULL,
                (UBaseType_t)1,
                (TaskHandle_t *)&GM6020TaskHandle);
    #endif
    
    #ifdef DM4310_USE
    xTaskCreate((TaskFunction_t)DM4310_Control_Task,
                (const char *)"DM4310ControlTask",
                (configSTACK_DEPTH_TYPE)128,
                (void *)NULL,
                (UBaseType_t)2,
                (TaskHandle_t *)&DM4310TaskHandle);
    #endif

    #ifdef TASK_TRACK_STACK
    xTaskCreate((TaskFunction_t)Track_Stack_Task,
                (const char *)"TrackStackTask",
                (configSTACK_DEPTH_TYPE)128,
                (void *)NULL,
                (UBaseType_t)4,
                NULL);
    #endif

    vTaskDelete(NULL); // Delete the start task after creating other tasks

    taskEXIT_CRITICAL();
    /*退出临界区*/
}

#ifdef GM6020_USE
/*
 * @brief 6020闭环控制任务
 * @param pvParameters:任务参数
 */
static void GM6020_Position_Task(void *pvParameters)
{
    gm6020_behavior.speed=0;
    while (1)
    {
        // Gimbal speed control logic here
        gm6020_behavior.angle = gm6020_data.angle*360.0f/8191.0f;
        // if(gm6020_data.angle>180.0f)    gm6020_behavior.angle -=360.0f;
        gm6020_behavior.speed=PID_Calculate(&gm6020_position_pid, gm6020_behavior.target_angle, gm6020_behavior.angle);
        current_set=PID_Calculate(&gm6020_speed_pid, gm6020_behavior.speed, gm6020_data.speed);
        gm6020_set_current((int16_t)current_set);
        vTaskDelay(pdMS_TO_TICKS(5)); // Delay for 50 milliseconds
    }
}
#endif

#ifdef DM4310_USE
/*
 * @brief DM4310控制任务
 * @param pvParameters:任务参数
 */
static void DM4310_Control_Task(void *pvParameters)
{
    Joint_Torque_Set[0]=0.0f;
    Joint_Torque_Set[1]=2.0f;
    Joint_Torque_Set[2]=0.0f;
    Joint_Torque_Set[3]=0.0f;
    while(1)
    {
        DM4310_Set_Torque(DM4310_Left_Back,Joint_Torque_Set[1]);
        // portDISABLE_INTERRUPTS();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
#endif

#ifdef TASK_TRACK_STACK
/*
 * @brief 任务栈使用率监测函数
 * @param taskHandle:任务句柄
 * @return 任务栈使用的字节
 */
static void Track_Stack_Task(void *pvParameters)
{
    UBaseType_t high_stack_of_DM4310=0;
    UBaseType_t high_stack_of_GM6020=0;
    while(1)
    {
        high_stack_of_DM4310=uxTaskGetStackHighWaterMark(DM4310TaskHandle);
        high_stack_of_GM6020=uxTaskGetStackHighWaterMark(GM6020TaskHandle);
        printf("DM4310 Task High Water Mark: %lu\r\n", high_stack_of_DM4310);
        printf("GM6020 Task High Water Mark: %lu\r\n", high_stack_of_GM6020);
        vTaskDelay(50);
    }
} 
#endif

#endif
