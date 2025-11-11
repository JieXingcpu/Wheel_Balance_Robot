#ifndef __ROBOT_TASK_H
#define __ROBOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "GM6020.h"
#include "DM4310.h"
#include "config.h"
#include "queue.h"

void FreeRTOS_Start(void);
void startTask(void* pvParameters);



#endif 