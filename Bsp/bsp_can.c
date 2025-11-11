#include "bsp_can.h"

extern CAN_HandleTypeDef hcan1;

/*
 * @brief CAN滤波器初始化
 * @param 无
 * @return 无
 * @note 配置为屏蔽滤波器,不过滤
 */
void CAN_Filter_Init(void)
{
    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = ENABLE;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.FilterIdHigh = 0x0000;
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterBank = 0;
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO1;
    HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

    #ifdef CAN_2_USE
    canfilterconfig.SlaveStartFilterBank = 14;
    canfilterconfig.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    #endif
}

