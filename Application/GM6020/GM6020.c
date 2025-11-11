#include "GM6020.h"

#ifdef GM6020_USE
uint8_t tx1_data[8];
volatile GM6020_Data_t gm6020_data;
GM6020_Behavior gm6020_behavior;
/*GM6020 位置环PID参数*/
PID_InitTypeDef gm6020_position_pid={
    .mode=POSITION_PID,
    .Kp=7.0f,
    .Ki=0.1f,
    .Kd=0.3f,
    .feedback=0.0f,
    .integral=0.0f,
    .error={0.0f,0.0f,0.0f},
    .target=0.0f,
    .output=0.0f,
    .integral_limit=350.0f,
    .output_limit=500.0f,
    .dead_zone=0.5f,
    .derivative_filter=0.8f,
    .integral_reset=0
};
/*GM6020 速度环PID参数*/
PID_InitTypeDef gm6020_speed_pid={
    .mode=POSITION_PID,
    .Kp=12.3f,
    .Ki=5.15f,
    .Kd=4.95f,
    .feedback=0.0f,
    .integral=0.0f,
    .error={0.0f,0.0f,0.0f},
    .target=0.0f,
    .output=0.0f,
    .integral_limit=1500.0f,
    .output_limit=10000.0f,
    .dead_zone=5.0f,
    .derivative_filter=0.8f,
    .integral_reset=0
};

/*
 * @brief GM6020初始化
 * @param 无
 */
void GM6020_Init(void)
{
    PID_Init(&gm6020_speed_pid, DELTA_PID);
    gm6020_behavior.target_angle = 15.0f;
}
/*
 * @brief GM6020设置电流
 * @param current:电流值
 * @return 无
 * @note 电流值范围为-16384到16384
 */
void gm6020_set_current(int16_t current)
{
    uint32_t send_TX_mailbox;
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = 0x2FE;
	tx_header.ExtId = 0x00;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 0x08;
    tx1_data[0]=(current>>8)&0xff;
    tx1_data[1]=current&0xff;
    tx1_data[2]=(current>>8)&0xff;
    tx1_data[3]=current&0xff;
    tx1_data[4]=(current>>8)&0xff;
    tx1_data[5]=current&0xff;
    tx1_data[6]=(current>>8)&0xff;
    tx1_data[7]=current&0xff;

    HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx1_data,(uint32_t*)CAN_TX_MAILBOX1);
}
/*
 * @brief GM6020状态数据读取
 * @param data:电机状态结构体指针
 * @param rx_data:CAN消息指针
 * @return 无
 */
static void read_gm6020_data(volatile GM6020_Data_t* data, uint8_t* rx_data)
{
    data->angle = (rx_data[0] << 8) | rx_data[1];
    data->speed = (rx_data[2] << 8) | rx_data[3];
    data->current = (rx_data[4] << 8) | rx_data[5];
    data->temperature = (rx_data[6] << 8) | rx_data[7];
}

/* CAN1 FIFO1接收回调函数 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef Rx_header;
  uint8_t Rx_data[8];
  HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&Rx_header,Rx_data);
  if(Rx_header.StdId==0x20A)
  {
      read_gm6020_data(&gm6020_data,Rx_data);
  }
}
#endif
