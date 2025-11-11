#include "DM4310.h"

#ifdef DM4310_USE

float Joint_Torque_Set[4]={0.0f,0.0f,0.0f,0.0f};

/*DM4310状态数据*/
volatile DM4310_State Left_Front_Joint_State;
volatile DM4310_State Left_Back_Joint_State;
volatile DM4310_State Right_Front_Joint_State;
volatile DM4310_State Right_Back_Joint_State;
/*DM4310原始数据*/
volatile DM4310_Data_Raw Left_Front_Joint_Raw;
volatile DM4310_Data_Raw Left_Back_Joint_Raw;
volatile DM4310_Data_Raw Right_Front_Joint_Raw;
volatile DM4310_Data_Raw Right_Back_Joint_Raw;
/*CAN消息数据*/
uint8_t tx0_data[8];
/*函数声明*/
float DM4310_Receive_Calc(uint16_t raw_data, float max_value, float min_value, int bits);
uint16_t DM4310_Send_Calc(float torque ,float max_torque, float min_torque, int bits);
void DM4310_Set_Torque(DM4310_ID id, float torque);
void DM4310_Send_Message(DM4310_ID id, uint8_t* data);
void DM4310_Data_Read(volatile DM4310_State *state, uint8_t* rx_data);
float DM4310_Receive_Calc(uint16_t raw_data, float max_value, float min_value, int bits);

/*
 * @brief 函数简介:DM_J4310初始化
 * @param 无
 * @return 无
 * @note 每发送两组数据,需要加200us延时
 * @note 使能命令:0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
 */
void DM4310_Init(void)
{
    uint8_t init_data[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
    DM4310_Send_Message(DM4310_Left_Front, init_data);
    DM4310_Send_Message(DM4310_Left_Back, init_data);
    HAL_Delay(5);
    DM4310_Send_Message(DM4310_Right_Front, init_data);
    DM4310_Send_Message(DM4310_Right_Back, init_data);
    HAL_Delay(5);
}

/*
 * @brief 函数简介:DM_J4310失能
 * @param 无
 * @return 无
 * @note 断电命令:0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD
 */
void DM4310_DeInit(void)
{
    uint8_t deinit_data[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
    DM4310_Send_Message(DM4310_Left_Front, deinit_data);
    DM4310_Send_Message(DM4310_Left_Back, deinit_data);
    HAL_Delay(5);
    DM4310_Send_Message(DM4310_Right_Front, deinit_data);
    DM4310_Send_Message(DM4310_Right_Back, deinit_data);
    HAL_Delay(5);
}

/*
 * @brief DM4310设置力矩命令
 * @param id:电机ID
 * @param torque:力矩值(Nm)
 * @return 无
 */
void DM4310_Set_Torque(DM4310_ID id, float torque)
{

    uint16_t torque_cmd=DM4310_Send_Calc(torque,DM4310_MAX_TORQUE,DM4310_MIN_TORQUE,12);
    tx0_data[0]=0;
    tx0_data[1]=0;
    tx0_data[2]=0;
    tx0_data[3]=0;
    tx0_data[4]=0;
    tx0_data[5]=0;
    tx0_data[6]=torque_cmd>>8;
    tx0_data[7]=torque_cmd&0xff;
    DM4310_Send_Message(id, tx0_data);
}

/*
 * @brief DM4310发送CAN消息
 * @param id:电机ID
 * @param data_t:发送数据指针
 * @return 无
 */
void DM4310_Send_Message(DM4310_ID id, uint8_t* data_t)
{
    CAN_TxHeaderTypeDef DM4310_tx_header;

    DM4310_tx_header.StdId=id;
    DM4310_tx_header.ExtId=0x00;
    DM4310_tx_header.RTR=CAN_RTR_DATA;
    DM4310_tx_header.IDE=CAN_ID_STD;
    DM4310_tx_header.DLC=0x08;

    HAL_CAN_AddTxMessage(&hcan1, &DM4310_tx_header, data_t, (uint32_t *)CAN_TX_MAILBOX0);
}

/*
 * @brief DM4310读取状态数据
 * @param state:电机状态结构体指针
 * @param rx_data:接收数据指针
 * @return 无
 */
void DM4310_Data_Read(volatile DM4310_State *state, uint8_t* rx_data)
{
    uint16_t pos_raw=rx_data[1]<<8 | rx_data[2];
    uint16_t speed_raw=rx_data[3]<<4 | ((rx_data[4]>>4)&0x0F);
    uint16_t torq_raw=((rx_data[4]&0x0F)<<8) | rx_data[5];
    state->Position=DM4310_Receive_Calc(pos_raw,DM4310_MAX_POSITION,DM4310_MIN_POSITION,16);
    state->Speed=DM4310_Receive_Calc(speed_raw,DM4310_MAX_SPEED,DM4310_MIN_SPEED,12);
    state->Torque=DM4310_Receive_Calc(torq_raw,DM4310_MAX_TORQUE,DM4310_MIN_TORQUE,12);

    // data->ID_ERR=rx_data[0];
    // data->POS_H=rx_data[1];
    // data->POS_L=rx_data[2];
    // data->SPEED_H=rx_data[3];
    // data->SPEED_L_TORQ_H=rx_data[4];
    // data->TORQ_L=rx_data[5];
    // data->TEMP_MOS=rx_data[6];
    // data->TEMP_ROTOR=rx_data[7];
}

/*
 * @brief MIT模式下DM4310力矩设置值映射
 * @param torque:力矩值(Nm)
 * @param max_torque:最大力矩值(Nm)
 * @param min_torque:最小力矩值(Nm)
 * @param bits:数据位数
 * @return 计算后的数据
 */
uint16_t DM4310_Send_Calc(float torque ,float max_torque, float min_torque, int bits)
{
    float scale=max_torque-min_torque;
    return (uint16_t)((torque-min_torque)/scale*(float)((1<<bits)-1));
}

/*
 * @brief MIT模式下DM4310接收数据映射
 * @param raw_data:原始数据
 * @param max_value:最大值
 * @param min_value:最小值
 * @param bits:数据位数
 * @return 计算后的数据
 */
float DM4310_Receive_Calc(uint16_t raw_data, float max_value, float min_value, int bits)
{
    float scale=max_value-min_value;
    return (((float)raw_data)*scale/(float)((1<<bits)-1))+min_value;
}

/* CAN1 FIFO0接收回调函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef Rx_header;
    uint8_t Rx_data[8];
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&Rx_header,Rx_data);
    if(Rx_header.StdId==DM4310_Left_Front_Rx)
    {
        DM4310_Data_Read(&Left_Front_Joint_State,Rx_data);
    }
    else if(Rx_header.StdId==DM4310_Left_Back_Rx)
    {
        DM4310_Data_Read(&Left_Back_Joint_State,Rx_data);
    }
    else if(Rx_header.StdId==DM4310_Right_Front_Rx)
    {
        DM4310_Data_Read(&Right_Front_Joint_State,Rx_data);
    }
    else if(Rx_header.StdId==DM4310_Right_Back_Rx)
    {
        DM4310_Data_Read(&Right_Back_Joint_State,Rx_data);
    }
}

#endif
