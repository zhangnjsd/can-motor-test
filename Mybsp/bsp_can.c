#include "bsp_can.h"
#include "main.h"


int je=0;

volatile uint32_t can_rx_count = 0;
volatile uint8_t can_rx_fresh = 0;
CAN_RxHeaderTypeDef can_rx_header;
uint8_t rx_data[8] = {0};
	
uint16_t angle_motor=0;
int16_t speed=0;
uint16_t current=0;
uint8_t Temperature=0;

void CAN_Send(CAN_HandleTypeDef *hcan, int16_t *voltage,uint32_t StdId) 
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};  // 清零所有数据字节
    uint32_t send_mailbox;     // 发送邮箱号（由HAL_CAN_AddTxMessage填充）

    // 配置CAN帧头（控制ID 1~4，标识符0x1FF）
    tx_header.StdId = StdId;       // 标准标识符
    tx_header.IDE = CAN_ID_STD;    // 标准帧
    tx_header.RTR = CAN_RTR_DATA;  // 数据帧
    tx_header.DLC = 8;             // 数据长度8字节
    tx_header.TransmitGlobalTime = DISABLE;

    // 填充数据（ID=1的电压值，范围-25000~25000）
		// 填充数据（ID=1的电压值，范围-16384~16384)
    tx_data[0] = (voltage[0] >> 8) & 0xFF;  // 高8位
    tx_data[1] = voltage[0] & 0xFF;         // 低8位
    tx_data[2] = (voltage[1] >> 8) & 0xFF;  // 高8位
    tx_data[3] = voltage[1] & 0xFF;         // 低8位
    tx_data[4] = (voltage[2] >> 8) & 0xFF;  // 高8位
    tx_data[5] = voltage[2] & 0xFF;         // 低8位
		
	tx_data[6] = (voltage[3] >> 8) & 0xFF;  // 高8位
    tx_data[7] = voltage[3] & 0xFF;         // 低8位
	
    // DATA[2-7] 保留为0（其他ID未使用）

    // 发送CAN帧（使用hcan1）
    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &send_mailbox);
}

// 补充中断回调函数，接收CAN消息并存储到全局变量中
static void CAN_RxFifo0_CopyFrame(CAN_HandleTypeDef *hcan)
{
    uint8_t rx_buf[8] = {0};

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, rx_buf) != HAL_OK)
    {
      return;
    }

    for (int i = 0; i < 8; i++)
    {
      rx_data[i] = rx_buf[i];
    }

    can_rx_fresh = 1;
    can_rx_count++;
    je++;
}




void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
