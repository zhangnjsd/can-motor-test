#include "bsp_can.h"
#include "main.h"
#include <stdint.h>
#include "can_send_task_fun.h"

void can_send_task_fun(void *argument) {
    can_filter_init();
    for (;;) {
        CAN_TxHeaderTypeDef tx_header;

        uint8_t msg[4] = {1, 0, 0, 0};
        uint32_t mailbox;

        tx_header.StdId = REMOTE_CAN_ID;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 0x08;

        HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg, &mailbox);
        
        //HAL_UART_Transmit(&huart1, (uint8_t *)"\nSend data: ", 13, 500);
        //HAL_UART_Transmit(&huart1, msg, 4, 500);

        osDelay(1000);
        msg[0]++;
    }
}

// ? Rewrite RX callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1) {
        // Read remote can id
        if (rx_header.StdId == REMOTE_CAN_ID) {
            // ? Process received data (if needed)
            recv_data1 = rx_data[0];
            //HAL_UART_Transmit(&huart1, (uint8_t *)"\nRecv data: ", 13, 500);
            //HAL_UART_Transmit(&huart1, &recv_data1, 1, 500);
        }
    }
}