#include "bsp_can.h"
#include "main.h"
#include "can_send_task_fun.h"

void can_send_task_fun(void *argument) {
    can_filter_init();
    for (;;) {
/* 
        CAN_TxHeaderTypeDef tx_header;

        uint8_t msg[4] = {1, 0, 0, 0};
        uint32_t mailbox;

        tx_header.StdId = LOCAL_CAN_ID;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = 0x08;

        HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg, &mailbox);

        osDelay(1000);
         */

        // ! Should run this task in FIFO.
        osDelay(1000);
    }
}

// ? Rewrite RX callback
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    if (hcan == &hcan1) {
        // Read remote can id
        if (rx_header.StdId == REMOTE_CAN_ID) {
            CAN_TxHeaderTypeDef tx_header;

            // ? Echo back the message.
            uint8_t msg[4] = {rx_data[0], 0, 0, 0};

            uint32_t mailbox;

            tx_header.StdId = LOCAL_CAN_ID;
            tx_header.IDE = CAN_ID_STD;
            tx_header.RTR = CAN_RTR_DATA;
            tx_header.DLC = 0x08;

            HAL_CAN_AddTxMessage(&hcan1, &tx_header, msg, &mailbox);
            
        }
    }
}