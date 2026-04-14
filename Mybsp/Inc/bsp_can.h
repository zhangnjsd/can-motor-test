#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "main.h"

void CAN_Send(CAN_HandleTypeDef *hcan, int16_t *voltage,uint32_t StdId);

extern void can_filter_init(void);
extern volatile uint32_t can_rx_count;
extern volatile uint8_t can_rx_fresh;
extern CAN_RxHeaderTypeDef can_rx_header;
extern uint8_t rx_data[8];

#endif
