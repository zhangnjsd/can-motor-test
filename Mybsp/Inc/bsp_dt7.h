#ifndef __BSP_DT7_H
#define __BSP_DT7_H

#include "main.h"
#include "stdint.h"
#include "stdlib.h"
#include <stdio.h>
#include <string.h>

#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart3
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

typedef __packed struct
{
  int16_t ch0;
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t roll;
  uint8_t sw1;
  uint8_t sw2;
} rc_info_t;
 
#define rc_Init   \
{                 \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
		0,            \
}
extern uint8_t dbus_buf[DBUS_BUFLEN];
void uart_receive_handler(UART_HandleTypeDef *huart);
static void uart_rx_idle_callback(UART_HandleTypeDef* huart);
void rc_callback_handler(rc_info_t *rc, uint8_t *buff);
void dbus_uart_init(void);//DBUS���ڳ�ʼ��
extern rc_info_t rc;


#endif
