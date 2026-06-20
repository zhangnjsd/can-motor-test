#ifndef __RECV_TASK_FUN_H
#define __RECV_TASK_FUN_H

#include <stdio.h>
#include <limits.h>

#include "cmsis_os2.h"
#include "main.h"
#include "bsp_can.h"
#include "bsp_dt7.h"

struct dt7_data_t {
	int16_t channel0;
	int16_t channel1;
	int16_t channel2;
	int16_t channel3;
	uint8_t switch1;
	uint8_t switch2;
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_left;
	uint8_t mouse_right;
	uint16_t key_code;
};

extern struct dt7_data_t dt7_data;


void recv_task_fun(void *argument);

#endif /* __RECV_TASK_FUN_H */