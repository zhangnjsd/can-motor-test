#ifndef __INS_TASK_FUN_H_
#define __INS_TASK_FUN_H_

#include <stdint.h>
typedef struct dt7_data_t {
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
} dt7_data_t;

extern dt7_data_t dt7_data;

extern void dt7_recv_task_fun(void const * argument);
#endif
