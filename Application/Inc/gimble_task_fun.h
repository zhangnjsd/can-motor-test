#ifndef __GIMBLE_TASK_FUN_H
#define __GIMBLE_TASK_FUN_H

#include "bsp_can.h"
#include "main.h"
#include "cmsis_os.h"
#include <limits.h>
#include <math.h>
#include "controller.h"

#define CAN_ID 0x1FF

void gimble_task_fun(void *argument);

#endif /* __GIMBLE_TASK_FUN_H */