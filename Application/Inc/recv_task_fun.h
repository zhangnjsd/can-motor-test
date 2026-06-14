#ifndef __RECV_TASK_FUN_H
#define __RECV_TASK_FUN_H

#include "cmsis_os.h"
#include <limits.h>
#include "main.h"
#include "bsp_can.h"
#include <stdio.h>
#include "gimble_task_fun.h"

void recv_task_fun(void *argument);

#endif /* __RECV_TASK_FUN_H */