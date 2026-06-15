#ifndef __RECV_TASK_FUN_H
#define __RECV_TASK_FUN_H

#include <stdio.h>
#include <limits.h>

#include "cmsis_os2.h"
#include "main.h"
#include "bsp_can.h"

void recv_task_fun(void *argument);

#endif /* __RECV_TASK_FUN_H */