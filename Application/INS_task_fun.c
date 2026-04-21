#include "INS_task_fun.h"
#include "main.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "math.h"

/**
 * @brief 云台解算任务函数
 * @param argument 与freertos.c中若定义保持一致
 */
void INS_task_fun(void const * argument)
{
	for(; ;)
	{ 
		INS_Task();
		osDelay(1);
	}
}
