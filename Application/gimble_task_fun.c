#include "bsp_can.h"
#include "main.h"
#include "gimble_task_fun.h"

void gimble_task_fun(void *argument) {
    i = 0;
    uint32_t tick;

    // For transferring data to motor
    can_filter_init();
    int16_t vol[4] = {0};
    double t = 0;
    for (;;) {
        tick = osKernelGetTickCount();
        i ++;
        if (i == UINT32_MAX) {
            i = 0;
        }

        t += 0.001;
        vol[0] = (int16_t)(25000 * sin(t));
        CAN_Send(&hcan1, vol, CAN_ID);

        //osDelay(1);
        osDelayUntil(tick + 1);
    }
}