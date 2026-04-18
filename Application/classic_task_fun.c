#include "main.h"
#include "classic_task_fun.h"
#include "bsp_can.h"

void classic_task_fun(void *argument) {
    j = 0;

    for (;;) {
        uint8_t rx_buf[8] = {0};

        for (int i = 0; i < 8; i++) {
            rx_data[i] = rx_buf[i];
        }

        j++;
        if (j >= INT_MAX) {
            j = 0;
        }

        osDelay(1);
    }
}