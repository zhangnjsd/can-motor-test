#include "recv_task_fun.h"

// Rewrite fputc to use UART for printf
int __io_putchar(int ch) {
    uint8_t temp = (uint8_t)ch;
    HAL_UART_Transmit(&huart1, &temp, 1, HAL_MAX_DELAY);
    return ch;
}
int fputc(int ch, FILE *f) {
    return __io_putchar(ch);
}

void recv_task_fun(void *argument) {
    uint32_t print_div = 0;
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;) {
        current_angle = ((uint16_t)rx_data[0] << 8) | rx_data[1];
        current_speed = ((uint16_t)rx_data[2] << 8) | rx_data[3];
        current_torque = ((uint16_t)rx_data[4] << 8) | rx_data[5];

        if (++print_div >= 10) {
            print_div = 0;
            printf("n:%d,%f\n",
                (int16_t)current_speed,
                target_speed_dbg);
        }

        next_wake_tick += 1;
        osDelayUntil(next_wake_tick);
        
    }
}