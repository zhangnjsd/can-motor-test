#include "recv_task_fun.h"
#include <math.h>

// Rewrite fputc to use UART6 for printf
int __io_putchar(int ch) {
    uint8_t temp = (uint8_t)ch;
    HAL_UART_Transmit(&huart6, &temp, 1, HAL_MAX_DELAY);
    return ch;
}
int fputc(int ch, FILE *f) {
    return __io_putchar(ch);
}

void recv_task_fun(void *argument) {
    can_filter_init();
    // * Print divider for reducing print frequency
    uint32_t print_div = 0;
    uint32_t next_wake_tick = osKernelGetTickCount();

    for (;;) {
        // * Get data from CAN.
        current_angle = ((uint16_t)rx_data[0] << 8) | rx_data[1];
        current_speed = ((uint16_t)rx_data[2] << 8) | rx_data[3];
        current_torque = ((uint16_t)rx_data[4] << 8) | rx_data[5];
        
        // * Set and send controller.
        volt[1] = 0;
        volt[2] = 0;
        volt[3] = 0;
        CAN_Send(&hcan1, volt, CAN_ID);

        // ? Output Freq = 100Hz
        if (++print_div >= 10) {
            print_div = 0;
            // Bias.
            printf("stream:%f,0\n",
                (float_t)current_angle - (float_t)target_angle_dbg);
        }

        
        // ? Req&Set Freq = 1000Hz
        next_wake_tick += 1;
        osDelayUntil(next_wake_tick);
    }
}