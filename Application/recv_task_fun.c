#include "recv_task_fun.h"

// Rewrite fputc to use UART for printf
int fputc(int ch, FILE *f) {
    (void)f;
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

void recv_task_fun(void *argument) {
    uint32_t print_div = 0;

    for (;;) {
        current_angle = ((uint16_t)rx_data[0] << 8) | rx_data[1];
        current_speed = ((uint16_t)rx_data[2] << 8) | rx_data[3];
        current_torque = ((uint16_t)rx_data[4] << 8) | rx_data[5];

        if (++print_div >= 10) {
            print_div = 0;
            printf("rx_ang=%u/8191 rx_spd=%drpm tgt_ang=%.1f tgt_spd=%.1frpm\r\n",
                current_angle,
                (int16_t)current_speed,
                target_angle_dbg,
                target_speed_dbg);
        }

        osDelay(10);
    }
}