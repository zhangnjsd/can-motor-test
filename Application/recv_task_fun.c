#include "recv_task_fun.h"
#include <math.h>

struct dt7_data_t dt7_data;

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

    dbus_uart_init();

    for (;;) {
        // * Get data from CAN.
        current_angle = ((uint16_t)rx_data[0] << 8) | rx_data[1];
        current_speed = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
        current_torque = ((uint16_t)rx_data[4] << 8) | rx_data[5];

        // * Get data from DT7.
		dt7_data.channel0 = (int16_t)(((dbus_buf[0] | (dbus_buf[1] << 8)) & 0x07FF) - 1024);
		dt7_data.channel1 = (int16_t)((((dbus_buf[1] >> 3) | (dbus_buf[2] << 5)) & 0x07FF) - 1024);
		dt7_data.channel2 = (int16_t)((((dbus_buf[2] >> 6) | (dbus_buf[3] << 2) | (dbus_buf[4] << 10)) & 0x07FF) - 1024);
		dt7_data.channel3 = (int16_t)((((dbus_buf[4] >> 1) | (dbus_buf[5] << 7)) & 0x07FF) - 1024);

		dt7_data.switch1 = (uint8_t)((dbus_buf[5] >> 4) & 0x03);
		dt7_data.switch2 = (uint8_t)((dbus_buf[5] >> 6) & 0x03);

		dt7_data.mouse_x = (int16_t)(dbus_buf[6]  | (dbus_buf[7]  << 8));
		dt7_data.mouse_y = (int16_t)(dbus_buf[8]  | (dbus_buf[9]  << 8));
		dt7_data.mouse_z = (int16_t)(dbus_buf[10] | (dbus_buf[11] << 8));

		dt7_data.mouse_left = (uint8_t)dbus_buf[12];
		dt7_data.mouse_right = (uint8_t)dbus_buf[13];

		dt7_data.key_code = (uint16_t)(dbus_buf[14] | (dbus_buf[15] << 8));

        // * Set and send controller.
        volt[1] = 0;
        volt[2] = 0;
        volt[3] = 0;
        CAN_Send(&hcan1, volt, CAN_ID);

        // ? Output Freq = 100Hz
        if (++print_div >= 10) {
            print_div = 0;
            // Bias.
            printf("stream:%f,%f,%f,%f\n",
                (float_t)current_angle, (float_t)target_angle_dbg, 
                (float_t)current_speed * 8, (float_t)target_speed_dbg);
        }

        
        // ? Req&Set Freq = 1000Hz
        next_wake_tick += 1;
        osDelayUntil(next_wake_tick);
    }
}


