#include "dt7_recv_task_fun.h"
#include "main.h"
#include "bsp_dt7.h"
#include "cmsis_os.h"

dt7_data_t dt7_data;

void dt7_recv_task_fun(void const * argument)
{
	dbus_uart_init();
	for(;;)
	{ 
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
		osDelay(1);
	}
}
