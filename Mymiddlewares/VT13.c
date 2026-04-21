/*
使用说明：一个串口
编码格式：UTF-8
TODO：遥控方式的自动切换
方法：自行编写一个dt7离线的方法，离线后切换至图传两个遥控模式（自带优先级）
*/
#include "VT13.h"
#include "crc.h"
#include "ring_buffer.h"

#define VT13_UART huart6 // 串口接口
#define OFFSET 1024      // 偏移量
extern UART_HandleTypeDef VT13_UART;
extern DMA_HandleTypeDef hdma_usart6_rx;
static VT13_info_t VT13_info = {0};           // VT13数据结构体,static修饰符表示保证了此变量仅内部可见和访问
static remote_control_t remote_control = {0}; // 数据结构体
uint8_t VT13_buffer[70];                      // VT13接收数据缓存区
bool VT13_Flag = false;                       // VT13可信度标志

void VT13_HANDLER(uint8_t *VT13_buffer)
{
    // 处理VT13数据
    // 这里可以添加VT13数据处理的代码
    uint8_t VT13_tx_buf[21];
    memmove(VT13_tx_buf, VT13_buffer, 21);                // 将接收到的数据复制到VT13_tx_buf
    VT13_Flag = Verify_CRC16_Check_Sum(VT13_tx_buf, 21); // 验证CRC16
    if (VT13_Flag == true)
    {
        VT13_info.SOF[0] = VT13_tx_buf[0]; // 数据帧头
        VT13_info.SOF[1] = VT13_tx_buf[1]; // 数据帧头
        // 处理VT13数据
        // 这里可以添加VT13数据处理的代码
        VT13_info.Channel_0 = VT13_tx_buf[2] | ((VT13_tx_buf[3] & 0x07) << 8);                                 // 通道0
        VT13_info.Channel_1 = (VT13_tx_buf[3] >> 3) | ((VT13_tx_buf[4] & 0x3F) << 5);                          // 通道1
        VT13_info.Channel_2 = (VT13_tx_buf[4] >> 6) | (VT13_tx_buf[5] << 2) | ((VT13_tx_buf[6] & 0x01) << 10); // 通道2
        VT13_info.Channel_3 = (VT13_tx_buf[6] >> 1) | ((VT13_tx_buf[7] & 0x0F) << 7);                          // 通道3
        VT13_info.switch_cns = (VT13_tx_buf[7] >> 4) & 0x03;                                                   // 挡位切换开关
        VT13_info.key_pause = (VT13_tx_buf[7] >> 6) & 0x01;                                                    // 暂停按键
        VT13_info.key_left = (VT13_tx_buf[7] >> 7) & 0x01;                                                     // 自定义按键（左）
        VT13_info.key_right = (VT13_tx_buf[8] >> 0) & 0x01;                                                    // 自定义按键（右）
        VT13_info.dial = (VT13_tx_buf[8] >> 1) | ((VT13_tx_buf[9] & 0x0F) << 7);                               // 拨轮
        VT13_info.trigger = (VT13_tx_buf[9] >> 4) & 0x01;                                                      // 扳机
        VT13_info.mouse_x = VT13_tx_buf[10] | (VT13_tx_buf[11] << 8);                                          // 鼠标X轴
        VT13_info.mouse_y = VT13_tx_buf[12] | (VT13_tx_buf[13] << 8);                                          // 鼠标Y轴
        VT13_info.mouse_z = VT13_tx_buf[14] | (VT13_tx_buf[15] << 8);                                          // 鼠标Z轴
        VT13_info.left_button_down = (VT13_tx_buf[16] >> 0) & 0x03;                                            // 鼠标左键
        VT13_info.right_button_down = (VT13_tx_buf[16] >> 2) & 0x03;                                           // 鼠标右键
        VT13_info.middle_button_down = (VT13_tx_buf[16] >> 4) & 0x03;                                          // 鼠标中键
        VT13_info.keyboard_value = VT13_tx_buf[17] | (VT13_tx_buf[18] << 8);                                   // 键盘值
        VT13_info.CRC_606 = VT13_tx_buf[19] | (VT13_tx_buf[20] << 8);                                          // 保留
        VT13_info.Channel_0 -= OFFSET;                                                                         // 通道0
        VT13_info.Channel_1 -= OFFSET;                                                                         // 通道1
        VT13_info.Channel_2 -= OFFSET;                                                                         // 通道2
        VT13_info.Channel_3 -= OFFSET;                                                                         // 通道3
    }
}
void REMOTE_HANDLE(uint8_t *tc_buffer)
{
    uint8_t tc_tx_buf[21];
    memmove(tc_tx_buf, tc_buffer, 21);                                     // 将接收到的数据复制到tc_tx_buf
    remote_control.SOF = tc_tx_buf[0];                                    // 数据帧头
    remote_control.data_length[0] = tc_tx_buf[1];                         // 数据长度
    remote_control.data_length[1] = tc_tx_buf[2];                         // 数据长度
    remote_control.seq = tc_tx_buf[3];                                    // 序列号
    remote_control.CRC_8 = tc_tx_buf[4];                                  // CRC8校验
    remote_control.cmd_id[0] = tc_tx_buf[5];                              // 命令ID
    remote_control.cmd_id[1] = tc_tx_buf[6];                              // 命令ID
    VT13_info.mouse_x = tc_tx_buf[7] | (tc_tx_buf[8] << 8);          // 鼠标X轴
    VT13_info.mouse_y = tc_tx_buf[9] | (tc_tx_buf[10] << 8);         // 鼠标Y轴
    VT13_info.mouse_z = tc_tx_buf[11] | (tc_tx_buf[12] << 8);        // 鼠标Z轴
    VT13_info.left_button_down = tc_tx_buf[13];                      // 鼠标左键
    VT13_info.right_button_down = tc_tx_buf[14];                     // 鼠标右键
    VT13_info.keyboard_value = tc_tx_buf[15] | (tc_tx_buf[16] << 8);      // 键盘值
    remote_control.reserved = tc_tx_buf[17] | (tc_tx_buf[18] << 8);       // 保留
    remote_control.CRC_16 = tc_tx_buf[19] | (tc_tx_buf[20] << 8);         // CRC16校验
}

// 该函数用于获取VT13数据结构体的指针，方便外部访问VT13数据
const VT13_info_t *get_VT13_info(void) // 获取VT13数据函数
{
    return &VT13_info; // 返回VT13数据结构体指针
}

const remote_control_t *get_remote_control(void) // 获取遥控器数据函数
{
    return &remote_control; // 返回遥控器数据结构体指针
}

void VT13_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&VT13_UART, VT13_buffer, 70); // 启动DMA接收
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == VT13_UART.Instance)
    {
       Write_Buffer(VT13_buffer, Size);                           // 将接收到的数据写入环形缓冲区
			 
       HAL_UARTEx_ReceiveToIdle_DMA(&VT13_UART, VT13_buffer, 70); // 启动DMA接收
    }
}
