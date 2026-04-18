#ifndef __VT13_H
#define __VT13_H
#include "main.h"
#include "stdbool.h"
#pragma pack(1) // 设置结构体对齐方式为 1 字节对齐
typedef struct
{
    uint8_t SOF[2];     // 数据帧，固定值为 0xA9 0x53
    int16_t Channel_0;  // 通道0
    int16_t Channel_1;  // 通道1
    int16_t Channel_2;  // 通道2
    int16_t Channel_3;  // 通道3
    uint8_t switch_cns; // 挡位切换开关
    uint8_t key_pause;     // 暂停按键
    uint8_t key_left;      // 自定义按键（左）
    uint8_t key_right;     // 自定义按键（右）
    int16_t dial;       // 拨轮
    uint8_t trigger;       // 扳机
    int16_t mouse_x;    // 鼠标左右移动速度
    int16_t mouse_y;    // 鼠标前后移动速度
    int16_t mouse_z;    // 鼠标滚轮移动速度
    int8_t left_button_down;    //鼠标左键
    int8_t right_button_down;   //鼠标右键
    int8_t middle_button_down;  //鼠标中键
    uint16_t keyboard_value; // 键盘值
    uint16_t CRC_606;            // crc校验
} VT13_info_t;

typedef struct
{
    uint8_t SOF;
    uint8_t data_length[2];
    uint8_t seq;
    uint8_t CRC_8;
    uint8_t cmd_id[2];
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
    uint16_t CRC_16; 
}remote_control_t;

typedef struct
{
    uint8_t data[30];  //自定义控制器
} custom_robot_data_t;
#pragma pack()
extern void VT13_init(void);
void VT13_HANDLER(uint8_t* VT13_buffer);
void REMOTE_HANDLE(uint8_t* tc_buffer);
const VT13_info_t *get_VT13_info(void);
const remote_control_t *get_remote_control(void);
#endif /* __VT13_H */
