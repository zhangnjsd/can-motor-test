/*API库（循环缓冲区）
功能设定：1.写入缓存区，写索引偏移并判断是否可以写入
2.读缓存区，数据处理完偏移
*/
#include "ring_buffer.h"
#include "main.h"
#include "VT13.h"
// 循环缓冲区大小
#define BUFFER_SIZE 500
// 循环缓冲区
uint8_t buffer[BUFFER_SIZE];
ring_buffer_t ring_buffer;
uint8_t Priority_VT13; // 优先级 0:VT13 1:自定义控制器 2:图传链路

// 向缓存区写入数据
uint8_t Write_Buffer(uint8_t *data, uint8_t length)
{
    if ((BUFFER_SIZE - (ring_buffer.writeIndex - ring_buffer.readIndex + BUFFER_SIZE) % BUFFER_SIZE) < length)
    {
        return 0; // 缓冲区剩余空间不足
    }
    if (ring_buffer.writeIndex + length < BUFFER_SIZE)
    {
        memmove(buffer + ring_buffer.writeIndex, data, length); // 写入数据
        ring_buffer.writeIndex = ring_buffer.writeIndex + length; // 写索引偏移
    }
    else
    {
        uint8_t first_part = BUFFER_SIZE - ring_buffer.writeIndex; // 计算第一部分的长度
        memmove(buffer + ring_buffer.writeIndex, data, first_part); // 写入第一部分数据
        memmove(buffer, data + first_part, length - first_part);    // 写入第二部分数据
        ring_buffer.writeIndex = length - first_part; // 更新写索引
    }
		return 1;
}
// 从缓存区读取数据
uint8_t Read_Buffer(uint8_t *data)
{
    static uint16_t no_found = 0; // 记录未找到数据的次数
    uint8_t length = 0;
    if ((ring_buffer.writeIndex - ring_buffer.readIndex + BUFFER_SIZE) % BUFFER_SIZE < 22)
    {
        return 0; // 缓冲区数据不足
    }
    if ((buffer[ring_buffer.readIndex] != 0xA9) || (buffer[ring_buffer.readIndex + 1] != 0x53)) // 判断数据帧头
    {
        ring_buffer.readIndex = (ring_buffer.readIndex + 1) % BUFFER_SIZE; // 更新读索引
        no_found++;                                                        // 未找到数据次数加1
        if (no_found > 400)
            no_found = 400;
    }
    if (buffer[ring_buffer.readIndex] == 0xA9 && buffer[ring_buffer.readIndex + 1] == 0x53)
    {
        no_found = 0;
        Priority_VT13 = 0;
        length = 21; // 读取数据长度
    }
    if (buffer[ring_buffer.readIndex] == 0xA5 && buffer[ring_buffer.readIndex + 1] == 0x0C && no_found >= 400)
    {
        length = 21; // 读取数据长度
        Priority_VT13 = 2;
    }
    if (buffer[ring_buffer.readIndex] == 0xA5 && buffer[ring_buffer.readIndex + 1] == 0x1E)
    {
        length = 30; // 读取数据长度
        Priority_VT13 = 1; // 自定义控制器
    }
    if (length > 0)
    {
        if (ring_buffer.readIndex + length < BUFFER_SIZE)
        {
            memmove(data, buffer + ring_buffer.readIndex, length);   // 读取数据
            ring_buffer.readIndex = ring_buffer.readIndex + length; // 读索引偏移
        }
        else if (ring_buffer.readIndex + length >= BUFFER_SIZE)
        {
            uint8_t first_part = BUFFER_SIZE - ring_buffer.readIndex; // 计算第一部分的长度
            memmove(data, buffer + ring_buffer.readIndex, first_part); // 读取第一部分数据
            memmove(data + first_part, buffer, length - first_part);   // 读取第二部分数据
            ring_buffer.readIndex = length - first_part;              // 更新读索引
        }
        length = 0; // 清零数据长度
    }
    if (Priority_VT13 == 0)
    {
        VT13_HANDLER(data); // 处理VT13数据
    }
    else if (Priority_VT13 == 2)
    {
      REMOTE_HANDLE(data); // 处理图传链路数据               
    }
    else
    {
        // 处理自定义控制器数据
    }
		return 1;
}
