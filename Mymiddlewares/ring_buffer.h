#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_
#include "main.h"
#include "string.h"
typedef struct
{
    uint16_t writeIndex; // 循环缓冲区写索引
    uint16_t readIndex;  // 循环缓冲区读索引
} ring_buffer_t;

uint8_t Write_Buffer(uint8_t *data, uint8_t length);
uint8_t Read_Buffer(uint8_t *data);
#endif
