//
// Created by zzz on 2026/11/3.
//
#include "../../User/Inc/bsp_ws2812.h"
#include "stdint.h"

#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xF8     // 1码 (绝对不能用 F0)

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[24];
    uint8_t res = 0;

    // 1. 拼装数据
    for (int i = 0; i < 8; i++)
    {
        txbuf[7 - i]  = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }

    // 2. 准备发送 (拉低CS)
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 100);

    // 3. 发送 24 字节 RGB 数据
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 100);

    // 4. 发送复位信号 (64字节 = 80us 低电平)
    uint8_t reset_buf[64] = {0};
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, reset_buf, 64, 100);
}