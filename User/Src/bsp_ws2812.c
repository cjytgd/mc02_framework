//
// Created by zzz on 2026/11/3.
//
#include "../../User/Inc/bsp_ws2812.h"
#include "stdint.h"

#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xF0     // 1码 (绝对不能用 F0)

static uint8_t txbuf[24];

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < 8; i++)
    {
        txbuf[7 - i]  = (((g >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[15 - i] = (((r >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        txbuf[23 - i] = (((b >> i) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
    }


    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, HAL_MAX_DELAY);

    uint8_t reset_buf[64] = {0};
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, reset_buf, 64, 100);
}