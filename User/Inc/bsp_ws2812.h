//
// Created by zzz on 2026/11/3.
//

#ifndef MC02_1_WS2812_H
#define MC02_1_WS2812_H
#ifndef __WS2812_H__
#define __WS2812_H__


#include "main.h"


#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);
#endif
#endif //MC02_1_WS2812_H
