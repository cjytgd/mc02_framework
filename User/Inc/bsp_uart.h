//
// Created by zzz on 2026/11/4.
//

#ifndef MC02_1_BSP_UART_H
#define MC02_1_BSP_UART_H
#include "stm32h7xx_hal.h"
void uart_init(UART_HandleTypeDef * huart);//开启串口和空闲中断
void uart_start(const char*information,...);
#endif //MC02_1_BSP_UART_H
