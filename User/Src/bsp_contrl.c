//
// Created by zzz on 2026/11/21.
//
#include"stdint.h"
#include"stm32h7xx_hal.h"
#include"../../User/Inc/bsp_contrl.h"
#include"usart.h"
#include"FreeRTOS.h"
#include"task.h"
uint8_t buf[18] = {0};
uint8_t mem_buf[18] = {0};
uint16_t sum = 0;
CONTRL_DATA contrl_data;

void contrl_init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5,(uint8_t*)buf,18);
}
void contrl_math(uint8_t* buf,uint16_t size) {
        contrl_data. channel_0 = ((int16_t)(buf[0] | (int16_t)buf[1] << 8) & 0x07FF)-1024;
        contrl_data. channel_1 = ((int16_t)(buf[1]>>3 | (int16_t)(buf[2]<<5)) & 0x07FF)-1024;
        contrl_data. channel_2 = (((int16_t)(buf[2]>>6) | (int16_t)(buf[3]<<2) | (int16_t)(buf[4]<<10)) & 0x07FF)-1024;
        contrl_data. channel_3 = (((int16_t)(buf[4]>>1) | (int16_t)(buf[5]<<7) ) & 0x07FF)-1024;
        contrl_data.s1 = ((buf[5]>>4)&0x000C) >> 2;
        contrl_data.s2 = (buf[5]>>4)&0x0003;
        contrl_data.time = xTaskGetTickCount();
    }

