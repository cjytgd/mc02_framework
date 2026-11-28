//
// Created by zzz on 2026/11/21.
//
#include"stdint.h"
#include"stm32h7xx_hal.h"
#include"../../User/Inc/bsp_contrl.h"
#include"usart.h"
#include"FreeRTOS.h"
#include"task.h"
#define CONTRL_MIN_WIDE 10
#define CONTRL_MAX_WAITING 100
uint8_t buf[18] = {0};
uint8_t mem_buf[18] = {0};
uint16_t sum = 0;
int16_t contrl_buf[4] = {0};
CONTRL_DATA contrl_data;
CONTROLL_STATE  contrl_state = Error ;
void contrl_init(void) {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5,(uint8_t*)buf,18);
}
void contrl_math(uint8_t* buf,uint16_t size) {
        contrl_data.ch[0] = ((int16_t)(buf[0] | (int16_t)buf[1] << 8) & 0x07FF)-1024;
        contrl_data. ch[1] = ((int16_t)(buf[1]>>3 | (int16_t)(buf[2]<<5)) & 0x07FF)-1024;
        contrl_data. ch[2] = (((int16_t)(buf[2]>>6) | (int16_t)(buf[3]<<2) | (int16_t)(buf[4]<<10)) & 0x07FF)-1024;
        contrl_data. ch[3] = (((int16_t)(buf[4]>>1) | (int16_t)(buf[5]<<7) ) & 0x07FF)-1024;
        contrl_data.s1 = ((buf[5]>>4)&0x000C) >> 2;
        contrl_data.s2 = (buf[5]>>4)&0x0003;
        contrl_data.time = xTaskGetTickCount();
    }
void contrl_check(void)
{
    if (xTaskGetTickCount()-contrl_data.time>CONTRL_MIN_WIDE)
    {
        contrl_state = Offline;
    }
}
int16_t contrl_getch(int16_t channel)
{
    if (contrl_data.ch[channel] < CONTRL_MIN_WIDE && contrl_data.ch[channel] > -CONTRL_MIN_WIDE )
{
    return 0;
}
    buf[channel] = contrl_data.ch[channel];
    return buf[channel];
}
void contrl_update(void)
{
    switch(contrl_data.s1)
    {
    case 3:
        if (contrl_data.s2 == 3)
        {//进行pid计算
            contrl_state = Normal;
        }
        else  if (contrl_data.s2 == 2)
        {//进行小陀螺模式
            contrl_state = spin;
        }
        else if (contrl_data.s2 == 1)
        {//进行自瞄
            contrl_state = Auto;
        }
        break;
    case 1:
    case 2:
        contrl_state = Error;
        break;;
    }
}

