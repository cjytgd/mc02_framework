//
// Created by zzz on 2026/11/14.
//
#include"../../User/Inc/bsp_can.h"
#include"../../User/Inc/bsp_motor.h"
#include  "fdcan.h"
#include "FreeRTOS.h"
#include"task.h"
int16_t gm6020_m2006_1_4[5] = {0};
int16_t gm6020_m2006_5_7[5] = {0};
int16_t m3508_1_4[5] = {0};
int16_t m3508_5_8[5] = {0};
RX_DATA rx_data;
void motor_set_current(Type type,uint8_t id,float current)
{
    if (id < 0 || id > 8)
    {
        return;
    }
    switch(type)
    {
    case GM6020:
    case M2006:
        if (id < 5)
        {
            gm6020_m2006_1_4[id] = (int16_t)current;
        }

        if (id >= 5)
        {
            gm6020_m2006_5_7[id - 4] = (int16_t)current;
        }
        break;
    case M3508:
        if (id < 5)
        {
            m3508_1_4[id] = (int16_t)current;
        }
        if (id >= 5)
        {
            m3508_5_8[id - 4] = (int16_t)current;
        }
        break;
    case DM_J4310:
        uint8_t tx_buf[8] = {0};
        tx_buf[0] = ((int16_t)current >>8) & 0xFF;
        tx_buf[1] = ((int16_t)current) & 0xFF;
        fdcanx_send_data(&hfdcan1,id,tx_buf,8);
        break;;

    }
}
void motor_send_current()
{
    uint8_t buf[8];
    for (uint8_t i = 0; i <= 4;i++)
    {
        buf[(i-1) * 2] = (gm6020_m2006_1_4[i]>>8) & 0xFF;
        buf[(i-1) * 2 + 1] = gm6020_m2006_1_4[i] & 0xFF;
    }
    fdcanx_send_data(&hfdcan1,0x1FE,buf,8);
    for (uint8_t i = 0; i <= 3;i++)
    {
        buf[(i-1) * 2] = (gm6020_m2006_5_7[i]>>8) & 0xFF;
        buf[(i-1) * 2 + 1] = gm6020_m2006_5_7[i] & 0xFF;
        buf[6] = 0;
        buf[7] = 0;
    }
    fdcanx_send_data(&hfdcan1,0x2FE,buf,8);
    for (uint8_t i = 0; i <= 4;i++)
    {
        buf[(i-1) * 2] = (m3508_1_4[i]>>8) & 0xFF;
        buf[(i-1) * 2 + 1] = m3508_1_4[i]&0xFF;
    }
    fdcanx_send_data(&hfdcan1,0x200,buf,8);
    for (uint8_t i = 0; i <= 4;i++)
    {
        buf[(i-1) * 2] = (m3508_5_8[i]>>8) & 0xFF;
        buf[(i-1) * 2 + 1] = m3508_5_8[i] & 0xFF;
    }
    fdcanx_send_data(&hfdcan1,0x1FF,buf,8);
}
void motor_prase_can(uint32_t rec_id,uint8_t*Rx_data)
{
    rx_data.me_angle =  (int16_t)(Rx_data[0]<<8 | Rx_data[1]);
    // if (rx_data.me_angle > 8000){count++;}
    rx_data.speed = Rx_data[2]<<8 | Rx_data[3]&0xFF;
    rx_data.current = Rx_data[4]<<8 | Rx_data[5]&0xFF;
    rx_data.temperature = Rx_data[6];
    rx_data.time = xTaskGetTickCount();
}
