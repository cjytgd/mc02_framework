//
// Created by zzz on 2026/11/14.
//
#include"../../User/Inc/bsp_can.h"
void motor_set_I(uint16_t I) {
    uint8_t tx_data[8] ={0};
    if (I > 16000){I = 16000;}
    if (I < -16000){I = -16000;}
    tx_data[0] = I >> 8;
    tx_data[1] = I & 0xFF;
    fdcanx_send_data(&hfdcan1,0x1FE,tx_data,sizeof(tx_data));
}
