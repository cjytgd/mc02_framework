//
// Created by zzz on 2026/11/14.
//
#include"../../User/Inc/bsp_can.h"
#include"../../User/Inc/bsp_motor.h"
#include  "fdcan.h"
#define GM60501_4  0x1FE
#define GM60505_7  0x2FE
#define M35081_4 0x200
#define M35085_8  0x1FF

extern  float duty;
void motor_set_I(MOTOR_TYPE type,ID_data id,int16_t I) {
    if (I > 16000){I = 16000;}
    if (I < -16000){I = -16000;}
    uint8_t tx_data_1[8] ={0};
    uint8_t tx_data_2[8] ={0};
    switch (type) {
        case GM6020:
        switch (id) {
            case id1:
                tx_data_1[0] = I >> 8;
                tx_data_1[1] = I & 0xFF;
                fdcanx_send_data(&hfdcan1, GM60501_4,tx_data_1,sizeof(tx_data_1));
                break;
            case id2:
                tx_data_1[2] = I >> 8;
                tx_data_1[3] = I & 0xFF;
                fdcanx_send_data(&hfdcan1, GM60501_4,tx_data_1,sizeof(tx_data_1));
                break;
            case id3:
                tx_data_1[4] = I >> 8;
                tx_data_1[5] = I & 0xFF;
                fdcanx_send_data(&hfdcan1, GM60501_4,tx_data_1,sizeof(tx_data_1));
                break;
            case id4:
                tx_data_1[6] = I  >> 8;
                tx_data_1[7] = I & 0xFF;
                fdcanx_send_data(&hfdcan1, GM60501_4,tx_data_1,sizeof(tx_data_1));
                break;
            case id5:
                tx_data_2[0] = I >> 8;
                tx_data_2[1] = I & 0xFF;
                fdcanx_send_data(&hfdcan1, GM60505_7 ,tx_data_2,sizeof(tx_data_2));
                break;
            case id6:
                tx_data_2[2] = I>> 8;
                tx_data_2[3] = I & 0xFF;
                fdcanx_send_data(&hfdcan1, GM60505_7 ,tx_data_2,sizeof(tx_data_2));
                break;
            case id7:
                tx_data_2[4] = I>> 8;
                tx_data_2[5] = I & 0xFF;
                fdcanx_send_data(&hfdcan1, GM60505_7 ,tx_data_2,sizeof(tx_data_2));
                break;
        }
            break;
        case M3508:
            switch (id) {
                case id1:
                    tx_data_1[0] = I>> 8;
                    tx_data_1[1] = I & 0xFF;
                    fdcanx_send_data(&hfdcan1,M35081_4,tx_data_1,sizeof(tx_data_1));
                    break;
                case id2:
                    tx_data_1[2] = I >> 8;
                    tx_data_1[3] = I & 0xFF;
                    fdcanx_send_data(&hfdcan1,M35081_4,tx_data_1,sizeof(tx_data_1));
                    break;
                case id3:
                    tx_data_1[4] = I >> 8;
                    tx_data_1[5] = I & 0xFF;
                    fdcanx_send_data(&hfdcan1,M35081_4,tx_data_1,sizeof(tx_data_1));
                    break;
                case id4:
                    tx_data_1[6] = I>> 8;
                    tx_data_1[7] = I & 0xFF;
                    fdcanx_send_data(&hfdcan1,M35081_4,tx_data_1,sizeof(tx_data_1));
                    break;
                case id5:
                    tx_data_2[0] = I >> 8;
                    tx_data_2[1] = I & 0xFF;
                    fdcanx_send_data(&hfdcan1,M35085_8,tx_data_2,sizeof(tx_data_2));
                    break;
                case id6:
                    tx_data_2[2] = I >> 8;
                    tx_data_2[3] = I & 0xFF;
                    fdcanx_send_data(&hfdcan1,M35085_8,tx_data_2,sizeof(tx_data_2));
                    break;
                case id7:
                    tx_data_2[4] = I>> 8;
                    tx_data_2[5] = I & 0xFF;
                    fdcanx_send_data(&hfdcan1,M35085_8,tx_data_2,sizeof(tx_data_2));
                    break;
                case id8:
                    tx_data_2[6] = I >> 8;
                    tx_data_2[7] = I& 0xFF;
                    fdcanx_send_data(&hfdcan1,M35085_8,tx_data_2,sizeof(tx_data_2));
                    break;
            }
    }
};

