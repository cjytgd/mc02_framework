//
// Created by zzz on 2026/11/14.
//
#include"../../User/Inc/bsp_can.h"
#include"../../User/Inc/bsp_motor.h"
extern uint8_t rx_data1[8] ;
RX_DAtA rx_data;
void motor_send(void) {
    rx_data.me_angle =  rx_data1[0]>>8 | rx_data1[1]&0xFF;
    rx_data.speed = rx_data1[2]>>8 | rx_data1[3]&0xFF;
    rx_data.current = rx_data1[4]>>8 | rx_data1[5]&0xFF;
    rx_data.temperature = rx_data1[6];
}
void motor_set_I(ID_data id,uint16_t I) {
    if (I > 16000){I = 16000;}
    if (I < -16000){I = -16000;}
    uint8_t tx_data_1[8] ={0};
    uint8_t tx_data_2[8] ={0};
    switch (id) {
        case id1:
            tx_data_1[0] = I >> 8;
            tx_data_1[1] = I & 0xFF;
            fdcanx_send_data(&hfdcan1,0x1FE,tx_data_1,sizeof(tx_data_1));
            break;
        case id2:
            tx_data_1[2] = I >> 8;
            tx_data_1[3] = I & 0xFF;
            fdcanx_send_data(&hfdcan1,0x1FE,tx_data_1,sizeof(tx_data_1));
            break;
        case id3:
            tx_data_1[4] = I >> 8;
            tx_data_1[5] = I & 0xFF;
            fdcanx_send_data(&hfdcan1,0x1FE,tx_data_1,sizeof(tx_data_1));
            break;
        case id4:
            tx_data_1[6] = I >> 8;
            tx_data_1[7] = I & 0xFF;
            fdcanx_send_data(&hfdcan1,0x1FE,tx_data_1,sizeof(tx_data_1));
            break;
        case id5:
            tx_data_2[0] = I >> 8;
            tx_data_2[1] = I & 0xFF;
            fdcanx_send_data(&hfdcan1,0x2FE,tx_data_2,sizeof(tx_data_2));
            break;
        case id6:
            tx_data_2[2] = I >> 8;
            tx_data_2[3] = I & 0xFF;
            fdcanx_send_data(&hfdcan1,0x2FE,tx_data_2,sizeof(tx_data_2));
            break;
        case id7:
            tx_data_2[4] = I >> 8;
            tx_data_2[5] = I & 0xFF;
            fdcanx_send_data(&hfdcan1,0x2FE,tx_data_2,sizeof(tx_data_2));
            break;
    };
};

