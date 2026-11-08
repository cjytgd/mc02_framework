//
// Created by zzz on 2026/11/4.
//
#include "main.h"
#include"../../User/Inc/bsp_uart.h"
#include<stdarg.h>
#include"stdio.h"
#include"string.h"
#include"usart.h"
#include"../../User/Inc/bsp_buzzer.h"
#include"../../User/Inc/bsp_led.h"
#define uart huart7
#define UART UART7
char re_buf[128];
char tx_buf[128];
static volatile uint8_t busy = 0;
void uart_init(UART_HandleTypeDef * huart) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t*)re_buf,sizeof(re_buf));
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if(huart->Instance == UART) {
       // HAL_UART_Transmit_DMA(huart,(uint8_t*)re_buf,Size);
        //buzzer_start(1000,1000);
         if (re_buf[0] == '1'){ bsp_led_set(normal,250,0,0);};//发1表示 正常
         if (re_buf[0] == '2'){ bsp_led_set(error,250,0,0);}//发2表示错误
         if (re_buf[0] == '3'){ bsp_led_set(init,0,0,0);}//发3表示未工作
       HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t*)re_buf,sizeof(re_buf));
    }

}
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) {
//     if(huart->Instance == UART) {
//         HAL_UART_Transmit(&uart,(uint8_t*)re_buf,strlen((char*)re_buf),1000);
//         HAL_UART_Receive_IT(&uart,(uint8_t*)re_buf,1);
//     }
//
// }
// void HAL_UART_TxCpltCallback(UART_HandleTypeDef * huart) {
//     if(huart->Instance == UART) {
//         busy = 0;
//     }
// }

void uart_start(const char*information, ...) {
    strncpy(tx_buf,information,sizeof(tx_buf)-1);
    tx_buf[sizeof(tx_buf)-1] = '\0';
    // HAL_UART_Transmit_DMA(&uart,(uint8_t*)information,strlen(information));
    HAL_UART_Transmit_DMA(&uart, (uint8_t *) tx_buf, strlen(information));
}

    