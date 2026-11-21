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
#include"../../User/Inc/bsp_pid.h"
#define uart huart7
#define UART UART7
char re_buf[128];
char tx_buf[128];
uint8_t motor_set_uart = 1;
static volatile uint8_t busy = 0;
uint8_t i = 1;
uint16_t speed = 0;
void uart_init(UART_HandleTypeDef * huart) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t*)re_buf,sizeof(re_buf));
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if(huart->Instance == UART) {
        //数据溢出进行闪烁红灯警报
        if (Size >= sizeof(re_buf)) {
            bsp_led_set(error,250,0,0);
        }else{
            re_buf[Size] = '\0';//强赛一个结束符
            //大写为速度，小写为角度
            if (re_buf[0] == 'p') {
                while (re_buf[i]>='0' && re_buf[i]<='9') {
                    speed=speed*10+re_buf[i]-'0';//将ASCLL码的字符数字转化成真实数字，存入内存进行PID
                    i++;
                    pid_set_angle(speed,pid_angle.ki,pid_angle.kd);}
            }
                else if(re_buf[0] == 'i') {
                    while (re_buf[i]>='0' && re_buf[i]<='9') {
                        speed=speed*10+re_buf[i]-'0';//将ASCLL码的字符数字转化成真实数字，存入内存进行PID
                        i++;
                        pid_set_angle(pid_angle.kp,speed,pid_angle.kd);}
                }
                else if(re_buf[0] == 'd') {
                    while (re_buf[i]>='0' && re_buf[i]<='9') {
                        speed=speed*10+re_buf[i]-'0';//将ASCLL码的字符数字转化成真实数字，存入内存进行PID
                        i++;
                        pid_set_angle(pid_angle.kp,pid_angle.ki,speed);}
                }
            else if(re_buf[0] == 'P') {
                    while (re_buf[i]>='0' && re_buf[i]<='9') {
                        speed=speed*10+re_buf[i]-'0';//将ASCLL码的字符数字转化成真实数字，存入内存进行PID
                        i++;
                        pid_set_speed(speed,pid_speed.ki,pid_speed.kd);}
                }
            else if(re_buf[0] == 'I') {
                while (re_buf[i]>='0' && re_buf[i]<='9') {
                    speed=speed*10+re_buf[i]-'0';//将ASCLL码的字符数字转化成真实数字，存入内存进行PID
                    i++;
                    pid_set_speed(pid_speed.kp,speed,pid_speed.kd);}
            }
            else if(re_buf[0] == 'D') {
                while (re_buf[i]>='0' && re_buf[i]<='9') {
                    speed=speed*10+re_buf[i]-'0';//将ASCLL码的字符数字转化成真实数字，存入内存进行PID
                    i++;
                    pid_set_speed(pid_speed.kp,pid_speed.ki,speed);}
            }

                HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t*)re_buf,sizeof(re_buf));
            }
    }

}
void uart_start(const char*information, ...) {
    int len;
    va_list buf;

    if (busy == 1) {
        return;
    }
    busy = 1;
    va_start(buf,information);
    /* 注意：这里必须用 vsnprintf 的返回值作为发送长度，
       不能用 strlen(information)，那是格式字符串本身（"%.2f,..."）的长度，
       会把格式化后的数据截断！ */
    len = vsnprintf(tx_buf,sizeof(tx_buf),information,buf);
    va_end(buf);

    if (len <= 0) {
        busy = 0;
        return;
    }
    if (len >= (int)sizeof(tx_buf)) {
        len = (int)sizeof(tx_buf) - 1;   /* 超长时按缓冲区实际内容发送 */
    }
    if (HAL_UART_Transmit_DMA(&uart, (uint8_t *) tx_buf, (uint16_t)len) != HAL_OK) {
        busy = 0;                        /* 启动失败要放行，否则串口从此再也发不出数据 */
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART) {
        busy = 0;//保证发完一包数据才能进行下一次发送，防止数据互踩
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART) {
        __HAL_UART_CLEAR_OREFLAG(huart);//清空ore
        HAL_UARTEx_ReceiveToIdle_DMA(huart,(uint8_t*)re_buf,sizeof(re_buf));//重新开始接收
    }
}

