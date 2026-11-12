//
// Created by zzz on 2026/11/7.
//
#include "main.h"
#include"../../User/Inc/bsp_led.h"
#include"../../User/Inc/bsp_ws2812.h"
#include"math.h"
#define FLASH_FRG 500
led_state mode = init;
uint32_t time = 0;
uint32_t now_time = 0;
uint8_t ba_r = 0;
uint8_t ba_g = 0;
uint8_t ba_b = 0;
void bsp_led_init() {
    WS2812_Ctrl(0,0,0);
}
//决定状态模式
void bsp_led_set(led_state state,uint8_t r,uint8_t g,uint8_t b) {
    mode = state;
    ba_r = r;
    ba_g = g;
    ba_b = b;
}
//进行呼吸灯相关函数计算
void bsp_led_task(uint32_t duration){
    uint32_t now = HAL_GetTick();
    switch (mode) {
        case init:
            WS2812_Ctrl(0,0,0);
            break;
        case normal: {
            float recycle = (now %  duration) * ( 2 * 3.14f) / duration;
            float duty =  (sin(recycle) + 1.0f) / 2.0f;
            uint8_t red = ba_r * duty;
            uint8_t green = ba_g * duty;
            uint8_t blue = ba_b * duty;
            WS2812_Ctrl(red,green,blue);
            break;
            case error: {
               static uint32_t last_flash_time = 0;
                static uint8_t flashing = 0;
                if (HAL_GetTick() - last_flash_time > FLASH_FRG) {
                    last_flash_time = HAL_GetTick();
                    flashing = !flashing;
                }
                if (flashing) {
                    WS2812_Ctrl(250,0,0);
                }
                else {
                    WS2812_Ctrl(0,0,0);
                }

                }



            }
        }
    }

