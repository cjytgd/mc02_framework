//
// Created by zzz on 2026/11/7.
//

#ifndef MC02_1_BSP_LED_H
#define MC02_1_BSP_LED_H
#include"bsp_ws2812.h"
typedef enum {
    init,//表示未工作
    normal,//状态正常用红灯呼吸表示
    error,//状态不正常用红灯快闪表示
}led_state;
void bsp_led_init(void);
void bsp_led_set(led_state state,uint8_t r,uint8_t g,uint8_t b);
void bsp_led_task(uint32_t duration);
#endif //MC02_1_BSP_LED_H
