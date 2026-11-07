//
// Created by zzz on 2026/11/3.
//
#include"../../User/Inc/bsp_buzzer.h"
#include"main.h"
#include"tim.h"
#define TIM htim12
#define TIM_CHANNEL TIM_CHANNEL_2
void buzzer_init() {
    buzzer_stop();
}
void buzzer_stop() {
    __HAL_TIM_SET_COMPARE(&TIM, TIM_CHANNEL,10000);
}
void buzzer_start(uint32_t frg) {
    uint32_t arr = 1000000 / frg;
    __HAL_TIM_SET_AUTORELOAD(&TIM,arr);
    __HAL_TIM_SET_COMPARE(&TIM,TIM_CHANNEL,arr/2);
    HAL_TIM_PWM_Start(&TIM, TIM_CHANNEL);

}