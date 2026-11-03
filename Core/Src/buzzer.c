//
// Created by zzz on 2026/11/3.
//
#include"buzzer.h"
#include"main.h"
#include"tim.h"
void buzzer_stop() {
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2,10000);
}

void buzzer_start(uint32_t frg) {
    uint32_t arr = 1000000 / frg;
    __HAL_TIM_SET_AUTORELOAD(&htim12,arr);
    __HAL_TIM_SET_COUNTER(&htim12,arr/2);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);

}