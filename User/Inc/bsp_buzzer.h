//
// Created by zzz on 2026/11/3.
//

#ifndef MC02_1_BUZZER_H
#define MC02_1_BUZZER_H
#include"main.h"
void buzzer_start(uint32_t frg,uint8_t lasting);//lasting代表蜂鸣器响的时间
void buzzer_stop();
void buzzer_init();
void buzzer_task();
#endif //MC02_1_BUZZER_H
