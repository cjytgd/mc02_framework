//
// Created by zzz on 2026/11/21.
//
#include"stdint.h"
#ifndef MC02_1_BSP_CONTRL_H
#define MC02_1_BSP_CONTRL_H

typedef struct {
    int16_t channel_1;
    int16_t channel_2;
    int16_t channel_3;
    int16_t channel_0;
    uint32_t time;
    uint8_t s1;
    uint8_t s2;
}CONTRL_DATA;
extern CONTRL_DATA contrl_data;
typedef enum
{
    Normal,
    Error,
    spin,
    Auto,
}CONTROLL_STATE;
extern CONTROLL_STATE contrl_state;
extern uint8_t buf[18];
extern uint8_t mem_buf[18] ;
void contrl_math(uint8_t* buf,uint16_t size);
void contrl_init(void);
void contrl_update(void);
#endif //MC02_1_BSP_CONTRL_H
