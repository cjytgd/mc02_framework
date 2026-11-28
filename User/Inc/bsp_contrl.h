//
// Created by zzz on 2026/11/21.
//
#include"stdint.h"
#ifndef MC02_1_BSP_CONTRL_H
#define MC02_1_BSP_CONTRL_H

typedef struct {
    int16_t ch[4];
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
    Offline,
}CONTROLL_STATE;
extern CONTROLL_STATE contrl_state;
extern uint8_t buf[18];
extern uint8_t mem_buf[18] ;
void contrl_math(uint8_t* buf,uint16_t size);
void contrl_init(void);
void contrl_update(void);
void contrl_check(void);
int16_t contrl_getch(int16_t channel);
#endif //MC02_1_BSP_CONTRL_H
