//
// Created by zzz on 2026/11/15.
//

#ifndef MC02_1_BSP_PID_H
#define MC02_1_BSP_PID_H
#include"stdint.h"
typedef struct {
    float kp;
    float ki;
    float kd;
    float current;
    float target;
    float duty;
    float re_duty;
}PID;
extern PID pid;
extern int32_t ta_error;
void pid_set( float kp, float ki, float kd);
float pid_math(float target,float current,float dt,float pro);

#endif //MC02_1_BSP_PID_H
