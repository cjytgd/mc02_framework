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
    float ta_error;
    float error;
    float last_error;
    float last_duty;
    float target_speed;
}PID;
extern PID pid_angle;
extern PID pid_speed;
void pid_set_angle( float kp, float ki, float kd);
void pid_set_speed( float kp, float ki, float kd);
float pid_math_speed(float target,float current,float dt,float pro);
float pid_math_angle(float target,float current,float dt,float pro );

#endif //MC02_1_BSP_PID_H
