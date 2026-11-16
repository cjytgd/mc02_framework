//
// Created by zzz on 2026/11/15.
//
#include"../../User/Inc/bsp_pid.h"
#include"../../User/Inc/bsp_can.h"
#include"../../User/Inc/bsp_motor.h"
#include"math.h"
PID pid;
int32_t ta_error = 0;
static int16_t error = 0;
static int16_t last_error = 0;
void pid_set( float kp, float ki, float kd) {
    pid.kp = kp;
    pid.ki = ki;
    pid.kd = kd;
}
float pid_math(float target,float current,float dt) {
    pid.target = target;
    pid.current = current;
    error = target-current;
    ta_error = ta_error + error * dt;
    if(ta_error > 5000){ ta_error = 5000;}
    if(ta_error < -5000){ ta_error = -5000;}
    pid.duty = pid.kp * error + pid.ki * ta_error + pid.kd * (error - last_error) / dt ;
    if (pid.duty > 16000){ pid.duty = 16000; }
    if (pid.duty < -16000){ pid.duty = -16000; }
    last_error = error;
    return (pid.duty);
}

