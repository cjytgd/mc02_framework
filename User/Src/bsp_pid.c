//
// Created by zzz on 2026/11/15.
//
#include"../../User/Inc/bsp_pid.h"
#include"../../User/Inc/bsp_can.h"
#include"../../User/Inc/bsp_motor.h"
#include"math.h"
PID pid_angle;
PID pid_speed;

void pid_set_speed( float kp_v, float ki_v, float kd_v) {
     pid_speed.kp = kp_v;
     pid_speed.ki = ki_v;
     pid_speed.kd = kd_v;
}
void pid_set_angle( float kp_a, float ki_a, float kd_a) {
   pid_angle.kp = kp_a;
   pid_angle.ki = ki_a;
   pid_angle.kd = kd_a;
}
float pid_math_speed(float target,float current,float dt,float pro ) {
    pid_speed.target = target;
    pid_speed.current = current;
    pid_speed.error = target-current;
     pid_speed.ta_error = pid_speed.ta_error +  pid_speed.error * dt;
    if(pid_speed.ta_error < -500){ pid_speed.ta_error = -500;}
 if(pid_speed.ta_error > 500){ pid_speed.ta_error = 500;}
    pid_speed.duty =pid_speed.kp *  pid_speed.error +pid_speed.ki *  pid_speed.ta_error +pid_speed.kd * ( pid_speed.error -  pid_speed.last_error) ;
    pid_speed.last_error = pid_speed.error;
    pid_speed.re_duty = pro * pid_speed.duty + (1.0f - pro) *  pid_speed.last_duty;
     pid_speed.last_duty = pid_speed.duty;
    if (pid_speed.re_duty > 16000){ pid_speed.re_duty = 16000; }
    if (pid_speed.re_duty < -16000){pid_speed.re_duty = -16000 ; }

    return (pid_speed.re_duty);
}
float pid_math_angle(float target,float current,float dt,float pro ) {
    pid_angle.target = target;
    pid_angle.current = current;
    pid_angle.error = target-current;
    if (pid_angle.error > 4096.0f){ pid_angle.error -= 8192.0f; }
    if (pid_angle.error < -4096.0f){ pid_angle.error += 8192.0f; }
    if (pid_angle.error > -5.0f && pid_angle.error < 5.0f){ motor_set_I(GM6020,id1,0);}
    pid_angle.ta_error =  pid_angle.ta_error + pid_angle.error * dt;
    if( pid_angle.ta_error > 500){  pid_angle.ta_error = 500;}
    if( pid_angle.ta_error < -500){  pid_angle.ta_error = -500;}
    pid_angle.duty = pid_angle.kp * pid_angle.error + pid_angle.ki * pid_angle.ta_error + pid_angle.kd * (pid_angle.error - pid_angle.last_error)  ;
    pid_angle.re_duty = pro * pid_angle.duty + (1.0f - pro) * pid_angle.last_duty;
    pid_angle.last_duty =pid_angle.duty;
    if (pid_angle.re_duty > 350){ pid_angle.re_duty = 350; }
    if (pid_angle.re_duty < -350){ pid_angle.re_duty = -350; }
    pid_angle.last_error = pid_angle. error;
    return (pid_angle.re_duty);

}



