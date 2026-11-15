//
// Created by zzz on 2026/11/14.
//

#ifndef MC02_1_BSP_MOTOR_H
#define MC02_1_BSP_MOTOR_H
typedef enum {
    id1,
    id2,
    id3,
    id4,
    id5,
    id6,
    id7,
    id8,
}ID_data;
typedef enum {
    GM6020,
    M3508,

}MOTOR_TYPE;
void motor_send(void);
void motor_set_I(MOTOR_TYPE motor,ID_data id,uint16_t I);
#endif //MC02_1_BSP_MOTOR_H
