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
}ID_data;
typedef struct {
    float me_angle;
    float speed;
    float current;
    float temperature;
}RX_DAtA;
extern RX_DAtA rx_data;
void motor_send(void);
void motor_set_I(ID_data id,uint16_t I);
#endif //MC02_1_BSP_MOTOR_H
