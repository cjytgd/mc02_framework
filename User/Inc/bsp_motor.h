//
// Created by zzz on 2026/11/14.
//

#ifndef MC02_1_BSP_MOTOR_H
#define MC02_1_BSP_MOTOR_H
typedef enum
{
    GM6020,
    M3508,
    M2006,
    DM_J4310,
}Type;
typedef struct
{
    int16_t me_angle;
    int16_t speed;
    int16_t current;
    int16_t temperature;
    int32_t time;
}RX_DATA;
extern RX_DATA rx_data;

void motor_set_current(Type type,uint8_t id ,float current);
void motor_send_current();
void motor_prase_can(uint32_t rec_id,uint8_t*rx_data);
#endif //MC02_1_BSP_MOTOR_H
