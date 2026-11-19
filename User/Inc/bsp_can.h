//
// Created by zzz on 2026/11/10.
//

#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"

#define hcan_t FDCAN_HandleTypeDef
typedef struct {
    int16_t me_angle;
    int16_t speed;
    int16_t current;
    int16_t temperature;
    int32_t time;
}RX_DAtA;
extern RX_DAtA rx_data;
extern volatile float count;
void bsp_can_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);
void fdcan2_rx_callback(void);
void fdcan3_rx_callback(void);

#endif /* __BSP_FDCAN_H_ */