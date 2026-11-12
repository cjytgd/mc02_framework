//
// Created by zzz on 2026/11/10.
//
//  BMI088 板载 IMU（达妙 DM-MC02 / CtrBoard-H7）
//  硬件连接：SPI2（PB13 = SCK，PC1 = MOSI，PC2_C = MISO，模式 3）
//            ACC_CS  = PC0   （加速度计片选，低有效）
//            GYRO_CS = PC3   （陀螺仪片选，低有效）
//  注意：达妙板上 PC0 是加速度计、PC3 是陀螺仪，两者不能互换。
//

#ifndef MC02_1_BSP_IMU_H
#define MC02_1_BSP_IMU_H

#include"main.h"

/* 单位：accel_x/y/z = m/s^2 ，gyro_x/y/z = deg/s
 * 对应量程 ±3g / ±2000dps，改量程时请同步修改 bsp_imu.c 里的灵敏度系数 */
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
}IMU_Data_t;

/* 初始化：片选引脚 + 加速度计/陀螺仪的软复位、ID 校验和寄存器配置 */
void IMU_Init(void);

/* 读一次六轴数据（各用一次连续读，保证三轴来自同一次采样） */
void IMU_GetData(IMU_Data_t *IMU_Data);

/* 供调试/自检使用：accel 非 0 表示访问加速度计，0 表示访问陀螺仪 */
uint8_t IMU_ReadReg(uint8_t accel, uint8_t reg);
void IMU_WriteReg(uint8_t accel, uint8_t reg, uint8_t data);
/* 连续读多个寄存器（len 最大 8），加速度计自动插入哑字节 */
void IMU_ReadRegs(uint8_t accel, uint8_t reg, uint8_t *data, uint8_t len);

/* 读芯片 ID 自检：bit0 = 加速度计异常，bit1 = 陀螺仪异常，返回 0 表示都正常 */
uint8_t IMU_CheckID(void);

#endif //MC02_1_BSP_IMU_H