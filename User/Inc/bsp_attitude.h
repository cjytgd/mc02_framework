//
// Created by zzz on 2026/11/23.
//
//  BMI088 姿态解算：误差状态四元数 EKF（ESKF）
//
//  坐标系约定：
//      载体系 X 前、Y 左、Z 上（可通过下面的 MAP/SIGN 宏修改）
//      四元数 q = [w, x, y, z]，表示 body -> world
//      欧拉角 ZYX 顺序：yaw(Z) -> pitch(Y) -> roll(X)，单位 deg
//
//  状态：
//      名义状态：四元数 q + 陀螺零偏 b（rad/s）
//      误差状态：dx = [dtheta(3), dbias(3)]，共 6 维
//
//  观测量：
//      加速度计重力方向。BMI088 没有磁力计，roll/pitch 可观测，
//      yaw 不可观测，只能靠陀螺积分，会缓慢漂移。
//

#ifndef MC02_1_BSP_ATTITUDE_H
#define MC02_1_BSP_ATTITUDE_H

#include"bsp_imu.h"

/* ========================= 安装方向 =========================
 * 载体系定义为 X 前、Y 左、Z 上。
 * 0/1/2 分别表示取传感器的 x/y/z 轴。
 * 若 BMI088 在板子上的安装方向不同，只改下面 6 个宏。
 *
 * 例：传感器 X 朝后、Y 朝右、Z 朝上
 *     ATTITUDE_MAP_X = 0, ATTITUDE_SIGN_X = -1.0f
 *     ATTITUDE_MAP_Y = 1, ATTITUDE_SIGN_Y = -1.0f
 */
#define ATTITUDE_MAP_X              0
#define ATTITUDE_MAP_Y              1
#define ATTITUDE_MAP_Z              2
#define ATTITUDE_SIGN_X             1.0f
#define ATTITUDE_SIGN_Y             1.0f
#define ATTITUDE_SIGN_Z             1.0f

/* ========================= 采样与标定 ========================= */
#define ATTITUDE_SAMPLE_HZ          1000U   /* 姿态任务采样频率 */
#define ATTITUDE_GYRO_CALI_SAMPLES  1000U   /* 上电静止标定样本数（约 1s） */
#define ATTITUDE_GYRO_CALI_DISCARD  50U     /* 标定前丢弃的样本数 */

/* ========================= EKF 噪声参数 =========================
 * SIGMA_GYRO_DPS ：陀螺噪声，单位 deg/s，越大越不信任陀螺
 * SIGMA_BIAS_DPS ：零偏随机游走，单位 deg/s/sqrt(s)
 * SIGMA_ACC_G    ：加速度计方向噪声，单位 g（归一化后约等于 rad）
 * GATE           ：马氏距离门限，3 自由度 99% 约为 11.34
 */
#define ATTITUDE_EKF_SIGMA_GYRO_DPS     0.20f
#define ATTITUDE_EKF_SIGMA_BIAS_DPS     0.02f
#define ATTITUDE_EKF_SIGMA_ACC_G        0.05f
#define ATTITUDE_EKF_INIT_ATT_DEG       5.0f
#define ATTITUDE_EKF_INIT_BIAS_DPS      0.50f
#define ATTITUDE_EKF_GATE               11.34f
#define ATTITUDE_EKF_MIN_DT             0.0002f
#define ATTITUDE_EKF_MAX_DT             0.02f

/* ========================= 输出数据 ========================= */
typedef struct {
    float q[4];              /* 四元数 w,x,y,z */
    float roll;              /* deg */
    float pitch;             /* deg */
    float yaw;               /* deg，无磁力计，只有相对值 */
    float gyro_bias[3];      /* deg/s，已标定 + EKF 估计的零偏 */
    float accel_norm;        /* m/s^2，最近一次加速度模长 */
    uint32_t count;          /* EKF 更新次数 */
    uint32_t time;           /* 最近一次更新时间，HAL_GetTick() */
    uint8_t ready;           /* 1 = 零偏标定和初始对齐完成 */
    uint8_t accel_used;      /* 1 = 最近一次使用了加速度计修正 */
} ATTITUDE_DATA;

extern ATTITUDE_DATA attitude_data;

/* 初始化：DWT 计时 + 陀螺零偏标定 + 加速度计初始对齐
 * 返回值：0 成功，1 芯片 ID 异常，2 标定样本数为 0
 * 注意：调用时小车必须静止，函数内部会阻塞约 1s */
uint8_t Attitude_Init(void);

/* 重新标定陀螺零偏（运行中调用时必须保证小车静止，并暂停姿态任务） */
uint8_t Attitude_CalibrateGyro(void);

/* 1kHz 任务里调用，dt 由 DWT 周期计数器测量 */
void Attitude_Update(const IMU_Data_t *imu);

/* 指定 dt 更新，便于离线测试或与其他周期任务同步 */
void Attitude_UpdateWithDt(const IMU_Data_t *imu, float dt_s);

/* 读取欧拉角，单位 deg */
void Attitude_GetEuler(float *roll, float *pitch, float *yaw);

/* 读取四元数 w,x,y,z */
void Attitude_GetQuaternion(float q[4]);

/* 把当前朝向设为 yaw = 0（roll/pitch 不变） */
void Attitude_ResetYaw(void);

/* 复位滤波器（四元数、零偏、协方差全部清零），ready 置 0 */
void Attitude_Reset(void);

uint8_t Attitude_IsReady(void);

#endif //MC02_1_BSP_ATTITUDE_H
