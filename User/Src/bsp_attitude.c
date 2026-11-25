//
// Created by zzz on 2026/11/23.
//
//  BMI088 姿态解算：误差状态四元数 EKF（ESKF）
//  详细说明见 bsp_attitude.h
//

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "../../User/Inc/bsp_attitude.h"

#define ATTITUDE_PI             3.14159265358979323846f
#define ATTITUDE_DEG2RAD        (ATTITUDE_PI / 180.0f)
#define ATTITUDE_RAD2DEG        (180.0f / ATTITUDE_PI)
#define ATTITUDE_GRAVITY        9.80665f
#define ATTITUDE_EKF_N          6U

/* ========================= 对外数据 ========================= */
ATTITUDE_DATA attitude_data;

/* ========================= 名义状态 ========================= */
static float s_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};   /* w,x,y,z */
static float s_bias[3] = {0.0f, 0.0f, 0.0f};      /* rad/s */

/* ========================= 误差状态协方差 ========================= */
static float s_P[ATTITUDE_EKF_N][ATTITUDE_EKF_N];

/* ========================= 矩阵工作区 =========================
 * 全部放静态区，避免在 1kHz 任务栈上反复分配 6x6 矩阵。
 * 本模块只由姿态任务调用，不需要加锁。
 */
static float s_F[6][6];
static float s_FP[6][6];
static float s_Pnew[6][6];
static float s_T1[6][6];
static float s_H[3][6];
static float s_Ht[6][3];
static float s_PHt[6][3];
static float s_S[3][3];
static float s_Sinv[3][3];
static float s_K[6][3];
static float s_KH[6][6];
static float s_IKH[6][6];
static float s_IKHt[6][6];
static float s_Kt[3][6];
static float s_KRK[6][6];

/* ========================= DWT 计时 ========================= */
static uint32_t s_dwt_last = 0U;
static uint8_t s_dwt_ok = 0U;

/* ========================= 矩阵基础运算 ========================= */

static void mat_mul(uint8_t m, uint8_t n, uint8_t p,
                    const float *A, const float *B, float *C) {
    uint8_t i;
    uint8_t j;
    uint8_t k;

    for (i = 0U; i < m; i++) {
        for (j = 0U; j < p; j++) {
            float sum = 0.0f;
            for (k = 0U; k < n; k++) {
                sum += A[i * n + k] * B[k * p + j];
            }
            C[i * p + j] = sum;
        }
    }
}

static void mat_transpose(uint8_t rows, uint8_t cols,
                          const float *A, float *At) {
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < rows; i++) {
        for (j = 0U; j < cols; j++) {
            At[j * rows + i] = A[i * cols + j];
        }
    }
}

static void mat_symmetrize(uint8_t n, float *A) {
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < n; i++) {
        for (j = (uint8_t) (i + 1U); j < n; j++) {
            float m = 0.5f * (A[i * n + j] + A[j * n + i]);
            A[i * n + j] = m;
            A[j * n + i] = m;
        }
    }
}

/* 3x3 解析求逆，成功返回 0，奇异返回 1 */
static uint8_t mat3_inv(const float *A, float *Ainv) {
    float det = A[0] * (A[4] * A[8] - A[5] * A[7])
              - A[1] * (A[3] * A[8] - A[5] * A[6])
              + A[2] * (A[3] * A[7] - A[4] * A[6]);
    float inv_det;

    if ((fabsf(det) < 1.0e-9f) || (!isfinite(det))) {
        return 1U;
    }

    inv_det = 1.0f / det;

    Ainv[0] =  (A[4] * A[8] - A[5] * A[7]) * inv_det;
    Ainv[1] = -(A[1] * A[8] - A[2] * A[7]) * inv_det;
    Ainv[2] =  (A[1] * A[5] - A[2] * A[4]) * inv_det;
    Ainv[3] = -(A[3] * A[8] - A[5] * A[6]) * inv_det;
    Ainv[4] =  (A[0] * A[8] - A[2] * A[6]) * inv_det;
    Ainv[5] = -(A[0] * A[5] - A[2] * A[3]) * inv_det;
    Ainv[6] =  (A[3] * A[7] - A[4] * A[6]) * inv_det;
    Ainv[7] = -(A[0] * A[7] - A[1] * A[6]) * inv_det;
    Ainv[8] =  (A[0] * A[4] - A[1] * A[3]) * inv_det;

    return 0U;
}

/* ========================= 四元数运算 ========================= */

static void quat_normalize(float q[4]) {
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    float inv;

    if ((n < 1.0e-9f) || (!isfinite(n))) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    inv = 1.0f / n;
    q[0] *= inv;
    q[1] *= inv;
    q[2] *= inv;
    q[3] *= inv;
}

/* out = a * b（Hamilton 乘法），out 不能与 a/b 是同一块内存 */
static void quat_mul(const float a[4], const float b[4], float out[4]) {
    out[0] = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
    out[1] = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
    out[2] = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
    out[3] = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];
}

/* 旋转矢量转四元数，rv 单位 rad */
static void quat_from_rotvec(const float rv[3], float q[4]) {
    float angle2 = rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2];
    float angle = sqrtf(angle2);

    if (angle < 1.0e-6f) {
        q[0] = 1.0f;
        q[1] = 0.5f * rv[0];
        q[2] = 0.5f * rv[1];
        q[3] = 0.5f * rv[2];
        quat_normalize(q);
    } else {
        float half = 0.5f * angle;
        float s = sinf(half) / angle;
        q[0] = cosf(half);
        q[1] = rv[0] * s;
        q[2] = rv[1] * s;
        q[3] = rv[2] * s;
    }
}

/* ========================= 安装方向映射 ========================= */

static const uint8_t s_map[3] = {
        ATTITUDE_MAP_X,
        ATTITUDE_MAP_Y,
        ATTITUDE_MAP_Z
};

static const float s_sign[3] = {
        ATTITUDE_SIGN_X,
        ATTITUDE_SIGN_Y,
        ATTITUDE_SIGN_Z
};

static void attitude_map(const IMU_Data_t *imu, float acc[3], float gyro_dps[3]) {
    const float a[3] = {imu->accel_x, imu->accel_y, imu->accel_z};
    const float g[3] = {imu->gyro_x, imu->gyro_y, imu->gyro_z};
    uint8_t i;

    for (i = 0U; i < 3U; i++) {
        acc[i] = s_sign[i] * a[s_map[i]];
        gyro_dps[i] = s_sign[i] * g[s_map[i]];
    }
}

/* ========================= DWT 计时 ========================= */

static void attitude_dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_dwt_last = DWT->CYCCNT;
    s_dwt_ok = 1U;
}

/* 返回实际 dt，单位 s，并做上下限保护 */
static float attitude_get_dt(void) {
    uint32_t now;
    uint32_t diff;
    float dt;

    if (s_dwt_ok == 0U) {
        return 1.0f / (float) ATTITUDE_SAMPLE_HZ;
    }

    now = DWT->CYCCNT;
    diff = now - s_dwt_last;      /* uint32 减法，自动处理计数器回绕 */
    s_dwt_last = now;

    dt = (float) diff / (float) SystemCoreClock;
    if ((dt < ATTITUDE_EKF_MIN_DT) || (dt > ATTITUDE_EKF_MAX_DT)) {
        dt = 1.0f / (float) ATTITUDE_SAMPLE_HZ;
    }
    return dt;
}

/* ========================= 协方差初始化 ========================= */

static void ekf_reset_covariance(void) {
    float att_var;
    float bias_var;
    uint8_t i;

    memset(s_P, 0, sizeof(s_P));

    att_var = ATTITUDE_EKF_INIT_ATT_DEG * ATTITUDE_DEG2RAD;
    att_var = att_var * att_var;

    bias_var = ATTITUDE_EKF_INIT_BIAS_DPS * ATTITUDE_DEG2RAD;
    bias_var = bias_var * bias_var;

    for (i = 0U; i < 3U; i++) {
        s_P[i][i] = att_var;
        s_P[i + 3U][i + 3U] = bias_var;
    }
}

/* ========================= 初始姿态对齐 ========================= */

static void attitude_align_from_accel(const float acc[3]) {
    float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    float ax;
    float ay;
    float az;
    float roll;
    float pitch;
    float cr;
    float sr;
    float cp;
    float sp;

    if ((n < 1.0e-3f) || (!isfinite(n))) {
        s_q[0] = 1.0f;
        s_q[1] = 0.0f;
        s_q[2] = 0.0f;
        s_q[3] = 0.0f;
        return;
    }

    ax = acc[0] / n;
    ay = acc[1] / n;
    az = acc[2] / n;

    /* 载体系 Z 轴向上时，静止加速度方向就是重力反方向 */
    roll = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

    cr = cosf(0.5f * roll);
    sr = sinf(0.5f * roll);
    cp = cosf(0.5f * pitch);
    sp = sinf(0.5f * pitch);

    /* q = qy(pitch) * qx(roll)，yaw 取 0 */
    s_q[0] = cr * cp;
    s_q[1] = sr * cp;
    s_q[2] = cr * sp;
    s_q[3] = -sr * sp;

    quat_normalize(s_q);
}

/* ========================= 欧拉角与发布 ========================= */

static void attitude_update_euler(void) {
    const float w = s_q[0];
    const float x = s_q[1];
    const float y = s_q[2];
    const float z = s_q[3];
    float sinp = 2.0f * (w * y - z * x);

    if (sinp > 1.0f) {
        sinp = 1.0f;
    } else if (sinp < -1.0f) {
        sinp = -1.0f;
    }

    attitude_data.roll  = atan2f(2.0f * (w * x + y * z),
                                 1.0f - 2.0f * (x * x + y * y)) * ATTITUDE_RAD2DEG;
    attitude_data.pitch = asinf(sinp) * ATTITUDE_RAD2DEG;
    attitude_data.yaw   = atan2f(2.0f * (w * z + x * y),
                                 1.0f - 2.0f * (y * y + z * z)) * ATTITUDE_RAD2DEG;
}

static void attitude_publish(void) {
    uint8_t i;

    for (i = 0U; i < 4U; i++) {
        attitude_data.q[i] = s_q[i];
    }
    for (i = 0U; i < 3U; i++) {
        attitude_data.gyro_bias[i] = s_bias[i] * ATTITUDE_RAD2DEG;
    }
    attitude_update_euler();
}

/* ========================= EKF 预测 ========================= */

static void eskf_predict(const float gyro_rad[3], float dt) {
    float omega[3];
    float rv[3];
    float dq[4];
    float qn[4];
    float sigma_g;
    float sigma_b;
    float q_theta;
    float q_bias;
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < 3U; i++) {
        omega[i] = gyro_rad[i] - s_bias[i];
        rv[i] = omega[i] * dt;
    }

    quat_from_rotvec(rv, dq);
    quat_mul(s_q, dq, qn);
    quat_normalize(qn);
    memcpy(s_q, qn, sizeof(qn));

    /* F = [ I - [w]x*dt   -I*dt ]
     *     [      0           I  ] */
    for (i = 0U; i < 6U; i++) {
        for (j = 0U; j < 6U; j++) {
            s_F[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    s_F[0][1] =  omega[2] * dt;
    s_F[0][2] = -omega[1] * dt;
    s_F[1][0] = -omega[2] * dt;
    s_F[1][2] =  omega[0] * dt;
    s_F[2][0] =  omega[1] * dt;
    s_F[2][1] = -omega[0] * dt;

    s_F[0][3] = -dt;
    s_F[1][4] = -dt;
    s_F[2][5] = -dt;

    /* P = F * P * F^T + Q */
    mat_mul(6U, 6U, 6U, &s_F[0][0], &s_P[0][0], &s_FP[0][0]);
    mat_transpose(6U, 6U, &s_F[0][0], &s_T1[0][0]);
    mat_mul(6U, 6U, 6U, &s_FP[0][0], &s_T1[0][0], &s_Pnew[0][0]);

    sigma_g = ATTITUDE_EKF_SIGMA_GYRO_DPS * ATTITUDE_DEG2RAD;
    sigma_b = ATTITUDE_EKF_SIGMA_BIAS_DPS * ATTITUDE_DEG2RAD;
    q_theta = sigma_g * sigma_g * dt;
    q_bias  = sigma_b * sigma_b * dt;

    for (i = 0U; i < 3U; i++) {
        s_Pnew[i][i] += q_theta;
        s_Pnew[i + 3U][i + 3U] += q_bias;
    }

    memcpy(s_P, s_Pnew, sizeof(s_P));
    mat_symmetrize(6U, &s_P[0][0]);
}

/* ========================= EKF 加速度计量测更新 ========================= */

static void eskf_update_accel(const float acc[3]) {
    float n = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
    float a[3];
    float v[3];
    float r[3];
    float sigma_a;
    float r_var;
    float maha;
    float tmp[3];
    float dx[6];
    float dtheta[3];
    float angle;
    float max_bias;
    float dq[4];
    float qn[4];
    uint8_t i;
    uint8_t j;

    attitude_data.accel_norm = n;

    /* 模长异常：自由落体、强烈撞击、加减速过大，不做修正 */
    if ((n < 0.5f * ATTITUDE_GRAVITY) || (n > 1.5f * ATTITUDE_GRAVITY)) {
        attitude_data.accel_used = 0U;
        return;
    }

    a[0] = acc[0] / n;
    a[1] = acc[1] / n;
    a[2] = acc[2] / n;

    /* 估计的重力方向（机体系） */
    {
        const float w = s_q[0];
        const float x = s_q[1];
        const float y = s_q[2];
        const float z = s_q[3];

        v[0] = 2.0f * (x * z - w * y);
        v[1] = 2.0f * (w * x + y * z);
        v[2] = w * w - x * x - y * y + z * z;
    }

    r[0] = a[0] - v[0];
    r[1] = a[1] - v[1];
    r[2] = a[2] - v[2];

    /* H = [ [v]x , 0 ]，对右扰动 dtheta 的雅可比 */
    memset(s_H, 0, sizeof(s_H));
    s_H[0][1] = -v[2];
    s_H[0][2] =  v[1];
    s_H[1][0] =  v[2];
    s_H[1][2] = -v[0];
    s_H[2][0] = -v[1];
    s_H[2][1] =  v[0];

    /* S = H*P*H^T + R */
    mat_transpose(3U, 6U, &s_H[0][0], &s_Ht[0][0]);
    mat_mul(6U, 6U, 3U, &s_P[0][0], &s_Ht[0][0], &s_PHt[0][0]);
    mat_mul(3U, 6U, 3U, &s_H[0][0], &s_PHt[0][0], &s_S[0][0]);

    sigma_a = ATTITUDE_EKF_SIGMA_ACC_G;
    r_var = sigma_a * sigma_a;
    for (i = 0U; i < 3U; i++) {
        s_S[i][i] += r_var;
    }

    if (mat3_inv(&s_S[0][0], &s_Sinv[0][0]) != 0U) {
        attitude_data.accel_used = 0U;
        return;
    }

    /* 马氏距离门限：r^T * S^-1 * r */
    for (i = 0U; i < 3U; i++) {
        tmp[i] = s_Sinv[i][0] * r[0]
               + s_Sinv[i][1] * r[1]
               + s_Sinv[i][2] * r[2];
    }
    maha = r[0] * tmp[0] + r[1] * tmp[1] + r[2] * tmp[2];

    if ((!isfinite(maha)) || (maha > ATTITUDE_EKF_GATE)) {
        attitude_data.accel_used = 0U;
        return;
    }

    /* K = P*H^T*S^-1 */
    mat_mul(6U, 3U, 3U, &s_PHt[0][0], &s_Sinv[0][0], &s_K[0][0]);

    /* dx = K*r */
    for (i = 0U; i < 6U; i++) {
        dx[i] = s_K[i][0] * r[0]
              + s_K[i][1] * r[1]
              + s_K[i][2] * r[2];
    }

    /* 姿态修正限幅 */
    dtheta[0] = dx[0];
    dtheta[1] = dx[1];
    dtheta[2] = dx[2];
    angle = sqrtf(dtheta[0] * dtheta[0]
                + dtheta[1] * dtheta[1]
                + dtheta[2] * dtheta[2]);

    if (angle > 0.2f) {
        float scale = 0.2f / angle;
        dtheta[0] *= scale;
        dtheta[1] *= scale;
        dtheta[2] *= scale;
    }

    /* 零偏修正限幅 */
    max_bias = 5.0f * ATTITUDE_DEG2RAD;
    for (i = 3U; i < 6U; i++) {
        if (dx[i] > max_bias) {
            dx[i] = max_bias;
        } else if (dx[i] < -max_bias) {
            dx[i] = -max_bias;
        }
    }

    /* 误差注入：q = q * dq(dtheta)，bias += dbias */
    quat_from_rotvec(dtheta, dq);
    quat_mul(s_q, dq, qn);
    quat_normalize(qn);
    memcpy(s_q, qn, sizeof(qn));

    for (i = 0U; i < 3U; i++) {
        s_bias[i] += dx[i + 3U];
    }

    /* Joseph 形式协方差更新，保证正定 */
    mat_mul(6U, 3U, 6U, &s_K[0][0], &s_H[0][0], &s_KH[0][0]);

    for (i = 0U; i < 6U; i++) {
        for (j = 0U; j < 6U; j++) {
            s_IKH[i][j] = ((i == j) ? 1.0f : 0.0f) - s_KH[i][j];
        }
    }

    mat_transpose(6U, 6U, &s_IKH[0][0], &s_IKHt[0][0]);
    mat_mul(6U, 6U, 6U, &s_IKH[0][0], &s_P[0][0], &s_T1[0][0]);
    mat_mul(6U, 6U, 6U, &s_T1[0][0], &s_IKHt[0][0], &s_Pnew[0][0]);

    mat_transpose(6U, 3U, &s_K[0][0], &s_Kt[0][0]);
    mat_mul(6U, 3U, 6U, &s_K[0][0], &s_Kt[0][0], &s_KRK[0][0]);

    for (i = 0U; i < 6U; i++) {
        for (j = 0U; j < 6U; j++) {
            s_Pnew[i][j] += r_var * s_KRK[i][j];
        }
    }

    memcpy(s_P, s_Pnew, sizeof(s_P));
    mat_symmetrize(6U, &s_P[0][0]);

    attitude_data.accel_used = 1U;
}

/* ========================= 数值健康检查 ========================= */

static uint8_t attitude_state_sane(void) {
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < 4U; i++) {
        if (!isfinite(s_q[i])) {
            return 0U;
        }
    }
    for (i = 0U; i < 3U; i++) {
        if (!isfinite(s_bias[i])) {
            return 0U;
        }
    }
    for (i = 0U; i < 6U; i++) {
        for (j = 0U; j < 6U; j++) {
            if (!isfinite(s_P[i][j])) {
                return 0U;
            }
        }
    }
    return 1U;
}

/* ========================= 对外接口 ========================= */

void Attitude_Reset(void) {
    s_q[0] = 1.0f;
    s_q[1] = 0.0f;
    s_q[2] = 0.0f;
    s_q[3] = 0.0f;

    s_bias[0] = 0.0f;
    s_bias[1] = 0.0f;
    s_bias[2] = 0.0f;

    ekf_reset_covariance();

    memset(&attitude_data, 0, sizeof(attitude_data));
    attitude_data.q[0] = 1.0f;
    attitude_data.ready = 0U;

    if (s_dwt_ok != 0U) {
        s_dwt_last = DWT->CYCCNT;
    }
}

uint8_t Attitude_CalibrateGyro(void) {
    IMU_Data_t imu;
    float sum_acc[3] = {0.0f, 0.0f, 0.0f};
    float sum_gyro[3] = {0.0f, 0.0f, 0.0f};
    float acc[3];
    float gyro[3];
    float inv;
    uint32_t i;
    uint8_t j;

    /* 丢掉上电后的前几十个样本，等 BMI088 内部滤波器稳定 */
    for (i = 0U; i < ATTITUDE_GYRO_CALI_DISCARD; i++) {
        IMU_GetData(&imu);
        HAL_Delay(1U);
    }

    for (i = 0U; i < ATTITUDE_GYRO_CALI_SAMPLES; i++) {
        IMU_GetData(&imu);
        attitude_map(&imu, acc, gyro);

        for (j = 0U; j < 3U; j++) {
            sum_acc[j] += acc[j];
            sum_gyro[j] += gyro[j];
        }
        HAL_Delay(1U);
    }

    inv = 1.0f / (float) ATTITUDE_GYRO_CALI_SAMPLES;

    for (j = 0U; j < 3U; j++) {
        sum_acc[j] *= inv;
        sum_gyro[j] *= inv;
        s_bias[j] = sum_gyro[j] * ATTITUDE_DEG2RAD;
    }

    attitude_align_from_accel(sum_acc);
    ekf_reset_covariance();

    attitude_data.accel_norm = sqrtf(sum_acc[0] * sum_acc[0]
                                   + sum_acc[1] * sum_acc[1]
                                   + sum_acc[2] * sum_acc[2]);
    attitude_data.count = 0U;
    attitude_data.time = HAL_GetTick();
    attitude_data.ready = 1U;
    attitude_data.accel_used = 0U;

    attitude_publish();

    if (s_dwt_ok != 0U) {
        s_dwt_last = DWT->CYCCNT;
    }

    return 0U;
}

uint8_t Attitude_Init(void) {
    attitude_dwt_init();
    Attitude_Reset();

    /* 芯片 ID 不对时不信任标定数据，直接返回错误 */
    if (IMU_CheckID() != 0U) {
        attitude_data.ready = 0U;
        return 1U;
    }

    return Attitude_CalibrateGyro();
}

void Attitude_UpdateWithDt(const IMU_Data_t *imu, float dt_s) {
    float acc[3];
    float gyro_dps[3];
    float gyro_rad[3];
    uint8_t i;

    if (imu == NULL) {
        return;
    }

    if ((!isfinite(dt_s))
        || (dt_s < ATTITUDE_EKF_MIN_DT)
        || (dt_s > ATTITUDE_EKF_MAX_DT)) {
        dt_s = 1.0f / (float) ATTITUDE_SAMPLE_HZ;
    }

    attitude_map(imu, acc, gyro_dps);
    for (i = 0U; i < 3U; i++) {
        gyro_rad[i] = gyro_dps[i] * ATTITUDE_DEG2RAD;
    }

    eskf_predict(gyro_rad, dt_s);
    eskf_update_accel(acc);

    if (attitude_state_sane() == 0U) {
        Attitude_Reset();
        return;
    }

    attitude_data.count++;
    attitude_data.time = HAL_GetTick();
    attitude_publish();
}

void Attitude_Update(const IMU_Data_t *imu) {
    Attitude_UpdateWithDt(imu, attitude_get_dt());
}

void Attitude_GetEuler(float *roll, float *pitch, float *yaw) {
    if (roll != NULL) {
        *roll = attitude_data.roll;
    }
    if (pitch != NULL) {
        *pitch = attitude_data.pitch;
    }
    if (yaw != NULL) {
        *yaw = attitude_data.yaw;
    }
}

void Attitude_GetQuaternion(float q[4]) {
    uint8_t i;

    if (q == NULL) {
        return;
    }

    for (i = 0U; i < 4U; i++) {
        q[i] = attitude_data.q[i];
    }
}

void Attitude_ResetYaw(void) {
    const float w = s_q[0];
    const float x = s_q[1];
    const float y = s_q[2];
    const float z = s_q[3];
    float yaw = atan2f(2.0f * (w * z + x * y),
                       1.0f - 2.0f * (y * y + z * z));
    float qz[4];
    float qn[4];

    qz[0] = cosf(0.5f * yaw);
    qz[1] = 0.0f;
    qz[2] = 0.0f;
    qz[3] = -sinf(0.5f * yaw);

    /* 绕世界 Z 轴左乘，相当于把当前朝向旋转回 yaw = 0 */
    quat_mul(qz, s_q, qn);
    quat_normalize(qn);
    memcpy(s_q, qn, sizeof(qn));

    attitude_publish();
}

uint8_t Attitude_IsReady(void) {
    return attitude_data.ready;
}
