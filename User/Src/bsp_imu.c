//
// Created by zzz on 2026/11/10.
//
//  BMI088 驱动（达妙 DM-MC02 / CtrBoard-H7 板载 IMU）
//  参考：达妙官方例程 Device/BMI088（BMI088Middleware.c + BMI088driver.c）
//
//  加速度计和陀螺仪挂在同一个 SPI2 上，靠两个独立片选区分：
//      PC0 = ACC_CS  （加速度计）
//      PC3 = GYRO_CS （陀螺仪）
//  两者接反时读出来的全是 0x00 / 0xFF，这是这块板最常见的坑。
//
//  SPI 时序（CubeMX 中已配好，这里只做说明）：
//      CPOL = 1、CPHA = 2Edge（模式 3），8bit，MSB first
//      加速度计读：地址(bit7=1) + 1 个哑字节 + 数据
//      陀螺仪读  ：地址(bit7=1) + 数据
//      两者写    ：地址(bit7=0) + 数据
//

#include <stddef.h>

#include"../../User/Inc/bsp_imu.h"
#include"../../User/Inc/bsp_led.h"
#include"spi.h"

/* ========================= 硬件连接 ========================= */
#define IMU_SPI             (&hspi2)
#define IMU_CS_ACC_PORT     GPIOC
#define IMU_CS_ACC_PIN      GPIO_PIN_0
#define IMU_CS_GYRO_PORT    GPIOC
#define IMU_CS_GYRO_PIN     GPIO_PIN_3

#define IMU_SPI_TIMEOUT_MS  100U    /* HAL SPI 超时，单位 ms */
#define IMU_BURST_MAX       8U      /* 一次连续读的最大数据字节数 */

/* ========================= 芯片 ID ========================= */
#define BMI088_ACC_CHIP_ID          0x00U
#define BMI088_ACC_CHIP_ID_VALUE    0x1EU
#define BMI088_GYRO_CHIP_ID         0x00U
#define BMI088_GYRO_CHIP_ID_VALUE   0x0FU

/* ========================= 加速度计寄存器 ========================= */
#define BMI088_ACC_XOUT_L           0x12U   /* 0x12~0x17：X/Y/Z 各 16bit，低字节在前 */
#define BMI088_ACC_CONF             0x40U
#define BMI088_ACC_RANGE            0x41U
#define BMI088_ACC_PWR_CONF         0x7CU   /* 注意是 0x7C，不是 0x78 */
#define BMI088_ACC_PWR_CTRL         0x7DU
#define BMI088_ACC_SOFTRESET        0x7EU
#define BMI088_ACC_SOFTRESET_VAL    0xB6U

#define BMI088_ACC_ENABLE_ON        0x04U   /* ACC_PWR_CTRL：打开加速度计 */
#define BMI088_ACC_ACTIVE_MODE      0x00U   /* ACC_PWR_CONF：退出 suspend */
#define BMI088_ACC_CONF_800HZ       0xABU   /* 0x80(必写) | normal 带宽 | 800Hz ODR */
#define BMI088_ACC_RANGE_3G         0x00U   /* ±3g */

/* ========================= 陀螺仪寄存器 ========================= */
#define BMI088_GYRO_X_L             0x02U   /* 0x02~0x07：X/Y/Z 各 16bit，低字节在前 */
#define BMI088_GYRO_RANGE           0x0FU
#define BMI088_GYRO_BANDWIDTH       0x10U
#define BMI088_GYRO_LPM1            0x11U
#define BMI088_GYRO_SOFTRESET       0x14U
#define BMI088_GYRO_CTRL            0x15U
#define BMI088_GYRO_SOFTRESET_VAL   0xB6U

#define BMI088_GYRO_RANGE_2000      0x00U   /* ±2000dps */
#define BMI088_GYRO_BW_1000_116     0x82U   /* 0x80(必写) | 1000Hz ODR / 116Hz 带宽 */
#define BMI088_GYRO_NORMAL_MODE     0x00U   /* LPM1：normal */
#define BMI088_GYRO_DRDY_ON         0x80U   /* INT_CTRL：使能 new data 中断 */

/* ========================= 灵敏度 ========================= */
/* 加速度计 ±3g：3 * 9.80665 / 32768 = 0.0008974358974 (m/s^2)/LSB */
#define IMU_ACCEL_SEN       0.0008974358974f
/* 陀螺仪 ±2000dps：2000 / 32768 = 0.06103515625 (deg/s)/LSB
 * 需要弧度制就换成 0.0010652644360316953f */
#define IMU_GYRO_SEN        0.06103515625f

/* 数据手册要求的上电/复位等待时间 */
#define IMU_RESET_DELAY_MS  80U
#define IMU_WAIT_DELAY_MS   1U

/* ========================= 片选操作 ========================= */
static void bmi088_cs_low(uint8_t accel) {
    if (accel) {
        HAL_GPIO_WritePin(IMU_CS_ACC_PORT, IMU_CS_ACC_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IMU_CS_GYRO_PORT, IMU_CS_GYRO_PIN, GPIO_PIN_RESET);
    }
}

static void bmi088_cs_high(uint8_t accel) {
    if (accel) {
        HAL_GPIO_WritePin(IMU_CS_ACC_PORT, IMU_CS_ACC_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(IMU_CS_GYRO_PORT, IMU_CS_GYRO_PIN, GPIO_PIN_SET);
    }
}

/* ========================= 写单个寄存器 ========================= */
/* 加速度计、陀螺仪都是 2 字节：地址(bit7 清 0 表示写) + 数据 */
static void bmi088_write_reg(uint8_t accel, uint8_t reg, uint8_t data) {
    uint8_t tx_buf[2];

    tx_buf[0] = (uint8_t) (reg & 0x7FU);
    tx_buf[1] = data;

    bmi088_cs_low(accel);
    (void) HAL_SPI_Transmit(IMU_SPI, tx_buf, 2U, IMU_SPI_TIMEOUT_MS);
    bmi088_cs_high(accel);
}

/* ========================= 读单个寄存器 ========================= */
static uint8_t bmi088_read_reg(uint8_t accel, uint8_t reg) {
    uint8_t tx_buf[3] = {(uint8_t) (reg | 0x80U), 0x55U, 0x55U};
    uint8_t rx_buf[3] = {0U, 0U, 0U};
    /* 加速度计：地址 + 哑字节 + 数据 = 3 字节，数据在 rx_buf[2]
     * 陀螺仪  ：地址 + 数据        = 2 字节，数据在 rx_buf[1] */
    uint16_t len = accel ? 3U : 2U;
    uint16_t off = accel ? 2U : 1U;

    bmi088_cs_low(accel);
    (void) HAL_SPI_TransmitReceive(IMU_SPI, tx_buf, rx_buf, len, IMU_SPI_TIMEOUT_MS);
    bmi088_cs_high(accel);

    return rx_buf[off];
}

/* ========================= 连续读多个寄存器 ========================= */
/* 一次片选内把 len 个字节读回来，加速度计仍要多发一个哑字节 */
static void bmi088_read_regs(uint8_t accel, uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t tx_buf[IMU_BURST_MAX + 2U];
    uint8_t rx_buf[IMU_BURST_MAX + 2U];
    uint16_t off = accel ? 2U : 1U;     /* 数据在接收缓冲里的起始下标 */
    uint16_t total;
    uint16_t i;

    if ((data == NULL) || (len == 0U) || (len > IMU_BURST_MAX)) {
        return;
    }

    total = (uint16_t) (len + off);

    for (i = 0U; i < total; i++) {
        tx_buf[i] = 0x55U;              /* 哑字节随便发，MOSI 上读的时候没用 */
    }
    tx_buf[0] = (uint8_t) (reg | 0x80U);

    bmi088_cs_low(accel);
    (void) HAL_SPI_TransmitReceive(IMU_SPI, tx_buf, rx_buf, total, IMU_SPI_TIMEOUT_MS);
    bmi088_cs_high(accel);

    for (i = 0U; i < len; i++) {
        data[i] = rx_buf[off + i];
    }
}

/* ========================= 对外调试接口 ========================= */
uint8_t IMU_ReadReg(uint8_t accel, uint8_t reg) {
    return bmi088_read_reg(accel, reg);
}

void IMU_WriteReg(uint8_t accel, uint8_t reg, uint8_t data) {
    bmi088_write_reg(accel, reg, data);
}

void IMU_ReadRegs(uint8_t accel, uint8_t reg, uint8_t *data, uint8_t len) {
    bmi088_read_regs(accel, reg, data, len);
}

uint8_t IMU_CheckID(void) {
    uint8_t acc_id = bmi088_read_reg(1U, BMI088_ACC_CHIP_ID);
    uint8_t gyro_id = bmi088_read_reg(0U, BMI088_GYRO_CHIP_ID);
    uint8_t err = 0U;

    if (acc_id != BMI088_ACC_CHIP_ID_VALUE) {
        err |= 0x01U;
    }
    if (gyro_id != BMI088_GYRO_CHIP_ID_VALUE) {
        err |= 0x02U;
    }
    return err;
}

/* ========================= 片选引脚 ========================= */
/* 达妙官方例程把两个 CS 都配成 VERY_HIGH 的推挽输出；这里再配一次，
 * 即使 CubeMX 里引脚速度被改成 LOW（6MHz SPI 下片选边沿会被拖慢）也能正常工作 */
static void imu_cs_gpio_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 先给高电平（不选中），再切换到输出，避免初始化瞬间误选中芯片 */
    HAL_GPIO_WritePin(IMU_CS_ACC_PORT, IMU_CS_ACC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_CS_GYRO_PORT, IMU_CS_GYRO_PIN, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = IMU_CS_ACC_PIN | IMU_CS_GYRO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IMU_CS_ACC_PORT, &GPIO_InitStruct);
}

/* ========================= 加速度计初始化 ========================= */
static uint8_t imu_accel_init(void) {
    /* 顺序和达妙官方例程一致：先上电，再退出 suspend，最后配带宽/量程 */
    static const uint8_t cfg[][2] = {
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ON},    /* 0x7D = 0x04：加速度计上电 */
            {BMI088_ACC_PWR_CONF, BMI088_ACC_ACTIVE_MODE},  /* 0x7C = 0x00：退出 suspend */
            {BMI088_ACC_CONF,     BMI088_ACC_CONF_800HZ},   /* 0x40 = 0xAB：normal 带宽, 800Hz */
            {BMI088_ACC_RANGE,    BMI088_ACC_RANGE_3G},     /* 0x41 = 0x00：±3g */
    };
    uint8_t id;
    uint8_t i;

    /* 上电默认是 I2C 模式：先空读一次，片选的下降沿把芯片切到 SPI */
    (void) bmi088_read_reg(1U, BMI088_ACC_CHIP_ID);
    HAL_Delay(IMU_WAIT_DELAY_MS);

    /* 软复位，复位之后又会回到 I2C 模式 */
    bmi088_write_reg(1U, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VAL);
    HAL_Delay(IMU_RESET_DELAY_MS);

    /* 复位后再空读一次切回 SPI，然后才能读到正确的 ID(0x1E) */
    (void) bmi088_read_reg(1U, BMI088_ACC_CHIP_ID);
    HAL_Delay(IMU_WAIT_DELAY_MS);

    id = bmi088_read_reg(1U, BMI088_ACC_CHIP_ID);
    if (id != BMI088_ACC_CHIP_ID_VALUE) {
        return 1U;
    }

    for (i = 0U; i < (uint8_t) (sizeof(cfg) / sizeof(cfg[0])); i++) {
        bmi088_write_reg(1U, cfg[i][0], cfg[i][1]);
        HAL_Delay(IMU_WAIT_DELAY_MS);
    }

    /* 回读校验：写不进去一般是片选接反/SPI 没通 */
    if (bmi088_read_reg(1U, BMI088_ACC_PWR_CTRL) != BMI088_ACC_ENABLE_ON) {
        return 2U;
    }
    return 0U;
}

/* ========================= 陀螺仪初始化 ========================= */
static uint8_t imu_gyro_init(void) {
    static const uint8_t cfg[][2] = {
            {BMI088_GYRO_RANGE,     BMI088_GYRO_RANGE_2000},   /* 0x0F = 0x00：±2000dps */
            {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_BW_1000_116},  /* 0x10 = 0x82：1000Hz / 116Hz */
            {BMI088_GYRO_LPM1,      BMI088_GYRO_NORMAL_MODE},  /* 0x11 = 0x00：normal */
            {BMI088_GYRO_CTRL,      BMI088_GYRO_DRDY_ON},      /* 0x15 = 0x80：使能 data ready */
    };
    uint8_t id;
    uint8_t i;

    /* 陀螺仪上电就是 normal 模式，直接软复位再等 80ms */
    bmi088_write_reg(0U, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VAL);
    HAL_Delay(IMU_RESET_DELAY_MS);

    id = bmi088_read_reg(0U, BMI088_GYRO_CHIP_ID);
    if (id != BMI088_GYRO_CHIP_ID_VALUE) {
        return 1U;
    }

    for (i = 0U; i < (uint8_t) (sizeof(cfg) / sizeof(cfg[0])); i++) {
        bmi088_write_reg(0U, cfg[i][0], cfg[i][1]);
        HAL_Delay(IMU_WAIT_DELAY_MS);
    }

    if (bmi088_read_reg(0U, BMI088_GYRO_RANGE) != BMI088_GYRO_RANGE_2000) {
        return 2U;
    }
    return 0U;
}

/* ========================= 初始化入口 ========================= */
void IMU_Init(void) {
    uint8_t acc_err;
    uint8_t gyro_err;

    HAL_Delay(50);              /* 等 BMI088 上电稳定 */

    imu_cs_gpio_init();

    acc_err = imu_accel_init();
    gyro_err = imu_gyro_init();

    if ((acc_err == 0U) && (gyro_err == 0U)) {
        bsp_led_set(normal, 250, 0, 0);
    } else {
        bsp_led_set(error, 250, 0, 0);
    }
}

/* ========================= 读六轴数据 ========================= */
void IMU_GetData(IMU_Data_t *IMU_Data) {
    uint8_t accel_buf[6] = {0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t gyro_buf[6] = {0U, 0U, 0U, 0U, 0U, 0U};
    int16_t raw;

    if (IMU_Data == NULL) {
        return;
    }

    /* 每次片选连续读 6 字节：X/Y/Z 来自同一次采样，比逐字节读快得多 */
    bmi088_read_regs(1U, BMI088_ACC_XOUT_L, accel_buf, 6U);
    bmi088_read_regs(0U, BMI088_GYRO_X_L, gyro_buf, 6U);

    /* 低字节在前 */
    raw = (int16_t) (((uint16_t) accel_buf[1] << 8) | (uint16_t) accel_buf[0]);
    IMU_Data->accel_x = (float) raw * IMU_ACCEL_SEN;
    raw = (int16_t) (((uint16_t) accel_buf[3] << 8) | (uint16_t) accel_buf[2]);
    IMU_Data->accel_y = (float) raw * IMU_ACCEL_SEN;
    raw = (int16_t) (((uint16_t) accel_buf[5] << 8) | (uint16_t) accel_buf[4]);
    IMU_Data->accel_z = (float) raw * IMU_ACCEL_SEN;

    raw = (int16_t) (((uint16_t) gyro_buf[1] << 8) | (uint16_t) gyro_buf[0]);
    IMU_Data->gyro_x = (float) raw * IMU_GYRO_SEN;
    raw = (int16_t) (((uint16_t) gyro_buf[3] << 8) | (uint16_t) gyro_buf[2]);
    IMU_Data->gyro_y = (float) raw * IMU_GYRO_SEN;
    raw = (int16_t) (((uint16_t) gyro_buf[5] << 8) | (uint16_t) gyro_buf[4]);
    IMU_Data->gyro_z = (float) raw * IMU_GYRO_SEN;
}

