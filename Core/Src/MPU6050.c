#include "MPU6050.h"
#include "i2c.h"    // 引入 hi2c1 句柄
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "cmsis_os.h"

/* 业务逻辑宏定义 */
#define XY_ACC_THRESH    1800
#define FALL_DEBOUNCE_CNT 1

// 互补滤波算法参数
#define Kp 2.0f
#define Ki 0.005f

// 陀螺仪零偏变量 (新加入)
static int16_t gx_offset = 0;
static int16_t gy_offset = 0;
static int16_t gz_offset = 0;
static uint8_t fall_detect_cnt = 0;

/* DMA 数据接收缓冲区 (14字节：包含加速度、温度、陀螺仪) */
uint8_t mpu_rx_buf[14];

/* DMA 接收完成标志位 */
volatile uint8_t g_mpu_dma_ready = 0;

/************************ 底层驱动函数实现 ************************/

// 使用 HAL 库的阻塞式写入（用于初始化配置）
void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
}

// 使用 HAL 库的阻塞式读取（用于读取 ID 等单次操作）
uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
    uint8_t Data = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, RegAddress, I2C_MEMADD_SIZE_8BIT, &Data, 1, 100);
    return Data;
}

/************************ MPU6050 核心接口 ************************/

// MPU6050 寄存器初始化
void MPU6050_Init(void)
{
    // i2c.c 中的 MX_I2C1_Init 已由 HAL 库在 main 函数中调用，此处直接写寄存器

    // 1. 解除休眠，选择内部8MHz时钟
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    // 2. 采样率分频 = 1000/(1+9) = 100Hz
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x09);
    // 3. 低通滤波配置 = 256Hz
    MPU6050_WriteReg(MPU6050_CONFIG, 0x00);
    // 4. 陀螺仪量程配置 = ±2000°/s
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
    // 5. 加速度计量程配置 = ±16g
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x18);
}

// 获取设备 ID
uint8_t MPU6050_GetID(void)
{
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

// 触发一次 DMA 连续读取 14 个字节数据（非阻塞）
void MPU6050_ReadData_DMA(void)
{
    // 确保 I2C 总线当前处于空闲状态，避免上一次传输未完成导致死锁
    if (hi2c1.State == HAL_I2C_STATE_READY)
    {
        g_mpu_dma_ready = 0; // 重置标志位
        // 触发 DMA，从 0x3B (ACCEL_XOUT_H) 开始连续读取 14 字节
        HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H,
                             I2C_MEMADD_SIZE_8BIT, mpu_rx_buf, 14);
    }
}

// 解析由 DMA 获取到的原始数据
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
    if (AccX)  *AccX  = (int16_t)((mpu_rx_buf[0] << 8) | mpu_rx_buf[1]);
    if (AccY)  *AccY  = (int16_t)((mpu_rx_buf[2] << 8) | mpu_rx_buf[3]);
    if (AccZ)  *AccZ  = (int16_t)((mpu_rx_buf[4] << 8) | mpu_rx_buf[5]);

    // mpu_rx_buf[6] 和 [7] 是温度数据 (TEMP_OUT)，这里跳过

    if (GyroX) *GyroX = (int16_t)((mpu_rx_buf[8] << 8) | mpu_rx_buf[9]);
    if (GyroY) *GyroY = (int16_t)((mpu_rx_buf[10] << 8) | mpu_rx_buf[11]);
    if (GyroZ) *GyroZ = (int16_t)((mpu_rx_buf[12] << 8) | mpu_rx_buf[13]);
}

/************************ 业务逻辑接口 ************************/

// 跌倒检测算法（基于已缓冲的 DMA 数据）
uint8_t MPU6050_CheckFall(void)
{
    int16_t ax, ay;
    uint32_t xy_acc_sum;
    uint8_t fall_flag = 0;

    // 获取解析后的加速度数据
    MPU6050_GetData(&ax, &ay, NULL, NULL, NULL, NULL);

    // 计算 X 轴和 Y 轴加速度绝对值之和
    xy_acc_sum = abs(ax) + abs(ay);

    // 跌倒判定
    if (xy_acc_sum > XY_ACC_THRESH)
    {
        fall_detect_cnt++;
        if (fall_detect_cnt >= FALL_DEBOUNCE_CNT)
        {
            fall_flag = 1;
            fall_detect_cnt = 0;
        }
    }
    else
    {
        fall_detect_cnt = 0;
        fall_flag = 0;
    }

    return fall_flag;
}

/************************ 中断回调函数 ************************/

// I2C 的 DMA 内存接收完成回调函数，由 HAL 库在中断中自动调用
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        g_mpu_dma_ready = 1; // 标记数据已经准备就绪
    }
}

/************************ 轻量级姿态解算 ************************/
/**
 * @brief  获取三姿态角 (基于 DMA 的最新数据)
 * @param  pitch: 俯仰角 (返回 -90 到 90 度)
 * @param  roll:  横滚角 (返回 -180 到 180 度)
 * @param  yaw:   偏航角 (基于角速度积分，会缓慢漂移)
 */
void MPU6050_GetAngle(float *pitch, float *roll, float *yaw)
{
    int16_t ax, ay, az, gx, gy, gz;
    static float yaw_accumulated = 0.0f;  // 静态变量，保存累积的 Yaw 角
    static uint32_t last_time = 0;        // 保存上一次计算的时间戳
    uint32_t current_time = HAL_GetTick();

    // 1. 从 DMA 接收缓冲区中获取最新原始数据
    MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);
		printf("DMA Receive,OK\r\n");
    // 消除陀螺仪静态零偏！
    gx -= gx_offset;
    gy -= gy_offset;
    gz -= gz_offset;

    // 2. 根据加速度计数据计算 Pitch (俯仰) 和 Roll (横滚)
    // atan2f 返回的是弧度，乘以 RAD_TO_DEG 转换为角度
    if (pitch)
    {
        // 俯仰角：绕 Y 轴旋转
        *pitch = atan2f((float)-ax, sqrtf((float)ay * ay + (float)az * az)) * RAD_TO_DEG;
    }

    if (roll)
    {
        // 横滚角：绕 X 轴旋转
        *roll  = atan2f((float)ay, (float)az) * RAD_TO_DEG;
    }


    // 3. 根据陀螺仪 Z 轴数据对时间积分，计算 Yaw (偏航角)
    // 在 MPU6050_Init 中，我们将陀螺仪量程设置为了 ±2000°/s
    // 根据数据手册，量程 ±2000°/s 对应的灵敏度为 16.4 LSB/(°/s)
    if (yaw)
    {
        if (last_time != 0)
        {
            // 计算两次调用的时间差 (秒)
            float dt = (current_time - last_time) / 1000.0f;

            // 将原始数据转为实际角速度 (°/s)
            float gz_dps = (float)gz / 16.4f;

            // 简单零偏死区过滤：如果角速度极小，认为是静态噪声，不累加
            if(abs(gz) > 15)
            {
                yaw_accumulated += gz_dps * dt;
            }
        }
        *yaw = yaw_accumulated;
				if(*yaw > 180.0f)          *yaw -= 360;
				else if(*yaw < -180.0f)    *yaw += 360;
    }
		
		printf("pitch:%1.f,roll:%1.f,yaw:%1.f,OK\r\n", *pitch,*roll,*yaw);
    // 更新时间戳
    last_time = current_time;
}

/**
 * @brief 陀螺仪静置零偏校准
 * @note  请务必在 FreeRTOS 调度器启动后，且设备保持【绝对静止】时调用！
 */
void MPU6050_Calibrate_Offset(void)
{
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int16_t ax, ay, az, gx, gy, gz;
    const uint16_t CALIBRATION_SAMPLES = 200; // 采集200次求平均

    printf("[IMU] Calibrating... Please KEEP STILL!\r\n");

    // 丢弃前 20 组数据（等待传感器内部滤波稳定）
    for(int i = 0; i < 20; i++)
    {
        MPU6050_ReadData_DMA();
        while(!g_mpu_dma_ready) { osDelay(1); } // RTOS 友好的死等
        g_mpu_dma_ready = 0;
        osDelay(5);
    }

    // 正式采集并累加
    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        MPU6050_ReadData_DMA();

        // 等待 DMA 搬运完成，利用 osDelay(1) 挂起当前任务，不占用 CPU
        while(!g_mpu_dma_ready) {
            osDelay(1);
        }
        g_mpu_dma_ready = 0;

        // 获取原始数据
        MPU6050_GetData(&ax, &ay, &az, &gx, &gy, &gz);

        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;

        osDelay(5); // 采样间隔，200次 * 5ms = 大约 1 秒钟的校准时间
    }

    // 求平均值得到零偏
    gx_offset = gx_sum / CALIBRATION_SAMPLES;
    gy_offset = gy_sum / CALIBRATION_SAMPLES;
    gz_offset = gz_sum / CALIBRATION_SAMPLES;

    printf("[IMU] Calibration Done! Offsets: gx=%d, gy=%d, gz=%d\r\n",
           gx_offset, gy_offset, gz_offset);
}
