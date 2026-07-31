#include "Sensor.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "usart.h"
#include <stdio.h>
#include "i2c.h"
#include "delay.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* 全局传感器数据实例 */
SensorData_t g_SensorData;
unsigned char g_dmp_ready;
extern osMutexId_t global_sensor_data_mutexHandle;

/* 零偏校准值 */
float g_imu_offset_pitch = 0.0f;
float g_imu_offset_roll  = 0.0f;
float g_imu_offset_yaw   = 0.0f;

/* 低通滤波状态 */
static float s_filtered_pitch = 0.0f;
static float s_filtered_roll  = 0.0f;
static float s_filtered_yaw   = 0.0f;
static uint8_t s_filter_inited = 0;

/* -------------------------------------------------------------------------- */
/* 四元数 -> 欧拉角 (弧度)                                                     */
/* quat 为 DMP 输出的 q30 定点数: quat[0]=w, quat[1]=x, quat[2]=y, quat[3]=z  */
/* -------------------------------------------------------------------------- */
static void quat_to_euler(const long *quat, float *pitch, float *roll, float *yaw)
{
    const float q30 = 1073741824.0f;   /* 2^30 */

    float qw = (float)quat[0] / q30;
    float qx = (float)quat[1] / q30;
    float qy = (float)quat[2] / q30;
    float qz = (float)quat[3] / q30;

    float sinp, sinr_cosp, siny_cosp, cosr_cosp, cosy_cosp;

    /* Pitch (绕 Y 轴) */
    sinp = 2.0f * (qw * qy - qz * qx);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    *pitch = asinf(sinp);

    /* Roll (绕 X 轴) */
    sinr_cosp = 2.0f * (qw * qx + qy * qz);
    cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    /* Yaw (绕 Z 轴) */
    siny_cosp = 2.0f * (qw * qz + qx * qy);
    cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    *yaw = atan2f(siny_cosp, cosy_cosp);
}

/* -------------------------------------------------------------------------- */
/* MPU6050 DMP 初始化                                                          */
/* -------------------------------------------------------------------------- */
void MPU6050_DMP_Init(void)
{
    struct int_param_s int_param;
    int_param.cb  = NULL;
    int_param.arg = NULL;

    uint8_t attempt;
	
    
		
    for (attempt = 1; attempt <= 10; attempt++) {
        printf("[SENSOR] Init attempt %d/10rn", attempt);
				

        /* 1. 初始化 MPU6050：I2C 复位、唤醒、自检 */
        if (mpu_init(&int_param) != 0) {
            printf("[SENSOR] mpu_init failed!rn");
            continue;
        }
        printf("[SENSOR] mpu_init OKrn");

        /* 2. 使能三轴加速度计 + 三轴陀螺仪 */
        if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) != 0) {
            printf("[SENSOR] mpu_set_sensors failed!rn");
            continue;
        }

        /* 3. 设置采样率 200 Hz */
        if (mpu_set_sample_rate(200) != 0) {
            printf("[SENSOR] mpu_set_sample_rate failed!rn");
            continue;
        }

        /* 4. 加载 DMP 运动固件 */
        if (dmp_load_motion_driver_firmware() != 0) {
            printf("[SENSOR] dmp_load_firmware failed!rn");
            continue;
        }
        printf("[SENSOR] DMP firmware loaded OKrn");

        /* 5. 设置 DMP FIFO 输出速率 200 Hz */
        if (dmp_set_fifo_rate(200) != 0) {
            printf("[SENSOR] dmp_set_fifo_rate failed!rn");
            continue;
        }

        /* 6. 使能 6 轴低功耗四元数 + 原始加速度计 + 原始陀螺仪输出 */
        if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT
                                | DMP_FEATURE_SEND_RAW_ACCEL
                                | DMP_FEATURE_SEND_RAW_GYRO) != 0) {
            printf("[SENSOR] dmp_enable_feature failed!rn");
            continue;
        }

        /* 7. 启动 DMP */
        if (mpu_set_dmp_state(1) != 0) {
            printf("[SENSOR] mpu_set_dmp_state failed!rn");
            continue;
        }
        printf("[SENSOR] DMP started OKrn");
        g_dmp_ready = 1;
        return;
    }
    printf("[SENSOR] ALL 10 attempts FAILED!rn");
    g_dmp_ready = 0;
}

/* -------------------------------------------------------------------------- */
/* MPU6050 零偏校准：静置状态下读取 N 次，取均值作为零偏                         */
/* 必须在 DMP 启动后调用，调用时传感器需保持静止                                 */
/* -------------------------------------------------------------------------- */
void MPU6050_Calibrate(void)
{
    float sum_pitch = 0.0f, sum_roll = 0.0f, sum_yaw = 0.0f;
    int valid = 0;

    printf("[SENSOR] Calibrating zero offset (%d samples)...rn", IMU_CALIBRATION_SAMPLES);

    while (valid < IMU_CALIBRATION_SAMPLES) {
        short gyro[3], accel[3], sensors;
        long quat[4];
        unsigned long timestamp;
        unsigned char more;

        if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) == 0) {
            if (sensors & INV_WXYZ_QUAT) {
                float pitch, roll, yaw;
                quat_to_euler(quat, &pitch, &roll, &yaw);
                sum_pitch += pitch * 57.29578f;
                sum_roll  += roll  * 57.29578f;
                sum_yaw   += yaw   * 57.29578f;
                valid++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    g_imu_offset_pitch = sum_pitch / (float)IMU_CALIBRATION_SAMPLES;
    g_imu_offset_roll  = sum_roll  / (float)IMU_CALIBRATION_SAMPLES;
    g_imu_offset_yaw   = sum_yaw   / (float)IMU_CALIBRATION_SAMPLES;

    /* 滤波状态也同步初始化为零偏后的值 (零) */
    s_filtered_pitch = 0.0f;
    s_filtered_roll  = 0.0f;
    s_filtered_yaw   = 0.0f;
    s_filter_inited  = 0;  /* 第一帧将用原始值初始化 */

    printf("[SENSOR] Calibration done: P=%.2f R=%.2f Y=%.2frn",
           g_imu_offset_pitch, g_imu_offset_roll, g_imu_offset_yaw);
}

/* -------------------------------------------------------------------------- */
/* 读取一包 DMP FIFO 数据，更新全局 g_SensorData 并通过串口打印姿态              */
/* -------------------------------------------------------------------------- */
void Sensor_ReadDMP(void)
{
    short gyro[3], accel[3], sensors;
    long quat[4];
    unsigned long timestamp;
    unsigned char more;

     if (!g_dmp_ready) {
        return;
     }

    /* 从 DMP FIFO 读一包数据 */
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) != 0) {
        return;   /* FIFO 尚无可用数据 */
    }

    /* 保存原始传感器数据 */
    g_SensorData.accel[0] = accel[0];
    g_SensorData.accel[1] = accel[1];
    g_SensorData.accel[2] = accel[2];
    g_SensorData.gyro[0]  = gyro[0];
    g_SensorData.gyro[1]  = gyro[1];
    g_SensorData.gyro[2]  = gyro[2];

    /* 如果有四元数数据，转换为欧拉角 */
    if (sensors & INV_WXYZ_QUAT) {
        float pitch, roll, yaw;
        quat_to_euler(quat, &pitch, &roll, &yaw);

        /* 弧度转度，减去零偏 */
        pitch = pitch * 57.29578f - g_imu_offset_pitch;
        roll  = roll  * 57.29578f - g_imu_offset_roll;
        yaw   = yaw   * 57.29578f - g_imu_offset_yaw;

        /* Yaw 归一化到 [-180, 180] */
        if (yaw > 180.0f)  yaw -= 360.0f;
        if (yaw < -180.0f) yaw += 360.0f;

        /* 一阶低通滤波（含 yaw 环绕处理） */
        if (!s_filter_inited) {
            s_filtered_pitch = pitch;
            s_filtered_roll  = roll;
            s_filtered_yaw   = yaw;
            s_filter_inited  = 1;
        } else {
            s_filtered_pitch = s_filtered_pitch * IMU_FILTER_ALPHA
                             + pitch * (1.0f - IMU_FILTER_ALPHA);
            s_filtered_roll  = s_filtered_roll  * IMU_FILTER_ALPHA
                             + roll  * (1.0f - IMU_FILTER_ALPHA);

            /* Yaw 滤波需处理 ±180 环绕 */
            float diff = yaw - s_filtered_yaw;
            if (diff > 180.0f)  diff -= 360.0f;
            if (diff < -180.0f) diff += 360.0f;
            s_filtered_yaw += diff * (1.0f - IMU_FILTER_ALPHA);
            if (s_filtered_yaw > 180.0f)  s_filtered_yaw -= 360.0f;
            if (s_filtered_yaw < -180.0f) s_filtered_yaw += 360.0f;
        }

        g_SensorData.pitch = s_filtered_pitch;
        g_SensorData.roll  = s_filtered_roll;
        g_SensorData.yaw   = s_filtered_yaw;
    }

    /* 通过 UART1 输出姿态到串口 */
//    printf("Pitch:%.2f Roll:%.2f Yaw:%.2frn",
//           g_SensorData.pitch, g_SensorData.roll, g_SensorData.yaw);
}


uchar BUF[5];
uchar ge,shi,bai,qian,wan;

// PB3 PB4初始化 开漏+上拉 默认高电平
void BH1750_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // 上电默认拉高
    HAL_GPIO_WritePin(GZ_SOFT_SCL_GPIO_Port, GZ_SOFT_SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GZ_SOFT_SDA_GPIO_Port, GZ_SOFT_SDA_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GZ_SOFT_SCL_Pin | GZ_SOFT_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GZ_SOFT_SCL_GPIO_Port, &GPIO_InitStruct);
}


// 数据拆分转ASCII
void conversion(uint temp_data)
{
    wan=temp_data/10000+0x30 ;
    temp_data=temp_data%10000;
    qian=temp_data/1000+0x30 ;
    temp_data=temp_data%1000;
    bai=temp_data/100+0x30;
    temp_data=temp_data%100;
    shi=temp_data/10+0x30;
    temp_data=temp_data%10;
    ge=temp_data+0x30;
}

// I2C起始
void BH1750_Start()
{
    SDA_H();
    SCL_H();
    Delay5us();
    SDA_L();
    Delay5us();
    SCL_L();
}

// I2C停止
void BH1750_Stop()
{
    SDA_L();
    SCL_H();
    Delay5us();
    SDA_H();
    Delay5us();
}

// 发送应答
void BH1750_SendACK(uint8_t ack)
{
    if(ack) SDA_H();
    else SDA_L();
    SCL_H();
    Delay5us();
    SCL_L();
    Delay5us();
}

// 接收应答
uint8_t BH1750_RecvACK()
{
    uint8_t cy;
    SCL_H();
    Delay5us();
    cy = SDA_READ();
    SCL_L();
    Delay5us();
    return cy;
}

// 发送1字节
void BH1750_SendByte(uchar dat)
{
    uchar i;
    for (i=0; i<8; i++)
    {
					if (dat & 0x80) SDA_H();   // MSB=1 → SDA拉高
					else            SDA_L();   // MSB=0 → SDA拉低
				dat <<= 1;
        SCL_H();
        Delay5us();
        SCL_L();
        Delay5us();
    }
    BH1750_RecvACK();
}

// 接收1字节
uchar BH1750_RecvByte()
{
    uchar i;
    uchar dat = 0;
    SDA_H();
    for (i=0; i<8; i++)
    {
        dat <<= 1;
        SCL_H();
        Delay5us();
        if(SDA_READ()) dat |= 0x01;
        SCL_L();
        Delay5us();
    }
    return dat;
}

// BH1750单字节写
void Single_Write_BH1750(uchar REG_Address)
{
    BH1750_Start();
    BH1750_SendByte(SlaveAddress);
    BH1750_SendByte(REG_Address);
    BH1750_Stop();
}

// 读取光照2字节
void Multiple_read_BH1750(void)
{
    uchar i;
    BH1750_Start();
    BH1750_SendByte(SlaveAddress+1);

    for (i=0; i<2; i++)
    {
        BUF[i] = BH1750_RecvByte();
        if (i == 1)
        {
            BH1750_SendACK(1);
        }
        else
        {
            BH1750_SendACK(0);
        }
    }
    BH1750_Stop();
    Delay_ms(5);
}

// BH1750 初始化：上电 + 设置连续高分辨率模式
void BH1750_Init(void)
{
    Single_Write_BH1750(0x01);
    Delay_ms(10);
    Single_Write_BH1750(0x10);
    Delay_ms(200);
}

void BH1750_Read(void)
{
    if(osMutexAcquire(global_sensor_data_mutexHandle, 100) == osOK)
    {
        Multiple_read_BH1750();
        uint16_t raw = ((uint16_t)BUF[0] << 8)| BUF[1];
        g_SensorData.bh1750_lux = (uint16_t)((float)raw / 1.2f);  // BH1750 公式

        osMutexRelease(global_sensor_data_mutexHandle);
    }
}
