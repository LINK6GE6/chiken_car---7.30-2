#ifndef __MPU6050_H
#define __MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* MPU6050 寄存器地址定义 */
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_WHO_AM_I        0x75
#define RAD_TO_DEG  57.2957795f

/* MPU6050 设备地址 (0x68 左移1位) */
#define MPU6050_ADDR            0xD0

/* 全局变量，指示 DMA 是否完成了一次新数据的搬运 */
extern volatile uint8_t g_mpu_dma_ready;



/* 函数声明 */
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);

/* 发起一次 DMA 异步读取 */
void MPU6050_ReadData_DMA(void);

/* 从 DMA 接收缓冲区中解析原始数据 */
void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                     int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);

/* 直接获取姿态角函数声明 */
void MPU6050_GetAngle(float *pitch, float *roll, float *yaw);

/*零点偏置函数声明*/
void MPU6050_Calibrate_Offset(void);
/* 跌倒检测业务逻辑 */
uint8_t MPU6050_CheckFall(void);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H */
