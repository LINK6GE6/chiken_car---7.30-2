#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "main.h"
#include "delay.h"
/* 传感器数�?结构�? */
typedef struct {
    /* MPU6050 DMP 姿态数�? */
    float pitch;        /* �?仰�?? (�?) */
    float roll;         /* �?滚�?? (�?) */
    float yaw;          /* 偏航�? (�?) */
    int16_t accel[3];   /* 原�?�加速度�? */
    int16_t gyro[3];    /* 原�?�陀螺仪 */

    /* BH1750 �����⴫���� */
    int16_t bh1750_lux;        /* ����ǿ�� (lux) */

} SensorData_t;

/* Calibration & Filter */
#define IMU_CALIBRATION_SAMPLES   20
#define IMU_FILTER_ALPHA          0.9f

/* 全局传感器数�?实�?? */
extern unsigned char g_dmp_ready;
extern SensorData_t g_SensorData;
extern float g_imu_offset_pitch, g_imu_offset_roll, g_imu_offset_yaw;

/*���մ���������*/
typedef unsigned char uchar;
typedef unsigned int uint;

#define SlaveAddress 0x46
extern uchar BUF[5];
extern uchar ge,shi,bai,qian,wan;

// ���ź궨�� PB3=SCL PB4=SDA
#define GZ_SOFT_SCL_Pin    GPIO_PIN_4
#define GZ_SOFT_SCL_GPIO_Port   GPIOB
#define GZ_SOFT_SDA_Pin    GPIO_PIN_3
#define GZ_SOFT_SDA_GPIO_Port   GPIOB

#define SCL_H()        HAL_GPIO_WritePin(GZ_SOFT_SCL_GPIO_Port,GZ_SOFT_SCL_Pin,GPIO_PIN_SET)
#define SCL_L()        HAL_GPIO_WritePin(GZ_SOFT_SCL_GPIO_Port,GZ_SOFT_SCL_Pin,GPIO_PIN_RESET)
#define SDA_H()        HAL_GPIO_WritePin(GZ_SOFT_SDA_GPIO_Port,GZ_SOFT_SDA_Pin,GPIO_PIN_SET)
#define SDA_L()        HAL_GPIO_WritePin(GZ_SOFT_SDA_GPIO_Port,GZ_SOFT_SDA_Pin,GPIO_PIN_RESET)
#define SDA_READ()     HAL_GPIO_ReadPin(GZ_SOFT_SDA_GPIO_Port,GZ_SOFT_SDA_Pin)

/* MPU6050 DMP 初�?�化（I2C1，上电配�? + 加载 DMP 固件 + �?�? DMP�? */
void MPU6050_DMP_Init(void);

/* 读取一包 DMP FIFO */
void Sensor_ReadDMP(void);

/* 静置校准零偏 (需在 DMP 启动后调用) */
void MPU6050_Calibrate(void);

/*���մ�������������*/
void BH1750_GPIO_Init(void);
void conversion(uint temp_data);

void BH1750_Start(void);
void BH1750_Stop(void);
void BH1750_SendACK(uint8_t ack);
uint8_t BH1750_RecvACK(void);
void BH1750_SendByte(uchar dat);
uchar BH1750_RecvByte(void);

void Single_Write_BH1750(uchar REG_Address);
void Multiple_read_BH1750(void);
void BH1750_Init(void);         /* �ϵ�+���������߷ֱ���ģʽ */
void BH1750_Read(void);      /* ��ȡ����ֵ������ g_SensorData.bh1750_lux */
#endif /* __SENSOR_H__ */
