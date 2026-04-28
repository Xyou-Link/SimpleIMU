/*
    Hardware By Link
    Software By Link
    2026.03.01
    
*/

#ifndef LSM6DSRTR_H
#define LSM6DSRTR_H

#include "main.h"

//================================================定义 LSM6DSRTR 基本配置================================================
#define LSM6DSRTR_CS_PORT                           GPIOB
#define LSM6DSRTR_CS_PIN                            GPIO_PIN_0
#define LSM6DSRTR_CS(x)                             HAL_GPIO_WritePin(LSM6DSRTR_CS_PORT, LSM6DSRTR_CS_PIN, (x ? GPIO_PIN_SET : GPIO_PIN_RESET))

typedef enum
{
    LSM6DSRTR_ACC_SAMPLE_SGN_2G ,                                                // 加速度计量程 ±2G
    LSM6DSRTR_ACC_SAMPLE_SGN_4G ,                                                // 加速度计量程 ±4G
    LSM6DSRTR_ACC_SAMPLE_SGN_8G ,                                                // 加速度计量程 ±8G
    LSM6DSRTR_ACC_SAMPLE_SGN_16G,                                                // 加速度计量程 ±16G
}lsm6dsrtr_acc_sample_config;

typedef enum
{
    LSM6DSRTR_GYRO_SAMPLE_SGN_125DPS ,                                           // 陀螺仪量程 ±125DPS
    LSM6DSRTR_GYRO_SAMPLE_SGN_250DPS ,                                           // 陀螺仪量程 ±250DPS
    LSM6DSRTR_GYRO_SAMPLE_SGN_500DPS ,                                           // 陀螺仪量程 ±500DPS
    LSM6DSRTR_GYRO_SAMPLE_SGN_1000DPS,                                           // 陀螺仪量程 ±1000DPS
    LSM6DSRTR_GYRO_SAMPLE_SGN_2000DPS,                                           // 陀螺仪量程 ±2000DPS
    LSM6DSRTR_GYRO_SAMPLE_SGN_4000DPS,                                           // 陀螺仪量程 ±4000DPS
}lsm6dsrtr_gyro_sample_config;

#define LSM6DSRTR_ACC_SAMPLE_DEFAULT     ( LSM6DSRTR_ACC_SAMPLE_SGN_8G )          // 在这设置默认的 加速度计 初始化量程
#define LSM6DSRTR_GYRO_SAMPLE_DEFAULT    ( LSM6DSRTR_GYRO_SAMPLE_SGN_2000DPS )    // 在这设置默认的 陀螺仪   初始化量程
#define LSM6DSRTR_TIMEOUT_COUNT                      (0x00FF)                    // LSM6DSRTR 超时计数
//================================================定义 LSM6DSRTR 基本配置================================================

//================================================定义 LSM6DSRTR 内部地址================================================
#define LSM6DSRTR_DEV_ADDR                           (0x6B)                      // SA0接地：0x6A SA0上拉：0x6B 模块默认上拉
#define LSM6DSRTR_SPI_W                              (0x00)
#define LSM6DSRTR_SPI_R                              (0x80)

#define LSM6DSRTR_FUNC_CFG_ACCESS                    (0x01)
#define LSM6DSRTR_INT1_CTRL                          (0x0D)
#define LSM6DSRTR_WHO_AM_I                           (0x0F)
#define LSM6DSRTR_CTRL1_XL                           (0x10)
#define LSM6DSRTR_CTRL2_G                            (0x11)
#define LSM6DSRTR_CTRL3_C                            (0x12)
#define LSM6DSRTR_CTRL4_C                            (0x13)
#define LSM6DSRTR_CTRL5_C                            (0x14)
#define LSM6DSRTR_CTRL6_C                            (0x15)
#define LSM6DSRTR_CTRL7_G                            (0x16)
#define LSM6DSRTR_CTRL9_XL                           (0x18)
#define LSM6DSRTR_OUTX_L_G                           (0x22)
#define LSM6DSRTR_OUTX_L_A                           (0x28)

//集线器功能相关寄存器 需要将FUNC_CFG_ACCESS的SHUB_REG_ACCESS位设置为1才能正确访问
#define LSM6DSRTR_SENSOR_HUB_1                       (0x02)
#define LSM6DSRTR_MASTER_CONFIG                      (0x14)
#define LSM6DSRTR_SLV0_ADD                           (0x15)
#define LSM6DSRTR_SLV0_SUBADD                        (0x16)
#define LSM6DSRTR_SLV0_CONFIG                        (0x17)
#define LSM6DSRTR_DATAWRITE_SLV0                     (0x21)
#define LSM6DSRTR_STATUS_MASTER                      (0x22)

#define LSM6DSRTR_MAG_ADDR                           (0x0D)                      // 7位IIC地址
#define LSM6DSRTR_MAG_OUTX_L                         (0x00)
#define LSM6DSRTR_MAG_CONTROL1                       (0x09)
#define LSM6DSRTR_MAG_CONTROL2                       (0x0A)
#define LSM6DSRTR_MAG_FBR                            (0x0B)
#define LSM6DSRTR_MAG_CHIP_ID                        (0x0D)

#define LSM6DSRTR_ACC_SAMPLE                         (0x3C)                      // 加速度计量程
#define LSM6DSRTR_GYR_SAMPLE                         (0x5C)                      // 陀螺仪量程

//================================================声明 LSM6DSRTR 全局变量================================================
typedef struct
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} lsm6dsrtr_data_t;

extern lsm6dsrtr_data_t lsm6dsrtr_data;                                          // LSM6DSRTR 数据结构体
extern float lsm6dsrtr_transition_factor[2];                                     // 转换实际值的比例
//================================================声明 LSM6DSRTR 全局变量================================================

//================================================声明 LSM6DSRTR 基础函数================================================
void    lsm6dsrtr_get_acc            (void);                                     // 获取 LSM6DSRTR 加速度计数据
void    lsm6dsrtr_get_gyro           (void);                                     // 获取 LSM6DSRTR 陀螺仪数据
uint8_t   lsm6dsrtr_init               (void);                                     // 初始化 LSM6DSRTR
//================================================声明 LSM6DSRTR 基础函数================================================

#endif // LSM6DSRTR_H
