/*
    Hardware By Link
    Software By Link
    2026.03.01
    
*/

#ifndef IMU_H
#define IMU_H

#include "main.h"
#include "LSM6DSRTR.h"

/* =================================================================================
 *                                 宏定义与常量
 * ================================================================================= */
#define PI                  (3.1415926535898f)
#define ANGLE_TO_RAD(x)     ((x) * PI / 180.0f)
#define RAD_TO_ANGLE(x)     ((x) * 180.0f / PI)

/* =================================================================================
 *                                 硬件抽象层接口
 * ================================================================================= */
// 硬件层原始数据标准格式
typedef struct {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} IMU_RawData_t;

typedef struct {
    uint8_t (*init)(void);
    void    (*get_acc)(void);
    void    (*get_gyro)(void);
    IMU_RawData_t *raw_data; // 硬件层原始数据接口指针
} IMUchoice;

/* =================================================================================
 *                                 数据结构定义
 * ================================================================================= */
// 陀螺仪零偏参数
typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_param_t;

// 处理后的IMU数据 (弧度/g)
typedef struct {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
} IMU_param_t;

// 卡尔曼滤波器结构体
typedef struct {
    float X_last, X_mid, X_now;
    float P_mid, P_now, P_last;
    float kg, A, B, Q, R, H;
} extKalman_t;

// 姿态角结构体
typedef struct {
    float pitch_temp, roll_temp;
    float pitch, roll, yaw;
    float last_yaw;      
    int Dirchange;      
} Angle_t;

// 互补滤波结构体
typedef struct {
    Angle_t angle;
} First_Complement_t;

// 四元数结构体
typedef struct {
    float q0, q1, q2, q3;
} quater_param_t;

/* =================================================================================
 *                                 全局变量声明
 * ================================================================================= */
extern IMUchoice          IMU_Choice;
extern IMU_param_t        IMU_Data;
extern First_Complement_t complement;
extern extKalman_t        Kalman1, Kalman2, Zero;

extern float gyro_Offset_flag;
extern float Daty_Z, Daty_X, Daty_Y;
extern float yaw_limit_360, yaw_total;
extern float Pitch_Max, Pitch_Min, Roll_Max, Roll_Min;
extern float Max_Delta_Pitch, Max_Delta_Roll;

extern float I_ex, I_ey, I_ez;
extern float param_Kp, param_Ki;
extern float angle_Z;

/* =================================================================================
 *                                 函数原型声明
 * ================================================================================= */

// --- 系统接口 ---
void gyro_init(void);
void gyro_proc(void);

// --- 时间管理 (DWT) ---
void IMU_dwt_init(void);
void IMU_update_dt(void);

// --- 数据获取与校准 ---
float IMU_gyro_Offset_Init(void);
void  IMU_GetValues(void);

// --- 姿态解算算法 ---
void IMU_Complement(void);
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_getEulerianAngles(void);
void IMU_update_yaw(float yaw);

// --- 积分与航向处理 ---
void IMU_YAW_integral(void);
void IMU_180(void);
void IMU_360(void);
void IMU_0(void);

// --- 滤波与数学工具 ---
void  IMU_KalmanCreate(extKalman_t *p, float T_Q, float T_R);
float IMU_KalmanFilter(extKalman_t *p, float dat);
float fast_sqrt(float num);
float My_abs(float x);

#endif // IMU_H
