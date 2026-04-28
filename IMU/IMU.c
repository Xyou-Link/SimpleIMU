/*
    Hardware By Link
    Software By Link
    2026.03.01
    
*/

#include "IMU.h"
#include "LSM6DSRTR.h"
#include "math.h"

/* =================================================================================
 *                                      配置与宏
 * ================================================================================= */
#define IMUMODE     1           // 1: 开启软件解算，0: 硬件自带解算
#define IMUCHOICE   1           // 0： 维特陀螺仪 1: LSM6DSRTR 2：其他型号（预留）
#define delta_T     0.005f      // 采样周期 5ms（保留兼容）
// STM32F303 @ 72MHz 鉴于主控可能不同请在这里修改你的主频，数据解算周期由此主频自动计算，无需进一步更改
#define CPU_HZ      72000000UL  

/* =================================================================================
 *                                硬件抽象层接口
 * ================================================================================= */
//由于不确定使用IMU的型号，在这里做解耦处理，具体实现放在对应的IMU驱动文件中，通过函数指针调用
//此处可以传入解算算法需要的原始数据结构体指针，供解算算法使用
//不需解算的用户可以直接调用get_acc/get_gyro获取原始数据，将数据等效到complement.angle中，直接使用complement.angle  

IMUchoice IMU_Choice =
#if (IMUCHOICE == 1)
{
    .init     = lsm6dsrtr_init,
    .get_acc  = lsm6dsrtr_get_acc,
    .get_gyro = lsm6dsrtr_get_gyro,
    .raw_data = (IMU_RawData_t *)&lsm6dsrtr_data,
}

#else
{
    .init     = (void (*)(void))NULL,
    .get_acc  = (void (*)(void))NULL,
    .get_gyro = (void (*)(void))NULL,
    .raw_data = (void (*)(void))NULL,
}

#endif
;
/* =================================================================================
 *                                系统接口（Init & Proc）
 * ================================================================================= */
#if (IMUMODE == 0)

void gyro_init(void)
{
   IMU_Choice.init();

}
void gyro_proc(void)
{
    // IMU_Choice.get_acc();
    // IMU_Choice.get_gyro();
    complement.angle.pitch = IMU_Choice.raw_data->acc_x;
    complement.angle.roll = IMU_Choice.raw_data->acc_y;
    complement.angle.yaw = IMU_Choice.raw_data->gyro_z;
}

#elif (IMUMODE == 1)

void gyro_init(void)
{
    IMU_Choice.init();
    IMU_gyro_Offset_Init();
    IMU_dwt_init();
    IMU_KalmanCreate(&Kalman1, 0.001f, 0.1f);
    IMU_KalmanCreate(&Kalman2, 0.001f, 0.1f);

}

void gyro_proc(void)
{
    IMU_update_dt();
    IMU_GetValues();
    IMU_YAW_integral();
    IMU_update_yaw(Daty_Z);
    IMU_Complement();
}

#endif






/* =================================================================================
 *                                    全局变量定义
 * ================================================================================= */

// --- 时间管理 ---
float dt_actual = 0.005f;       // 实际采样周期，由 DWT 测量，单位：秒
static uint32_t last_cyccnt = 0;
static uint8_t  dt_inited   = 0;

// --- Mahony/AHRS 相关参数（保留） ---
float I_ex = 0, I_ey = 0, I_ez = 0;  // 积分误差
float param_Kp = 0.05f;              // 比例增益
float param_Ki = 0.2f;               // 积分增益
float angle_Z = 0;                   // 连续 yaw 角

// --- IMU 数据与滤波状态 ---
First_Complement_t complement;
gyro_param_t Gyro_Offset;            // 陀螺仪零偏参数
IMU_param_t IMU_Data;                 // 去零偏后的 IMU 数据
extKalman_t Kalman1;
extKalman_t Kalman2;
extKalman_t Zero;

// --- 校准与 yaw 积分状态 ---
float gyro_Offset_flag = 0;
float Daty_Z = 0;
float Daty_X = 0;
float Daty_Y = 0;
char Round = 0;
float yaw_limit_360 = 0;
float yaw_total = 0;

// --- 姿态极值统计 ---
float Pitch_Max = 0;
float Pitch_Min = 0;
float Roll_Max = 0;
float Roll_Min = 0;
float Max_Delta_Pitch = 0;
float Max_Delta_Roll = 0;



/* =================================================================================
 *                                      时间管理
 * ================================================================================= */
void IMU_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void IMU_update_dt(void)
{
    uint32_t now = DWT->CYCCNT;
    if (!dt_inited) {
        last_cyccnt = now;
        dt_inited   = 1;
        dt_actual   = 0.005f;
        return;
    }
    uint32_t diff = now - last_cyccnt;  // 32 位无符号溢出自动处理
    last_cyccnt = now;
    dt_actual = (float)diff / (float)CPU_HZ;
    // 异常保护：限制在 0.1ms ~ 100ms
    if (dt_actual < 0.0001f) dt_actual = 0.0001f;
    if (dt_actual > 0.1f)    dt_actual = 0.1f;
}

/* =================================================================================
 *                                  数据获取与零偏校准
 * ================================================================================= */
float IMU_gyro_Offset_Init(void)
{
    Gyro_Offset.Xdata = 0;
    Gyro_Offset.Ydata = 0;
    Gyro_Offset.Zdata = 0;

    for (uint16_t i = 0; i < 100; i++)
    {
        Gyro_Offset.Xdata += IMU_Choice.raw_data->gyro_x;
        Gyro_Offset.Ydata += IMU_Choice.raw_data->gyro_y;
        Gyro_Offset.Zdata += IMU_Choice.raw_data->gyro_z;
        HAL_Delay(5);
    }

    Gyro_Offset.Xdata /= 100;
    Gyro_Offset.Ydata /= 100;
    Gyro_Offset.Zdata /= 100;

    return gyro_Offset_flag = 1;
}

void IMU_GetValues(void)
{
    IMU_Choice.get_gyro();
    IMU_Choice.get_acc();

    IMU_Data.gyro_x = (IMU_Choice.raw_data->gyro_x - Gyro_Offset.Xdata) * PI / 180.0f;
    IMU_Data.gyro_y = (IMU_Choice.raw_data->gyro_y - Gyro_Offset.Ydata) * PI / 180.0f;
    IMU_Data.gyro_z = (IMU_Choice.raw_data->gyro_z - Gyro_Offset.Zdata) * PI / 180.0f;

    IMU_Data.acc_x = (((float)IMU_Choice.raw_data->acc_x) * 0.3f) + IMU_Data.acc_x * 0.7f;
    IMU_Data.acc_y = (((float)IMU_Choice.raw_data->acc_y) * 0.3f) + IMU_Data.acc_y * 0.7f;
    IMU_Data.acc_z = (((float)IMU_Choice.raw_data->acc_z) * 0.3f) + IMU_Data.acc_z * 0.7f;
}

/* =================================================================================
 *                                  Pitch/Roll 姿态解算
 * ================================================================================= */
void IMU_Complement(void)
{
    float k = 0.85;
    static float Pitch_Temp = 0;
    static float Roll_Temp = 0;
    static float Delta_Roll = 0;
    static char Pitch_Flag = 0;
    static char Roll_Flag = 0;

    // 使用 atan2f 替代 atanf(y/z)，可自动处理 z = 0 的情况，角度更稳定
    Pitch_Temp = atan2f(IMU_Data.acc_y, IMU_Data.acc_z) * 180.0f / 3.1415926535f;
    Roll_Temp  = atan2f(IMU_Data.acc_x, IMU_Data.acc_z) * 180.0f / 3.1415926535f;

    Pitch_Temp = Pitch_Temp + IMU_Data.gyro_y * dt_actual;
    Roll_Temp = Roll_Temp + IMU_Data.gyro_x * dt_actual;

    complement.angle.pitch = k * Pitch_Temp + (1 - k) * (complement.angle.pitch + IMU_Data.gyro_y * dt_actual);
    complement.angle.roll = k * Roll_Temp + (1 - k) * (complement.angle.roll + IMU_Data.gyro_x * dt_actual);

    if (Delta_Roll < 0.1 && Delta_Roll > -0.1)
    {
        complement.angle.roll = complement.angle.roll - Delta_Roll;
    }

    complement.angle.roll = IMU_KalmanFilter(&Kalman1, complement.angle.roll);
    complement.angle.pitch = IMU_KalmanFilter(&Kalman2, complement.angle.pitch);

    if (Pitch_Flag == 0)
    {
        Pitch_Min = complement.angle.roll;
        Pitch_Max = complement.angle.roll;

        Pitch_Flag = 1;
    }

    if (Roll_Flag == 0)
    {
        Roll_Min = complement.angle.pitch;
        Roll_Max = complement.angle.pitch;

        Roll_Flag = 1;
    }

    if (My_abs(Pitch_Min) > My_abs(complement.angle.roll))
    {
        Pitch_Min = My_abs(complement.angle.roll);
    }
    else if (My_abs(Pitch_Max) < My_abs(complement.angle.roll))
    {
        Pitch_Max = My_abs(complement.angle.roll);
    }

    if (My_abs(Roll_Min) > My_abs(complement.angle.pitch))
    {
        Roll_Min = My_abs(complement.angle.pitch);
    }
    else if (My_abs(Roll_Max) < My_abs(complement.angle.pitch))
    {
        Roll_Max = My_abs(complement.angle.pitch);
    }
    Max_Delta_Pitch = Pitch_Max - Pitch_Min;
    Max_Delta_Roll = Roll_Max - Roll_Min;
}

/* =================================================================================
 *                                    Yaw / Z 轴积分
 * ================================================================================= */
void IMU_YAW_integral(void)
{
    if (IMU_Data.gyro_z < 0.015 && IMU_Data.gyro_z > -0.015)
    {
        Daty_Z -= 0;
    }
    else
    {
        
        IMU_180();
        IMU_360();
        IMU_0();
    }

    if (IMU_Data.gyro_x < 0.015 && IMU_Data.gyro_x > -0.015)
    {
        Daty_X -= 0;
    }
    if (IMU_Data.gyro_y < 0.015 && IMU_Data.gyro_y > -0.015)
    {
        Daty_Y -= 0;
    }
}

void IMU_180(void)
{
    Daty_Z -= RAD_TO_ANGLE(IMU_Data.gyro_z * dt_actual);

    if ((Daty_Z > 0 && Daty_Z <= 180) || (Daty_Z < 0 && Daty_Z >= (-180)))
    {
        Daty_Z = +Daty_Z;
    }
    else if (Daty_Z > 180 && Daty_Z <= 360)
    {
        Daty_Z -= 360;
    }
    else if (Daty_Z < (-180) && Daty_Z >= (-360))
    {
        Daty_Z += 360;
    }
}

void IMU_360(void)
{
    if (Round == 0)
    {
        yaw_limit_360 -= RAD_TO_ANGLE(IMU_Data.gyro_z * dt_actual);
        if (yaw_limit_360 > 360)
        {
            yaw_limit_360 = 360;
            Round = 1;
        }
        else if (yaw_limit_360 < -360)
        {
            yaw_limit_360 = -360;
            Round = 1;
        }
    }
    if (Round == 1)
    {
        if (yaw_limit_360 <= 360 && yaw_limit_360 >= -360)
        {
            yaw_limit_360 += RAD_TO_ANGLE(IMU_Data.gyro_z * dt_actual);
            if (yaw_limit_360 > 360)
            {
                yaw_limit_360 = 360;
                Round = 0;
            }
        }
        else if (yaw_limit_360 >= -360)
        {
            yaw_limit_360 -= RAD_TO_ANGLE(IMU_Data.gyro_z * dt_actual);
            if (yaw_limit_360 < -360)
            {

                yaw_limit_360 = -360;
                Round = 0;
            }
        }
    }
}

void IMU_update_yaw(float yaw)
{
    if ((yaw - complement.angle.last_yaw) < -350.0f)
    {
        complement.angle.Dirchange++;
    }
    else if ((yaw - complement.angle.last_yaw) > 350.0f)
    {
        complement.angle.Dirchange--;
    }

    angle_Z = 360.0f * complement.angle.Dirchange + yaw;
    complement.angle.yaw = yaw;
    complement.angle.last_yaw = yaw;
}

void IMU_0(void)
{
    yaw_total -= RAD_TO_ANGLE(IMU_Data.gyro_z * dt_actual);
}

/* =================================================================================
 *                                  滤波与数学工具函数
 * ================================================================================= */
void IMU_KalmanCreate(extKalman_t *p, float T_Q, float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

float IMU_KalmanFilter(extKalman_t *p, float dat)
{
    p->X_mid = p->A * p->X_last;
    p->P_mid = p->A * p->P_last + p->Q;
    p->kg = p->P_mid / (p->P_mid + p->R);
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid);
    p->P_now = (1 - p->kg) * p->P_mid;
    p->P_last = p->P_now;
    p->X_last = p->X_now;
    return p->X_now;
}

float fast_sqrt(float num)
{
    float halfx = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f375a86 - (i >> 1);

    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    y = y * (1.5f - (halfx * y * y));
    return y;
}

float My_abs(float x)
{
    return x < 0 ? -x : x;
}
