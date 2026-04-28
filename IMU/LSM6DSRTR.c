/*
    Hardware By Link
    Software By Link
    2026.03.01
    
*/

#include "LSM6DSRTR.h"
#include "spi.h"

lsm6dsrtr_data_t lsm6dsrtr_data = {0};                                         // LSM6DSRTR 数据结构体
float lsm6dsrtr_transition_factor[2] = {4098.0f, 14.3f};                      // 转换实际值的比例

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     LSM6DSRTR 写寄存器
// 参数说明     reg             寄存器地址
// 参数说明     data            数据
// 返回参数     void
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void lsm6dsrtr_write_acc_gyro_register (uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg & 0x7F; // 写操作: MSB = 0
    buf[1] = data;
    LSM6DSRTR_CS(0);
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    LSM6DSRTR_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     LSM6DSRTR 读寄存器
// 参数说明     reg             寄存器地址
// 返回参数     uint8_t         数据
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t lsm6dsrtr_read_acc_gyro_register (uint8_t reg)
{
    uint8_t reg_addr = reg | 0x80; // 读操作: MSB = 1
    uint8_t data = 0;
    LSM6DSRTR_CS(0);
    HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &data, 1, HAL_MAX_DELAY);
    LSM6DSRTR_CS(1);
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     LSM6DSRTR 读多字节数据
// 参数说明     reg             寄存器地址
// 参数说明     data            数据缓冲区
// 参数说明     len             数据长度
// 返回参数     void
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static void lsm6dsrtr_read_acc_gyro_registers (uint8_t reg, uint8_t *data, uint32_t len)
{
    uint8_t reg_addr = reg | 0x80; // 读操作: MSB = 1
    LSM6DSRTR_CS(0);
    HAL_SPI_Transmit(&hspi1, &reg_addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
    LSM6DSRTR_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     LSM6DSRTR 六轴自检
// 返回参数     uint8_t         1-自检失败 0-自检成功
// 备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint8_t lsm6dsrtr_acc_gyro_self_check (void)
{
    uint8_t return_state = 0;
    uint8_t dat = 0;
    uint16_t timeout_count = 0;

    while(0x6B != dat)                                                          // 判断 ID 是否正确
    {
        if(LSM6DSRTR_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state = 1;
            break;
        }
        dat = lsm6dsrtr_read_acc_gyro_register(LSM6DSRTR_WHO_AM_I);
        HAL_Delay(10);
    }
    return return_state;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 LSM6DSRTR 加速度计数据并转换为 g
//-------------------------------------------------------------------------------------------------------------------
void lsm6dsrtr_get_acc (void)
{
    uint8_t dat[6];
    int16_t raw_x, raw_y, raw_z;

    lsm6dsrtr_read_acc_gyro_registers(LSM6DSRTR_OUTX_L_A, dat, 6);
    
    raw_x = (int16_t)(((uint16_t)dat[1]<<8 | dat[0]));
    raw_y = (int16_t)(((uint16_t)dat[3]<<8 | dat[2]));
    raw_z = (int16_t)(((uint16_t)dat[5]<<8 | dat[4]));

    lsm6dsrtr_data.acc_x = (float)raw_x / lsm6dsrtr_transition_factor[0];
    lsm6dsrtr_data.acc_y = (float)raw_y / lsm6dsrtr_transition_factor[0];
    lsm6dsrtr_data.acc_z = (float)raw_z / lsm6dsrtr_transition_factor[0];
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 LSM6DSRTR 陀螺仪数据并转换为 °/s
//-------------------------------------------------------------------------------------------------------------------
void lsm6dsrtr_get_gyro (void)
{
    uint8_t dat[6];
    int16_t raw_x, raw_y, raw_z;

    lsm6dsrtr_read_acc_gyro_registers(LSM6DSRTR_OUTX_L_G, dat, 6);
    
    raw_x = (int16_t)(((uint16_t)dat[1]<<8 | dat[0]));
    raw_y = (int16_t)(((uint16_t)dat[3]<<8 | dat[2]));
    raw_z = (int16_t)(((uint16_t)dat[5]<<8 | dat[4]));

    lsm6dsrtr_data.gyro_x = (float)raw_x / lsm6dsrtr_transition_factor[1];
    lsm6dsrtr_data.gyro_y = (float)raw_y / lsm6dsrtr_transition_factor[1];
    lsm6dsrtr_data.gyro_z = (float)raw_z / lsm6dsrtr_transition_factor[1];
}



//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 LSM6DSRTR
//-------------------------------------------------------------------------------------------------------------------
uint8_t lsm6dsrtr_init (void)
{
    uint8_t return_state = 0;
    HAL_Delay(10);                                                        // 上电延时

    do
    {
        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_FUNC_CFG_ACCESS, 0x00);       // 关闭HUB寄存器访问
        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL3_C, 0x01);               // 复位设备
        HAL_Delay(2);
        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_FUNC_CFG_ACCESS, 0x00);       // 关闭HUB寄存器访问
        if(lsm6dsrtr_acc_gyro_self_check())
        {
            return_state = 1;
            break;
        }

        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_INT1_CTRL, 0x03);             // 开启陀螺仪 加速度数据就绪中断

        switch(LSM6DSRTR_ACC_SAMPLE_DEFAULT)
        {
            default:
            {
                return_state = 1;
            }break;
            case LSM6DSRTR_ACC_SAMPLE_SGN_2G:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL1_XL, 0x30);
                lsm6dsrtr_transition_factor[0] = 16393.0f;
            }break;
            case LSM6DSRTR_ACC_SAMPLE_SGN_4G:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL1_XL, 0x38);
                lsm6dsrtr_transition_factor[0] = 8197.0f;
            }break;
            case LSM6DSRTR_ACC_SAMPLE_SGN_8G:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL1_XL, 0x3C);
                lsm6dsrtr_transition_factor[0] = 4098.0f;
            }break;
            case LSM6DSRTR_ACC_SAMPLE_SGN_16G:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL1_XL, 0x34);
                lsm6dsrtr_transition_factor[0] = 2049.0f;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        switch(LSM6DSRTR_GYRO_SAMPLE_DEFAULT)
        {
            default:
            {
                return_state = 1;
            }break;
            case LSM6DSRTR_GYRO_SAMPLE_SGN_125DPS:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL2_G, 0x52);
                lsm6dsrtr_transition_factor[1] = 228.6f;
            }break;
            case LSM6DSRTR_GYRO_SAMPLE_SGN_250DPS:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL2_G, 0x50);
                lsm6dsrtr_transition_factor[1] = 114.3f;
            }break;
            case LSM6DSRTR_GYRO_SAMPLE_SGN_500DPS:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL2_G, 0x54);
                lsm6dsrtr_transition_factor[1] = 57.1f;
            }break;
            case LSM6DSRTR_GYRO_SAMPLE_SGN_1000DPS:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL2_G, 0x58);
                lsm6dsrtr_transition_factor[1] = 28.6f;
            }break;
            case LSM6DSRTR_GYRO_SAMPLE_SGN_2000DPS:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL2_G, 0x5C);
                lsm6dsrtr_transition_factor[1] = 14.3f;
            }break;
            case LSM6DSRTR_GYRO_SAMPLE_SGN_4000DPS:
            {
                lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL2_G, 0x51);
                lsm6dsrtr_transition_factor[1] = 7.1f;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL3_C, 0x44);                   // 使能陀螺仪数字低通滤波器
        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL4_C, 0x02);                   // 使能数字低通滤波器
        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL5_C, 0x00);                   // 加速度计与陀螺仪四舍五入
        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL6_C, 0x00);                   // 开启加速度计高性能模式 陀螺仪低通滤波 133hz
        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL7_G, 0x00);                   // 开启陀螺仪高性能模式 关闭高通滤波
        lsm6dsrtr_write_acc_gyro_register(LSM6DSRTR_CTRL9_XL, 0x01);                  // 关闭I3C接口

    }while(0);
    return return_state;
}
