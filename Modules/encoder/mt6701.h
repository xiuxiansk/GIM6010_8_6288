#pragma once
#include "main.h"

// SPI句柄定义
#define MT6701_SPI_Get_HSPI (hspi1) // 获取SPI句柄

#define MT6701_SPI_CS_L()   HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_RESET)
#define MT6701_SPI_CS_H()   HAL_GPIO_WritePin(ENCODER_CS_GPIO_Port, ENCODER_CS_Pin, GPIO_PIN_SET)

// 常量定义
#define ENCODER_PLL_BANDWIDTH 2000.0f        // 5倍电角频率
#define MIN_VEL               0.1            // counts/s
#define ENCODER_CPR           (int32_t)16384 // 编码器分辨率u
#define ENCODER_CPR_F         16384.0f       // 编码器分辨率f
#define ENCODER_CPR_DIV       (ENCODER_CPR >> 1)
#define MAX_ANGLE             360.0f // 360度为一圈
#define MAX_ANGLE_HALF        180.0f // 180度

// 方向枚举类型定义
typedef enum {
    CW      = 1,  // 顺时针方向
    CCW     = -1, // 逆时针方向
    UNKNOWN = 0   // 未知或无效状态
} Direction;

typedef struct
{
    int32_t angle;
    int32_t raw; // 磁编码器的原始计数值
    uint8_t mag_status;
    uint8_t rx_err_count;
    uint8_t check_err_count;

    int32_t offset_lut[128]; // 用于磁编码器线性化的查找表

    // 角度信息
    uint8_t pole_pairs;     // 电机极对数，用于计算电角度
    int32_t dir;            // 磁编码器的旋转方向，+1 表示顺时针，-1 表示逆时针
    int32_t encoder_offset; // offset

    int32_t count_in_cpr;
    int32_t count_in_cpr_prev;
    int32_t shadow_count;
    float vel_estimate_counts;
    float pos_cpr_counts;
    float pos_estimate;
    float vel_estimate;
    float phase;
    float phase_vel;
    float interpolation;
    float mec_angle;

} ENCODER_DATA;

extern ENCODER_DATA encoder_data;

float normalize_angle(float angle); // 角度归一化函数
void Mt6701_Init(void);
void GetMotor_Angle(ENCODER_DATA *encoder); // 更新编码器数据函数