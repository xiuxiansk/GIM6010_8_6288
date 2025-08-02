#pragma once
#include "main.h"
#include "arm_math.h"
#include "mt6701.h"

// #define DUTY_LIMIT 0.3f
//  电机类型
#define MOTOR_6010_8 1
#define MOTOR_PR60   0

#define N_BASE       6.0f            // 6S电池
#define BATVEL       (4.2f * N_BASE) // 电池电压
#define INVBATVEL    (1.0f / BATVEL) // 电池电压的倒数

#if MOTOR_PR60
// 电机参数相关定义
#define MOTOR_RS                     0.15f    // 相电阻
#define MOTOR_LS                     0.00019f // 相电感
#define MOTOR_FLUX                   0.0f     // 磁链

#define MOTOR_INERTIA                0.0001f                        // 转动惯量 [A/(turn/s^2)]，含义为电机轴以1转每秒的加速度运转时需要提供的用于加速消耗的电流，需要根据电机轴重量和所带负载来进行调试。
#define MOTOR_CURRENT_RAMP_RATE      MOTOR_REF_CURRENT_LIMIT * 1000 // 电流爬升力矩 [Nm/s]
#define MOTOR_VEL_RAMP_RATE          50.0f                          // 转速爬升速度 [(turn/s)/s]
#define MOTOR_TRAJ_VEL               20.0f                          // 梯形轨迹控制模式下最大转速 [turn/s] (设定值应当小于或等于 MOTOR_VEL_LIMIT)
#define MOTOR_TRAJ_ACCEL             5.0f                           // 梯形轨迹控制模式下加速度 [(turn/s)/s]
#define MOTOR_TRAJ_DECEL             5.0f                           // 梯形轨迹控制模式下减速度 [(turn/s)/s]
#define MOTOR_VEL_LIMIT              300.0f                         // 转速限制值 [rad/s] 一般设置为电机最大允许转速
#define MOTOR_VOLTAGE_LIMIT          30.0f                          // 电机电压限制值 [V]
#define MOTOR_FDB_CURRENT_LIMIT      30.0f                          // 反馈电流最大值
#define MOTOR_REF_CURRENT_LIMIT      5.0f                           // 控制电流最大值
#define MOTOR_CURRENT_CTRL_P_GAIN    0.0f                           // 电流环增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_I_GAIN    0.0f                           // 电流环积分增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_BANDWIDTH 1000.0f                        // 电流环带宽[rad/s]，范围 (2~60000)
#define CURRENT_AVG_LEN              12                             // 电流均值滤波长度
#define VOLTAGE_AVG_LEN              20                             // 电压均值滤波长度
#define MOTOR_STALL_CURRENT          15.0f                          // Iq堵转电流，电机堵转时的电流，检测是否过流
#define MOTOR_INPUT_CURRENT          0.0f                           // 默认目标电流
#define MOTOR_INPUT_VELOCITY         0.0f                           // 默认目标转速
#define MOTOR_INPUT_POSITION         0.0f                           // 默认目标位置

// PID参数,调试时先关Q轴，用D轴调节
#define CURRENT_AUTO_CALIBRATION 0 // 手动调整增益还是自动计算
#define MOTOR_IQD_PID_KP_S       0.1f
#define MOTOR_IQD_PID_KI_S       0.05f
#define MOTOR_VEL_PID_KP         0.40f
#define MOTOR_VEL_PID_KI         0.0040f // 不可太大，否则会导致电机激烈抖动，以后想做无感时切换就不顺畅了
#define MOTOR_VEL_PID_KD         0.0f
#define MOTOR_POS_PID_KP         165.0f // 可以看回正时速度是否快准狠
#define MOTOR_POS_PID_KI         0.0f
#define MOTOR_POS_PID_KD         0.0f

#define CURRENT_PID_MAX_OUT      ((BATVEL) / _SQRT3)
#define IQ_PID_MAX_OUT           CURRENT_PID_MAX_OUT
#define IQ_PID_MAX_IOUT          CURRENT_PID_MAX_OUT
#define ID_PID_MAX_OUT           CURRENT_PID_MAX_OUT
#define ID_PID_MAX_IOUT          CURRENT_PID_MAX_OUT
#define VEL_PID_MAX_OUT          MOTOR_REF_CURRENT_LIMIT
#define VEL_PID_MAX_IOUT         MOTOR_REF_CURRENT_LIMIT
#define POS_PID_MAX_OUT          (MOTOR_VEL_LIMIT / 1.3f)
#define POS_PID_MAX_IOUT         (MOTOR_VEL_LIMIT / 1.3f)
#elif MOTOR_6010_8
// 电机参数相关定义
#define MOTOR_RS                     0.42f    // 相电阻
#define MOTOR_LS                     0.00034f // 相电感
#define MOTOR_FLUX                   0.0f     // 磁链

#define MOTOR_INERTIA                0.0001f                        // 转动惯量 [A/(turn/s^2)]，含义为电机轴以1转每秒的加速度运转时需要提供的用于加速消耗的电流，需要根据电机轴重量和所带负载来进行调试。
#define MOTOR_CURRENT_RAMP_RATE      MOTOR_REF_CURRENT_LIMIT * 1000 // 电流爬升力矩 [Nm/s]
#define MOTOR_VEL_RAMP_RATE          50.0f                          // 转速爬升速度 [(turn/s)/s]
#define MOTOR_TRAJ_VEL               20.0f                          // 梯形轨迹控制模式下最大转速 [turn/s] (设定值应当小于或等于 MOTOR_VEL_LIMIT)
#define MOTOR_TRAJ_ACCEL             5.0f                           // 梯形轨迹控制模式下加速度 [(turn/s)/s]
#define MOTOR_TRAJ_DECEL             5.0f                           // 梯形轨迹控制模式下减速度 [(turn/s)/s]
#define MOTOR_VEL_LIMIT              50.0f                          // 转速限制值 [rad/s] 一般设置为电机最大允许转速
#define MOTOR_VOLTAGE_LIMIT          30.0f                          // 电机电压限制值 [V]
#define MOTOR_FDB_CURRENT_LIMIT      30.0f                          // 反馈电流最大值
#define MOTOR_REF_CURRENT_LIMIT      20.0f                          // 控制电流最大值
#define MOTOR_CURRENT_CTRL_P_GAIN    0.0f                           // 电流环增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_I_GAIN    0.0f                           // 电流环积分增益，在校准过程中自动计算得出，也可自行设置 (Auto)
#define MOTOR_CURRENT_CTRL_BANDWIDTH 1000.0f                        // 电流环带宽[rad/s]，范围 (2~60000)
#define MOTOR_STALL_CURRENT          25.0f                          // Iq堵转电流，电机堵转时的电流，检测是否过流
#define MOTOR_INPUT_CURRENT          1.0f                           // 默认目标电流
#define MOTOR_INPUT_VELOCITY         0.0f                           // 默认目标转速
#define MOTOR_INPUT_POSITION         0.0f                           // 默认目标位置

// PID参数,调试时先关Q轴，用D轴调节
#define CURRENT_AUTO_CALIBRATION     0                              // 手动调整增益还是自动计算
#define MOTOR_IQD_PID_KP_S           10.0f
#define MOTOR_IQD_PID_KI_S           0.0f
#define MOTOR_VEL_PID_KP             0.40f
#define MOTOR_VEL_PID_KI             0.0040f // 不可太大，否则会导致电机激烈抖动，以后想做无感时切换就不顺畅了
#define MOTOR_VEL_PID_KD             0.0f
#define MOTOR_POS_PID_KP             165.0f // 可以看回正时速度是否快准狠
#define MOTOR_POS_PID_KI             0.0f
#define MOTOR_POS_PID_KD             0.0f

#define CURRENT_PID_MAX_OUT          ((BATVEL) / _SQRT3)
#define IQ_PID_MAX_OUT               CURRENT_PID_MAX_OUT
#define IQ_PID_MAX_IOUT              CURRENT_PID_MAX_OUT
#define ID_PID_MAX_OUT               CURRENT_PID_MAX_OUT
#define ID_PID_MAX_IOUT              CURRENT_PID_MAX_OUT
#define VEL_PID_MAX_OUT              MOTOR_REF_CURRENT_LIMIT
#define VEL_PID_MAX_IOUT             MOTOR_REF_CURRENT_LIMIT
#define POS_PID_MAX_OUT              (MOTOR_VEL_LIMIT / 1.3f)
#define POS_PID_MAX_IOUT             (MOTOR_VEL_LIMIT / 1.3f)
#endif

// Calib
#define CURRENT_MAX_CALIB    3.0f // 校准最大电流限制 (如果电机校准的时候顿挫感严重或者动一半就不动了，就需要调大这个值)
#define VOLTAGE_MAX_CALIB    6.0f // 校准最大电压限制(如果电机校准的时候，电机没有发出“哔”的声音(就像普通电调启动的声音一样)，就需要调大这个值)

#define OFFSET_LUT_NUM       128            // 偏移量LUT数
#define COGGING_MAP_NUM      3000           // 扇区数
#define CALB_SPEED           M_2PI          // 电机转速
#define MOTOR_POLE_PAIRS_MAX 20             // 最大检测极对数（可以增加但是要注意堆的大小分配）
#define SAMPLES_PER_PPAIR    OFFSET_LUT_NUM // 极对数采样数

// Control
#define BATVEL_MAX_LIMIT (30.0f) // 电池过压报错阈值
#define BATVEL_MIN_LIMIT (5.0f)
#define SPEED_MAX_LIMIT  (MOTOR_VEL_LIMIT * 1.2f) // 超速限制
#define TEMP_MAX_LIMIT   80.0f                    // 红温限制

// 基础配置
#define PWM_MEASURE_PERIOD           (float)(1.0f / (float)PWM_FREQ)           // PWM周期
#define CURRENT_MEASURE_HZ           PWM_FREQ                                  // 电流频率
#define CURRENT_MEASURE_PERIOD       (float)(1.0f / (float)CURRENT_MEASURE_HZ) // 电流周期
#define PWM_ARR                      (uint16_t)(PWM_PERIOD_CLOCKS)
#define VEL_POS_HZ                   8000                              // 速度位置环频率
#define VEL_POS_PERIOD               (float)(1.0f / (float)VEL_POS_HZ) // 速度位置环周期
#define CURRENT_CALIBRATION_DURATION 5000u                             // 电流校准时间
/*24KHzPWM周期41.7us，3路采样保持时间0.9us，死区0.2us，导通时间忽略*/
#define TS 0.95f // 采样周期 (将占空比限制在0-1之间)

// 硬件配置
#define CURRENT_SHUNT_RES     0.001f                                                      // 电流采样电阻
#define CURRENT_AMP_GAIN      40.161f                                                     // 电流放大倍数
#define VIN_R1                10000.0f                                                    // 电压采样分压电阻
#define VIN_R2                100000.0f                                                   // 电压采样分压电阻
#define FAC_CURRENT           ((3.3f / 4095.0f) / (CURRENT_SHUNT_RES * CURRENT_AMP_GAIN)) // 电流放大倍数
#define VOLTAGE_TO_ADC_FACTOR (((VIN_R2 + VIN_R1) / VIN_R1) * (3.3f / 4095.0f))           // 电压放大倍数

// 设置PWM占空比宏定义
#define set_dtc_a(value) TIM1->CCR3 = 0 // 设置A相PWM占空比
#define set_dtc_b(value) TIM1->CCR2 = 0 // 设置B相PWM占空比
#define set_dtc_c(value) TIM1->CCR1 = 0 // 设置C相PWM占空比

#pragma pack(1)
// 扇区枚举
typedef enum {
    SECTOR_1 = 1,
    SECTOR_2,
    SECTOR_3,
    SECTOR_4,
    SECTOR_5,
    SECTOR_6
} svpwm_sector_t;

typedef struct
{
    float vbus; // 母线电压
    float vbus_filt;
    float inv_vbus; // 母线电压倒数，用于计算电流和电压
    float theta;    // 角度
    float i_q_filt, i_d_filt, i_bus, i_bus_filt, power_filt;

    float current_ctrl_integral_d, current_ctrl_integral_q; // Current error integrals
    float current_ctrl_p_gain;                              // (Auto)
    float current_ctrl_i_gain;                              // (Auto)
    // 扇区信息（用于SVPWM）
    int sector; // 当前扇区，用于SVPWM调制

    // 坐标系电压、电流分量
    float i_a; // A 相电流
    float i_b; // B 相电流
    float i_c; // C 相电流

    float v_a; // A 相电压
    float v_b; // B 相电压
    float v_c; // C 相电压

    float i_d; // D 坐标系电流（FOC中的直轴电流）
    float i_q; // Q 坐标系电流（FOC中的交轴电流）

    float v_d; // D 坐标系电压（FOC中的直轴电压） 控制扭矩
    float v_q; // Q 坐标系电压（FOC中的交轴电压） 弱磁控制

    float i_alpha; // Alpha 坐标系电流（Clarke变换后的电流）
    float i_beta;  // Beta  坐标系电流（Clarke变换后的电流）

    float v_alpha; // Alpha 坐标系电压（逆Clarke变换后的电压）
    float v_beta;  // Beta  坐标系电压（逆Clarke变换后的电压）

    float dtc_a; // A 相 PWM 占空比
    float dtc_b; // B 相 PWM 占空比
    float dtc_c; // C 相 PWM 占空比

    float vd_set; // d 轴电压设置
    float vq_set; // q 轴电压设置
    float id_set; // d 轴电流设置
    float iq_set; // q 轴电流设置

    float sin_val; // 电角度的正弦值，用于FOC计算
    float cos_val; // 电角度的余弦值，用于FOC计算
} FOC_DATA;
#pragma pack()

extern FOC_DATA foc_data;

// 函数声明部分
void FOC_reset(FOC_DATA *foc);                                                             // 重置FOC数据
void Foc_Pwm_LowSides(void);                                                               // 启动FOC 但占空比为零
void Foc_Pwm_Start(void);                                                                  // 启动FOC PWM
void Foc_Pwm_Stop(void);                                                                   // 停止FOC PWM
void SetPwm(FOC_DATA *foc);                                                                // 设置PWM占空比
void Sin_Cos_Val(FOC_DATA *foc);                                                           // 计算正弦和余弦值
void Clarke(FOC_DATA *foc);                                                                // Clarke变换
void Park(FOC_DATA *foc);                                                                  // Park变换
void Inv_clarke(FOC_DATA *foc);                                                            // 逆Clarke变换
void Inv_Park(FOC_DATA *foc);                                                              // 逆Park变换
void Svpwm_Midpoint(FOC_DATA *foc);                                                        // SVPWM调制
void Svpwm_Sector(FOC_DATA *foc);                                                          // SVPWM扇区选择
void commonFOCOperations(FOC_DATA *foc);                                                   // 通用FOC操作
void commonInverseFOCOperations(FOC_DATA *foc);                                            // 通用逆FOC操作
void setPhaseVoltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase);              // 设置相电压和相电流
void FOC_voltage(FOC_DATA *foc, float Vd_set, float Vq_set, float phase);                  // 电压控制
void FOC_current(FOC_DATA *foc, float Id_set, float Iq_set, float phase, float phase_vel); // 电流控制                                                                                  // BLDCMotor_H
