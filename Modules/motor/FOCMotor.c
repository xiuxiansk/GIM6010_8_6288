#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "Motor_ADC.h"
#include "bsp_adc.h"
#include "bsp_dwt.h"
#include "main.h"
#include "trapTraj.h"
#include <stdlib.h>

MOTOR_DATA motor_data = {
    .components = {
        .foc     = &foc_data,
        .encoder = &encoder_data,
        .current = &current_data,
    },
    .state = {
        .State_Mode   = STATE_MODE_IDLE,     // 电机运行状态
        .Control_Mode = CONTROL_MODE_TORQUE, // 电机控制模式
        .Sub_State    = SUB_STATE_IDLE,      // 电机主校准状态
        .Cs_State     = CS_STATE_IDLE,       // 电机子校准状态
        .Fault_State  = FAULT_STATE_NORMAL,  // 电机故障状态
    },
    .parameters = {
        .Rs   = MOTOR_RS,   // 相电阻
        .Ls   = MOTOR_LS,   // 相电感
        .flux = MOTOR_FLUX, // 磁链
    },
    .Controller = {
        .inertia                = MOTOR_INERTIA,                // 转动惯量 [A/(turn/s^2)]，含义为电机轴以1转每秒的加速度运转时需要提供的用于加速消耗的电流，需要根据电机轴重量和所带负载来进行调试。
        .torque_ramp_rate       = MOTOR_CURRENT_RAMP_RATE,      // 电流爬升力矩[Nm/s]
        .vel_ramp_rate          = MOTOR_VEL_RAMP_RATE,          // 转速爬升速度 [(turn/s)/s]
        .traj_vel               = MOTOR_TRAJ_VEL,               // 梯形轨迹控制模式下最大转速 [turn/s]
        .traj_accel             = MOTOR_TRAJ_ACCEL,             // 梯形轨迹控制模式下加速度 [(turn/s)/s]
        .traj_decel             = MOTOR_TRAJ_DECEL,             // 梯形轨迹控制模式下减速度 [(turn/s)/s]
        .vel_limit              = MOTOR_VEL_LIMIT,              // 转速限制值 [turn/s]
        .voltage_limit          = MOTOR_VOLTAGE_LIMIT,          // 电机电压限制值 [V]
        .current_limit          = MOTOR_FDB_CURRENT_LIMIT,      // 电机电流限制值 [A]
        .current_ctrl_p_gain    = MOTOR_CURRENT_CTRL_P_GAIN,    // 电流环增益，在校准过程中自动计算得出，也可自行设置 (Auto)
        .current_ctrl_i_gain    = MOTOR_CURRENT_CTRL_I_GAIN,    // 电流环积分增益，在校准过程中自动计算得出，也可自行设置 (Auto)
        .current_ctrl_bandwidth = MOTOR_CURRENT_CTRL_BANDWIDTH, // 电流环带宽[rad/s]，范围 100~2000
        .input_torque           = MOTOR_INPUT_CURRENT,          // 目标电流
        .input_velocity         = MOTOR_INPUT_VELOCITY,         // 目标转速
        .input_position         = MOTOR_INPUT_POSITION,         // 目标位置
        .input_updated          = true,                         // 位置环设定值是否已更新
        .input_torque_q         = 0.0f,                         // 目标电流q轴
        .input_torque_d         = 0.0f,                         // 目标电流d轴
    },
    .IqPID = {
        // 电流环IQ参数
        .mode     = PID_POSITION,
        .Kp       = 0.0f,
        .Ki       = 0.0f,
        .Kd       = 0.0f,
        .max_out  = IQ_PID_MAX_OUT,
        .max_iout = IQ_PID_MAX_IOUT,
    },
    .IdPID = {
        // 电流环ID参数
        .mode     = PID_POSITION,
        .Kp       = 0.0f,
        .Ki       = 0.0f,
        .Kd       = 0.0f,
        .max_out  = ID_PID_MAX_OUT,
        .max_iout = ID_PID_MAX_IOUT,
    },
    .VelPID = {
        // 转速环PID参数
        .mode     = PID_POSITION,
        .Kp       = MOTOR_VEL_PID_KP,
        .Ki       = MOTOR_VEL_PID_KI,
        .Kd       = MOTOR_VEL_PID_KD,
        .max_out  = VEL_PID_MAX_OUT,
        .max_iout = VEL_PID_MAX_IOUT,
    },
    .PosPID = {
        // 位置环PID参数
        .mode     = PID_POSITION,
        .Kp       = MOTOR_POS_PID_KP,
        .Ki       = MOTOR_POS_PID_KI,
        .Kd       = MOTOR_POS_PID_KD,
        .max_out  = POS_PID_MAX_OUT,
        .max_iout = POS_PID_MAX_IOUT,
    },
};

static int *p_error_arr = NULL;

/**
 * @brief 读取NTC温度任务
 *
 * @param motor 指向电机数据结构的指针
 */
void TempResultTask(MOTOR_DATA *motor)
{
    GetTempNtc(adc2_median_filter(adc2_ch12), &motor->components.current->Temp_Result);
}

/**
 * @brief 修改PID限幅值
 *
 * @param motor 指向电机数据结构的指针
 */
static void SetPIDLimit(MOTOR_DATA *motor,
                        float current_max_out,
                        float current_max_iout,
                        float vel_max_out,
                        float vel_max_iout,
                        float pos_limit)
{
    motor->IdPID.max_out   = CURRENT_PID_MAX_OUT;
    motor->IdPID.max_iout  = CURRENT_PID_MAX_OUT;
    motor->IqPID.max_out   = current_max_out;
    motor->IqPID.max_iout  = current_max_iout;
    motor->VelPID.max_out  = vel_max_out;
    motor->VelPID.max_iout = vel_max_iout;
    motor->PosPID.max_out  = pos_limit;
    motor->PosPID.max_iout = pos_limit;
}

/**
 * @brief 速度开环控制任务
 *
 * 此函数用于执行开环控制任务，根据目标速度计算电机的电角度，并设置电流值。
 *
 * @param motor 指向电机数据结构的指针
 */
static void OpenControlMode(MOTOR_DATA *motor, float target_velocity)
{
    float Ts = 0.001f;
    if (Ts <= 0 || Ts > 0.5f)
        Ts = 1e-3f;

    motor->components.foc->theta = normalize_angle(motor->components.foc->theta + target_velocity * Ts); // 固定角度、周期

    FOC_voltage(motor->components.foc, 0.0f, 0.5f, motor->components.foc->theta); // 电压控制
}

/**
 * @brief 获取电机相电流
 *
 * 此函数从 ADC 中读取电机的相电流值，并将其存储在 foc_data 结构体中。
 * 该函数通常在控制循环中调用，以获取实时电流数据。
 *
 * @param motor 指向 foc_data 结构体的指针，该结构体包含电机状态和控制数据。
 */
void GetMotorADCPhaseCurrent(MOTOR_DATA *motor)
{
    // 三相电流偏置、母线电压换算
    motor->components.foc->i_a = ((float)hadc1.Instance->JDR1 - motor->components.current->Ia_offset) * FAC_CURRENT;

    motor->components.foc->i_b = ((float)hadc1.Instance->JDR2 - motor->components.current->Ib_offset) * FAC_CURRENT;

    motor->components.foc->i_c = -motor->components.foc->i_a - motor->components.foc->i_b;

    motor->components.foc->vbus = ((float)hadc2.Instance->JDR1) * VOLTAGE_TO_ADC_FACTOR;
    UTILS_LP_FAST(motor->components.foc->vbus_filt, motor->components.foc->vbus, 0.05f); // 低通滤波母线电压
    motor->components.foc->inv_vbus = 1.0f / motor->components.foc->vbus_filt;           // 母线电压倒数
}

/**
 * @brief 更新电流控制器的增益参数
 *
 * 电流环增益计算
 * P：Ls * （最大转速(rad/s) * 极对数）
 * I：Rs * （最大转速(rad/s) * 极对数）* 电流采样频率
 *
 * @param bandwidth 电流增益
 */
static void FOC_update_current_gain(MOTOR_DATA *motor)
{
#if CURRENT_AUTO_CALIBRATION
    motor->Controller.current_ctrl_p_gain = motor->parameters.Ls * motor->Controller.vel_limit * motor->components.encoder->pole_pairs;
    motor->Controller.current_ctrl_i_gain = motor->parameters.Rs * motor->Controller.vel_limit * motor->components.encoder->pole_pairs * CURRENT_MEASURE_PERIOD;
#else
    motor->Controller.current_ctrl_p_gain = 2 * PI * motor->parameters.Ls * motor->Controller.current_ctrl_bandwidth * MOTOR_IQD_PID_KP_S;
    motor->Controller.current_ctrl_i_gain = 2 * PI * motor->parameters.Rs * motor->Controller.current_ctrl_bandwidth * CURRENT_MEASURE_PERIOD * MOTOR_IQD_PID_KI_S;
#endif
    motor->IdPID.Kp = motor->Controller.current_ctrl_p_gain;
    motor->IdPID.Ki = motor->Controller.current_ctrl_i_gain;
    motor->IqPID.Kp = motor->Controller.current_ctrl_p_gain;
    motor->IqPID.Ki = motor->Controller.current_ctrl_i_gain;
}

void Init_Motor_No_Calib(MOTOR_DATA *motor)
{
    motor->state.Sub_State  = SUB_STATE_IDLE;
    motor->state.Cs_State   = CS_STATE_IDLE;
    motor->state.State_Mode = STATE_MODE_RUNNING;
    FOC_update_current_gain(motor);
}

void Init_Motor_Calib(MOTOR_DATA *motor)
{
    if (p_error_arr == NULL) {
        p_error_arr = malloc(SAMPLES_PER_PPAIR * MOTOR_POLE_PAIRS_MAX * sizeof(int));
    }
    motor->state.Sub_State = RSLS_CALIBRATING; // 改变子状态
    motor->state.Cs_State  = CS_MOTOR_R_START; // 改变子状态
}

/**
 * @brief 电流矫正
 *
 * @param motor 指向包含电机状态和控制数据的结构体指针
 */
static void CurrentCalibration(MOTOR_DATA *motor)
{
    static uint16_t loop_count = 0;
    loop_count++;

    // 累加电流偏移值
    motor->components.current->current_offset_sum_a += (float)(ADC1->JDR1);
    motor->components.current->current_offset_sum_b += (float)(ADC1->JDR2);
    motor->components.current->current_offset_sum_c += (float)(ADC1->JDR3);

    if (loop_count >= CURRENT_CALIBRATION_DURATION) {
        loop_count = 0;

        if (Flash_ReadMotorParam((uint32_t *)(&(motor->components.encoder->pole_pairs)),
                                 &(motor->components.encoder->encoder_offset),
                                 &(motor->components.encoder->dir)) == HAL_OK) {
            Init_Motor_No_Calib(motor);
        } else {
            Init_Motor_Calib(motor);
        }
        // 校准完成，更新电流偏移值
        motor->components.current->Ia_offset = motor->components.current->current_offset_sum_a / (float)CURRENT_CALIBRATION_DURATION;
        motor->components.current->Ib_offset = motor->components.current->current_offset_sum_b / (float)CURRENT_CALIBRATION_DURATION;
        motor->components.current->Ic_offset = motor->components.current->current_offset_sum_c / (float)CURRENT_CALIBRATION_DURATION;

        // 清零偏移量累加器
        motor->components.current->current_offset_sum_a = 0;
        motor->components.current->current_offset_sum_b = 0;
        motor->components.current->current_offset_sum_c = 0;
    }
}
/**
 * @brief 校准电机电阻、电感
 *
 * @param motor 指向包含电机状态和控制数据的结构体指针
 */
static void RSLSCalibration(MOTOR_DATA *motor)
{
    static uint32_t loop_count;

    // R
    static const float kI              = 2.0f;
    static const uint32_t num_R_cycles = CURRENT_MEASURE_HZ * 2;

    // L
    static float Ialphas[2];
    static float voltages[2];
    static const uint32_t num_L_cycles = CURRENT_MEASURE_HZ / 2;

    static const float calib_phase_vel = M_PI;

    static float phase_set;
    static float start_count;
    static int16_t sample_count;
    static float next_sample_time;

    float time          = (float)loop_count * CURRENT_MEASURE_PERIOD;
    const float voltage = CURRENT_MAX_CALIB * motor->parameters.Rs * 3.0f / 2.0f;

    switch (motor->state.Cs_State) {
        case CS_STATE_IDLE: // 空闲状态
            break;

        case CS_MOTOR_R_START: // 设定清空参数
            loop_count                     = 0;
            voltages[0]                    = 0.0f;
            motor->components.encoder->dir = CW;
            motor->state.Cs_State          = CS_MOTOR_R_LOOP;
            break;

        case CS_MOTOR_R_LOOP: // 校准电阻
            voltages[0] += kI * CURRENT_MEASURE_PERIOD * (CURRENT_MAX_CALIB - motor->components.foc->i_a);

            // Test voltage along phase A
            FOC_voltage(motor->components.foc, voltages[0], 0, 0);

            if (loop_count >= num_R_cycles) {
                Foc_Pwm_LowSides();
                motor->state.Cs_State = CS_MOTOR_R_END;
            }

            break;

        case CS_MOTOR_R_END: // 校准电阻完成
            motor->parameters.Rs  = (voltages[0] / CURRENT_MAX_CALIB) * (2.0f / 3.0f);
            motor->state.Cs_State = CS_MOTOR_L_START;

            break;

        case CS_MOTOR_L_START: // 设定清空参数
            loop_count  = 0;
            Ialphas[0]  = 0.0f;
            Ialphas[1]  = 0.0f;
            voltages[0] = -VOLTAGE_MAX_CALIB;
            voltages[1] = +VOLTAGE_MAX_CALIB;
            FOC_voltage(motor->components.foc, voltages[0], 0.0f, 0.0f);
            motor->state.Cs_State = CS_MOTOR_L_LOOP;
            break;

        case CS_MOTOR_L_LOOP: // 校准电感
        {
            int i = loop_count & 1;
            Ialphas[i] += motor->components.foc->i_a;

            // Test voltage along phase A
            FOC_voltage(motor->components.foc, voltages[i], 0.0f, 0.0f);

            if (loop_count >= (num_L_cycles << 1)) {
                Foc_Pwm_LowSides();
                motor->state.Cs_State = CS_MOTOR_L_END;
            }
        } break;

        case CS_MOTOR_L_END: // 校准电感完成
        {
            float dI_by_dt       = (Ialphas[1] - Ialphas[0]) / (float)(CURRENT_MEASURE_PERIOD * num_L_cycles);
            float L              = VOLTAGE_MAX_CALIB / dI_by_dt;
            motor->parameters.Ls = fabsf(L * 2.0f / 3.0f); // FIXME 会出现负数电感量
            FOC_update_current_gain(motor);

            phase_set             = 0;
            loop_count            = 0;
            motor->state.Cs_State = CS_DIR_PP_START;
        } break;

        case CS_DIR_PP_START: // 设定清空参数
            FOC_voltage(motor->components.foc, (voltage * time / 2.0f), 0.0f, phase_set);
            if (time >= 2.0f) {
                start_count           = (float)motor->components.encoder->shadow_count;
                motor->state.Cs_State = CS_DIR_PP_LOOP;
                break;
            }
            break;

        case CS_DIR_PP_LOOP: // 校准方向和电机极对数

            phase_set += calib_phase_vel * CURRENT_MEASURE_PERIOD;
            FOC_voltage(motor->components.foc, voltage, 0.0f, phase_set);
            if (phase_set >= 4.0f * M_2PI) {
                motor->state.Cs_State = CS_DIR_PP_END;
            }
            break;

        case CS_DIR_PP_END: // 校准方向和电机极对数完成
        {
            int32_t diff = motor->components.encoder->shadow_count - start_count;

            // Check direction
            if (diff > 0) {
                motor->components.encoder->dir = CW;
            } else {
                motor->components.encoder->dir = CCW;
            }

            // Motor pole pairs
            motor->components.encoder->pole_pairs = round(4.0f / ABS(diff / ENCODER_CPR_F));

            if (motor->components.encoder->pole_pairs > MOTOR_POLE_PAIRS_MAX) {
                break;
            }

            motor->state.Cs_State = CS_ENCODER_START;
        } break;

        case CS_ENCODER_START: // 设定清空参数
            phase_set             = 0;
            loop_count            = 0;
            sample_count          = 0;
            next_sample_time      = 0;
            motor->state.Cs_State = CS_ENCODER_CW_LOOP;
            break;

        case CS_ENCODER_CW_LOOP: // 校准编码器（正转）
            if (sample_count < (motor->components.encoder->pole_pairs * SAMPLES_PER_PPAIR)) {
                if (time > next_sample_time) {
                    next_sample_time += M_2PI / ((float)SAMPLES_PER_PPAIR * calib_phase_vel);

                    int count_ref = (phase_set * ENCODER_CPR_F) / (M_2PI * (float)motor->components.encoder->pole_pairs);
                    int error     = motor->components.encoder->raw - count_ref;
                    error += ENCODER_CPR * (error < 0);
                    p_error_arr[sample_count] = error;

                    sample_count++;
                }

                phase_set += calib_phase_vel * CURRENT_MEASURE_PERIOD;
            } else {
                phase_set -= calib_phase_vel * CURRENT_MEASURE_PERIOD;
                loop_count = 0;
                sample_count--;
                next_sample_time      = 0;
                motor->state.Cs_State = CS_ENCODER_CCW_LOOP;
                break;
            }
            FOC_voltage(motor->components.foc, voltage, 0, phase_set);
            break;

        case CS_ENCODER_CCW_LOOP: // 校准编码器（反转）
            if (sample_count >= 0) {
                if (time > next_sample_time) {
                    next_sample_time += M_2PI / ((float)SAMPLES_PER_PPAIR * calib_phase_vel);

                    int count_ref = (phase_set * ENCODER_CPR_F) / (M_2PI * (float)motor->components.encoder->pole_pairs);
                    int error     = motor->components.encoder->raw - count_ref;
                    error += ENCODER_CPR * (error < 0);
                    p_error_arr[sample_count] = (p_error_arr[sample_count] + error) / 2;

                    sample_count--;
                }

                phase_set -= calib_phase_vel * CURRENT_MEASURE_PERIOD;
            } else {
                Foc_Pwm_LowSides();
                motor->state.Cs_State = CS_ENCODER_END;
                break;
            }
            FOC_voltage(motor->components.foc, voltage, 0, phase_set);
            break;

        case CS_ENCODER_END: // 校准编码器完成,输出校准结果
        {
            // Calculate average offset
            int64_t moving_avg = 0;
            for (int i = 0; i < (motor->components.encoder->pole_pairs * SAMPLES_PER_PPAIR); i++) {
                moving_avg += p_error_arr[i];
            }
            motor->components.encoder->encoder_offset = moving_avg / (motor->components.encoder->pole_pairs * SAMPLES_PER_PPAIR);
            free(p_error_arr);
            p_error_arr = NULL;

            motor->state.Cs_State   = CS_STATE_IDLE;
            motor->state.Sub_State  = SUB_STATE_IDLE;
            motor->state.State_Mode = STATE_MODE_RUNNING;
            PID_clear(&motor->IqPID);
            PID_clear(&motor->IdPID);
            PID_clear(&motor->VelPID);
            PID_clear(&motor->PosPID);
            /*电阻电感校准不一定正确，影响到电流环参数，暂时定为统一值*/
            motor->parameters.Rs = MOTOR_RS;
            motor->parameters.Ls = MOTOR_LS;
            /*校准数据存入flash*/
            Flash_SaveMotorParam((uint32_t)(motor_data.components.encoder->pole_pairs),
                                 motor_data.components.encoder->encoder_offset,
                                 motor_data.components.encoder->dir);
        } break;

        default:
            break;
    }
    loop_count++;
}

static void MotorInitializeTask(MOTOR_DATA *motor)
{
    switch (motor->state.Sub_State) {
        case SUB_STATE_IDLE: // 空闲状态

            break;
        case CURRENT_CALIBRATING: // 电流矫正
            CurrentCalibration(motor);
            break;
        case RSLS_CALIBRATING: // 电阻与电感矫正
            RSLSCalibration(motor);
            break;
        case FLUX_CALIBRATING: // 磁链矫正
            // 待完善
            break;

        default:
            break;
    }
}

/**
 * @brief FOC控制任务
 *
 * 此函数用于执行FOC（Field Oriented Control）控制任务。根据控制模式，选择相应的控制任务。
 *
 * @param motor 指向电机数据结构的指针
 *
 * 力矩闭环的调节：
 * 1.首先Q轴设定值为0，给D轴设定值为一个值，然后计算好pid之后查看id波形，发现id会有一个阶跃响应，观察是否为我们的设定值
 * 2.再将Q轴设置为合适值，D轴设置为0，用手捏住看看响应情况
 *
 * 速度闭环的调节：
 * 1.首先kp与ki不要相差太近，不然电机的电流阶跃就会不稳定，会出现严重抖动
 * 2.kp调节要时电机不出现抖动，并且速度响应要灵敏，ki要比kp小几个量级
 *
 * 位置闭环的调节：
 * 1.看相应速度以及是否有过冲
 */
#define TORQUE_ADJUST 0 // 力矩闭环D轴调节
static void MotorControlTask(MOTOR_DATA *motor)
{
    static uint8_t loop_count;
    static float vel_set;

    switch (motor->state.Control_Mode) {
        case CONTROL_MODE_OPEN: // 速度电压开环
        {
            OpenControlMode(motor, 5);
        } break;

        case CONTROL_MODE_TORQUE: // 力矩闭环（空载时电流小，拿手捏住堵转时电流达到目标电流）
        {
            SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, 0.0f, 0.0f, 0.0f);
            motor->components.foc->theta = motor->components.encoder->phase;
#if TORQUE_ADJUST
            motor->components.foc->iq_set = 0;
            motor->components.foc->id_set = 0.5f;
#else
            // motor->Controller.input_torque = CLAMP(motor->Controller.input_torque, -motor->Controller.current_limit, motor->Controller.current_limit);
            // float max_step_size            = fabs(CURRENT_MEASURE_PERIOD * motor->Controller.torque_ramp_rate);
            // float full_step                = motor->Controller.input_torque - motor->components.foc->iq_set;
            // float step                     = CLAMP(full_step, -max_step_size, max_step_size);
            // motor->components.foc->iq_set += step;
            motor->components.foc->iq_set = motor->Controller.input_torque;
            motor->components.foc->id_set = 0.0f;
#endif
        } break;

        case CONTROL_MODE_VELOCITY: // 速度->力矩闭环
        {
            SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, 0.0f);
            motor->components.foc->theta = motor->components.encoder->phase;
        } break;

        case CONTROL_MODE_POSITION: // 位置->速度->力矩闭环
        {
            SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, POS_PID_MAX_OUT);
            motor->components.foc->theta = motor->components.encoder->phase;
        } break;

        case CONTROL_MODE_VELOCITY_RAMP: // 速度->力矩梯度闭环
        {
            SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, POS_PID_MAX_OUT);
            motor->components.foc->theta = motor->components.encoder->phase;
            float max_step_size          = fabs(VEL_POS_PERIOD * motor->Controller.vel_ramp_rate);
            float full_step              = motor->Controller.input_velocity - motor->Controller.vel_setpoint;
            float step                   = CLAMP(full_step, -max_step_size, max_step_size);
            motor->Controller.vel_setpoint += step;
            motor->Controller.torque_setpoint = (step / VEL_POS_PERIOD) * motor->Controller.inertia;
        } break;

        case CONTROL_MODE_POSITION_RAMP: // 位置->速度->力矩梯度闭环
        {
            SetPIDLimit(motor, CURRENT_PID_MAX_OUT, CURRENT_PID_MAX_OUT, VEL_PID_MAX_OUT, VEL_PID_MAX_OUT, POS_PID_MAX_OUT);
            motor->components.foc->theta = motor->components.encoder->phase;
            if (motor->Controller.input_updated) {
                TRAJ_plan(motor->Controller.input_position,
                          motor->components.encoder->pos_estimate,
                          motor->components.encoder->vel_estimate,
                          motor->Controller.traj_vel,    // Velocity
                          motor->Controller.traj_accel,  // Acceleration
                          motor->Controller.traj_decel); // Deceleration

                Traj.t                          = 0.0f;
                Traj.trajectory_done            = false;
                motor->Controller.input_updated = false;
            }

            // Avoid updating uninitialized trajectory
            if (Traj.trajectory_done) {
                break;
            }

            if (Traj.t > Traj.Tf_) {
                Traj.trajectory_done              = true;
                motor->Controller.pos_setpoint    = motor->Controller.input_position;
                motor->Controller.vel_setpoint    = 0.0f;
                motor->Controller.torque_setpoint = 0.0f;
            } else {
                TRAJ_eval(Traj.t);
                motor->Controller.pos_setpoint    = Traj.Y;
                motor->Controller.vel_setpoint    = Traj.Yd;
                motor->Controller.torque_setpoint = Traj.Ydd * motor->Controller.inertia;
                if (fabs(motor->Controller.pos_setpoint - motor->components.encoder->pos_estimate) < 1.0f) {
                    Traj.t += VEL_POS_PERIOD;
                }
            }
        } break;

        default:
            break;
    }

    // 电流环24khz，速度与位置8khz
    if (motor->state.Control_Mode > CONTROL_MODE_OPEN) {
        commonFOCOperations(motor->components.foc); // 计算反馈电流

        if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY && motor->state.Control_Mode <= CONTROL_MODE_POSITION) {
            if (++loop_count >= 4) {
                loop_count = 0;

                if (motor->state.Control_Mode == CONTROL_MODE_POSITION) {
                    vel_set = PID_Calc(&motor->PosPID, motor->components.encoder->pos_estimate, motor->Controller.input_position);
                    vel_set = CLAMP(vel_set, -motor->Controller.vel_limit, +motor->Controller.vel_limit);
                } else if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY) {
                    vel_set = motor->Controller.input_velocity;
                }
                motor->components.encoder->vel_estimate = CLAMP(motor->components.encoder->vel_estimate, -motor->Controller.vel_limit, +motor->Controller.vel_limit);
                motor->components.foc->iq_set           = PID_Calc(&motor->VelPID, motor->components.encoder->vel_estimate, vel_set);
                motor->components.foc->id_set           = 0.0f;
            }
        } else if (motor->state.Control_Mode >= CONTROL_MODE_VELOCITY_RAMP) {
            if (++loop_count >= 4) {
                loop_count = 0;

                if (motor->state.Control_Mode == CONTROL_MODE_POSITION_RAMP) {
                    vel_set = PID_Calc(&motor->PosPID, motor->components.encoder->pos_estimate, motor->Controller.pos_setpoint) + motor->Controller.vel_setpoint;
                    vel_set = CLAMP(vel_set, -motor->Controller.traj_vel, +motor->Controller.traj_vel);
                } else if (motor->state.Control_Mode == CONTROL_MODE_VELOCITY_RAMP) {
                    vel_set = motor->Controller.vel_setpoint;
                }
                motor->components.encoder->vel_estimate = CLAMP(motor->components.encoder->vel_estimate, -motor->Controller.vel_limit, +motor->Controller.vel_limit);
                motor->components.foc->iq_set           = PID_Calc(&motor->VelPID, motor->components.encoder->vel_estimate, vel_set) + motor->Controller.torque_setpoint;
                motor->components.foc->id_set           = 0.0f;
            }
        }

        motor->components.foc->iq_set   = CLAMP(motor->components.foc->iq_set, -motor->Controller.current_limit, +motor->Controller.current_limit);
        motor->components.foc->i_q_filt = CLAMP(motor->components.foc->i_q_filt, -motor->Controller.current_limit, +motor->Controller.current_limit);
        motor->components.foc->v_q      = PID_Calc(&motor->IqPID, motor->components.foc->i_q_filt, motor->components.foc->iq_set);
        motor->components.foc->v_d      = PID_Calc(&motor->IdPID, motor->components.foc->i_d_filt, motor->components.foc->id_set);
        commonInverseFOCOperations(motor->components.foc);
    }
}

/**
 * @brief 电机状态控制任务
 *
 * 电流环24khz，速度与位置8khz
 *
 * @param motor 指向包含电机状态和控制数据的结构体指针
 */
void MotorStateTask(MOTOR_DATA *motor)
{
    switch (motor->state.State_Mode) {
        case STATE_MODE_IDLE: // 空闲模式
            PID_clear(&motor->IqPID);
            PID_clear(&motor->IdPID);
            PID_clear(&motor->VelPID);
            PID_clear(&motor->PosPID);
            FOC_reset(motor->components.foc);
            Foc_Pwm_LowSides();
            motor->state.State_Mode = STATE_MODE_DETECTING;
            motor->state.Sub_State  = CURRENT_CALIBRATING;
            break;
        case STATE_MODE_DETECTING: // 电机矫正模式
            MotorInitializeTask(motor);
            break;
        case STATE_MODE_RUNNING: // 运行模式
            MotorControlTask(motor);
            break;
        case STATE_MODE_GUARD: // 守护模式
            PID_clear(&motor->IqPID);
            PID_clear(&motor->IdPID);
            PID_clear(&motor->VelPID);
            PID_clear(&motor->PosPID);
            FOC_reset(motor->components.foc);
            Foc_Pwm_LowSides();
            break;
        default:;
    }
}
