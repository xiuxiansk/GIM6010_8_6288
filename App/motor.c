#include "motor.h"
#include "cmd_task.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_it.h"
#include "tim.h"
#include "vofa.h"
#include "cmd_task.h"
#include "openloop.h"
#include "bsp_led.h"

void MotorGuardTask(MOTOR_DATA *motor);

void MotorInit(void)
{
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用DWT_Delay()
    __disable_irq();
    LED_Init();
    DWT_Init(168); // 初始化DWT
    Mt6701_Init();
    // LogInit(&huart3); // 初始化日志
    opamp_bsp_init();              // 初始化OPAMP，配置OPAMP硬件
    adc_bsp_init();                // 初始化ADC，配置ADC硬件
    Foc_Pwm_Start();               // foc启动PWM输出
    MotorCMDInit();                // 初始化FDCAN
    HAL_TIM_Base_Start_IT(&htim4); // 守护线程中断
    HAL_TIM_Base_Start_IT(&htim7); // 命令线程中断
    __enable_irq();                // 初始化完成,开启中断
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {
        // 1000Hz
        MotorCMDTask();
    } else if (htim->Instance == TIM7) {
        // 200Hz
        MotorGuardTask(&motor_data);
    }
}

/**
 * @brief 电机守护任务
 *
 * 此函数用于执行电机的守护任务。
 *
 * @param motor 指向包含电机状态和控制数据的结构体指针
 */
void MotorGuardTask(MOTOR_DATA *motor)
{
    if (motor->state.State_Mode == STATE_MODE_IDLE || motor->state.State_Mode == STATE_MODE_DETECTING) {
        LED_Update(WS2812_YELLOW); // 亮黄灯显示正在校准
    } else if (motor->state.State_Mode == STATE_MODE_RUNNING) {
        static int16_t loop_count, led_num;
        // 绿灯闪烁
        if (led_num < can_tx_id) {
            if (loop_count < 400.0 / can_tx_id / 2.0) {
                LED_Update(WS2812_GREEN);
            } else {
                LED_Update(WS2812_OFF);
                if (loop_count >= 400.0 / can_tx_id) {
                    loop_count = 0;
                    led_num++;
                }
            }
        } else {
            if (loop_count < 200) {
                LED_Update(WS2812_OFF);
            } else {
                loop_count = 0;
                led_num    = 0;
            }
        }
        loop_count++;

        // 检查电池电压是否过压或欠压
        if (motor->components.foc->vbus_filt <= BATVEL_MIN_LIMIT) {
            motor->state.Fault_State = FAULT_STATE_UNDER_VOLTAGE;
            motor->state.State_Mode  = STATE_MODE_GUARD;
        } else if (motor->components.foc->vbus_filt >= BATVEL_MAX_LIMIT) {
            motor->state.Fault_State = FAULT_STATE_OVER_VOLTAGE;
            motor->state.State_Mode  = STATE_MODE_GUARD;
        }
        // 检查编码器状态
        if ((motor->components.encoder->check_err_count == 0xFF) || (motor->components.encoder->rx_err_count == 0xFF)) {
            motor->state.Fault_State = FAULT_STATE_MAG;
            motor->state.State_Mode  = STATE_MODE_GUARD;
        }

        // // 检查电机电流是否过流
        // else if ((motor->components.foc->i_q >= MOTOR_STALL_CURRENT) || (motor->components.foc->i_q <= -MOTOR_STALL_CURRENT)) {
        //     motor->state.Fault_State = FAULT_STATE_OVER_CURRENT;
        //     motor->state.State_Mode  = STATE_MODE_GUARD;
        // }

        // // 检查mos温度是否过热
        // else if (motor->components.current->Temp_Result >= TEMP_MAX_LIMIT) {
        //     motor->state.Fault_State = FAULT_STATE_OVER_TEMPERATURE;
        //     motor->state.State_Mode  = STATE_MODE_GUARD;
        // }
    } else if (motor->state.State_Mode == STATE_MODE_GUARD) {
        // 亮红灯显示故障
        LED_Update(WS2812_RED);
    }
}
