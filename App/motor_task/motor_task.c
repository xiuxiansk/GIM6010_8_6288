#include "motor_task.h"
#include "FOCMotor.h"
#include <stdint.h>

/**
 * @brief ADC1 电流采样完成回调函数
 *
 * 电流采样频率24khz
 *
 * @param hadc 传入的 ADC1
 */
uint64_t h_count = 0;
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &ADC_HSPI) {
        h_count++;
        GetMotorADCPhaseCurrent(&motor_data);
        GetMotor_Angle(motor_data.components.encoder);
        MotorStateTask(&motor_data);
    }
}
