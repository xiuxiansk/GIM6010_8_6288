#pragma once
#include "main.h"
#include "my_queue.h"
#define ADC_INJECTED_ENABLE 1 // 启用 ADC 的注入模式

#define adc1_samples        5                          // 单通道采样点数
#define adc1_channel        3                          // 采样通道数
#define adc1_length         adc1_samples *adc1_channel // 数据数

#define adc2_samples        5                          // 单通道采样点数
#define adc2_channel        2                          // 采样通道数
#define adc2_length         adc2_samples *adc2_channel // 数据数

extern uint16_t adc1_dma_value[adc1_samples][adc1_channel];
extern uint16_t adc2_dma_value[adc2_samples][adc2_channel];
extern Queue adc1_queue[adc1_channel];
extern Queue adc2_queue[adc2_channel];
void adc_bsp_init(void);