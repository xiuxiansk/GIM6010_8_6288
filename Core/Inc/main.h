/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "opamp.h"
#include "adc.h"
#include "pid.h"
#include "spi.h"
#include "usart.h"

#include "general_def.h"
#include "bsp_can.h"
#include "bsp_opamp.h"
#include "bsp_adc.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usart.h"

#include "BLDCMotor.h"
#include "my_queue.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_TGRO_TIME PWM_PERIOD_CLOCKS-10
#define PWM_DEADTIM_CLOCKS 34
#define PWM_RCR 0
#define PWM_PERIOD_CLOCKS PWM_CLOCK_HZ/2/PWM_FREQ
#define PWM_FREQ 24000
#define PWM_CLOCK_HZ 168000000
#define DRDY_Pin LL_GPIO_PIN_13
#define DRDY_GPIO_Port GPIOC
#define IC_Pin LL_GPIO_PIN_3
#define IC_GPIO_Port GPIOA
#define MOTOR_TEMP_SP_Pin LL_GPIO_PIN_4
#define MOTOR_TEMP_SP_GPIO_Port GPIOA
#define VBUS_SP_Pin LL_GPIO_PIN_7
#define VBUS_SP_GPIO_Port GPIOA
#define ENCODER_CS_Pin LL_GPIO_PIN_4
#define ENCODER_CS_GPIO_Port GPIOC
#define IB_Pin LL_GPIO_PIN_1
#define IB_GPIO_Port GPIOB
#define IA_Pin LL_GPIO_PIN_12
#define IA_GPIO_Port GPIOB
#define INH_C_Pin LL_GPIO_PIN_13
#define INH_C_GPIO_Port GPIOB
#define INH_B_Pin LL_GPIO_PIN_14
#define INH_B_GPIO_Port GPIOB
#define INH_A_Pin LL_GPIO_PIN_15
#define INH_A_GPIO_Port GPIOB
#define INH_CA8_Pin LL_GPIO_PIN_8
#define INH_CA8_GPIO_Port GPIOA
#define INH_BA9_Pin LL_GPIO_PIN_9
#define INH_BA9_GPIO_Port GPIOA
#define INH_AA10_Pin LL_GPIO_PIN_10
#define INH_AA10_GPIO_Port GPIOA
#define WS2812_Pin LL_GPIO_PIN_15
#define WS2812_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
