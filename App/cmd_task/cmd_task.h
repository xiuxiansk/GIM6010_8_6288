/********************************************************************************
 * @file        : cmd_task.h
 * @author      : Zhang jia ming
 * @brief       : 发布&订阅应用任务
 * @version     : V1.0
 * @date        : 2024 - 11 - 12
 *
 * @details:
 *  - 发布&订阅应用任务
 *
 * @note:
 *  - 无
 *
 * @history:
 *  V1.0:
 *    - 修改注释，完善功能
 *
 * Copyright (c) 2024 Zhang jia ming. All rights reserved.
 ********************************************************************************/

#ifndef Motor_CMD_H
#define Motor_CMD_H
#include "main.h"
extern uint32_t can_tx_id;
/**
 * @brief 机器人核心控制任务初始化,会被MotorInit()调用
 */
void MotorCMDInit(void);

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 */
void MotorCMDTask(void);

#endif // Motor_CMD_H
