#include "bsp_opamp.h"
// 包含 OPAMP相关的 BSP（板级支持包）头文件

/**
 * @brief 初始化 OPAMP BSP
 *
 * 此函数用于初始化 OPAMP 的相关设置。
 */
void opamp_bsp_init(void)
{
    // 配置 OPAMP2
    // HAL_OPAMP_SelfCalibrate(&hopamp2);
    HAL_OPAMP_Start(&hopamp2);
}
