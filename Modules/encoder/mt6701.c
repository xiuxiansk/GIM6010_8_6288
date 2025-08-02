#include "mt6701.h"
#include "bsp_dwt.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_spi.h"
#include "sys/lock.h"

/**
 * @brief 编码器初始化函数
 *
 * 此函数用于初始化编码器相关参数和 SPI 外设。
 *
 * @param None
 */
ENCODER_DATA encoder_data = {
    .pole_pairs     = 0, // 设置极对数为电机参数
    .dir            = 0, // 设置编码器旋转方向为顺时针
    .encoder_offset = 0, // 设置电角度偏移量

    .count_in_cpr        = 0, // 原始值（整数型）
    .count_in_cpr_prev   = 0,
    .shadow_count        = 0,    // 原始值累加计数（整数型）
    .vel_estimate_counts = 0.0f, // 原始值转速
    .pos_cpr_counts      = 0.0f, // 原始值（浮点型）
    .pos_estimate        = 0.0f, // 圈数n
    .vel_estimate        = 0.0f, // 速度rad/s
    .phase               = 0.0f, // 电角度
    .phase_vel           = 0.0f,
    .interpolation       = 0.0f, // 插值系数
    .mec_angle           = 0.0f, // 机械角度
};

/**
 * @brief 将角度归一化到 [0, 2π] 范围内
 *
 * 此函数将给定的角度值归一化到 [0, 2π] 范围内。
 *
 * @param raw 要归一化的角度
 */
float normalize_angle(float raw)
{
    float a = fmod(raw, M_2PI);      // 使用取余运算进行归一化
    return a >= 0 ? a : (a + M_2PI); // 确保返回值在 [0, 2π] 范围内
}

uint8_t encode_buff[3];
void Mt6701_Init(void)
{
    DWT_Delay(0.04f); // MT6701上电启动时间最大32ms
                      // LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
                      // LL_SPI_EnableDMAReq_RX(SPI1);

    // LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, LL_SPI_DMA_GetRegAddr(SPI1));
    // LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)encode_buff);
    // LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
    // LL_SPI_Enable(SPI1);

    // LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 3);
    // LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    MT6701_SPI_CS_L();
    HAL_SPI_Receive_DMA(&hspi1, encode_buff, 3);
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    MT6701_SPI_CS_H();
}
/**
 * @brief 从 MT6701 编码器读取原始角度数据
 *
 * 此函数读取 MT6701 编码器的原始角度值
 *
 * @param encoder 指向包含电机状态和控制数据的结构体指针
 */
bool mt6701_read_raw(ENCODER_DATA *encoder)
{
    // 磁场状态检测
    encoder->mag_status = encode_buff[2] >> 6;
    if (encoder->mag_status) {
        goto CHECK_ERR;
    }
    // 角度读取
    encoder->angle = (encode_buff[0] << 6) | (encode_buff[1] >> 2);
    MT6701_SPI_CS_L();
    HAL_SPI_Receive_DMA(&hspi1, encode_buff, 3);
    return true;

CHECK_ERR:
    if (encoder->check_err_count < 0xFF) {
        encoder->check_err_count++;
    }
    return false;
    // 奇偶校验
    // h_count = 0;
    // for (uint8_t j = 0; j < 16; j++) {
    //     if (rawAngle & (0x01 << j)) {
    //         h_count++;
    //     }
    // }
    // if (h_count & 0x01) {
    //     goto CHECK_ERR;
    // }
}

/**
 * @brief 将读取的原始值转化成实际角度和电角度
 *
 * 此函数将读取的原始值转化成实际角度和电角度。
 *
 * @param encoder 指向包含电机状态和控制数据的结构体指针
 */
void GetMotor_Angle(ENCODER_DATA *encoder)
{
    static const float pll_kp_        = 2.0f * ENCODER_PLL_BANDWIDTH;
    static const float pll_ki_        = 0.25f * SQ(pll_kp_);
    static const float snap_threshold = 0.5f * CURRENT_MEASURE_PERIOD * pll_ki_;
    if (mt6701_read_raw(encoder)) {
        if (encoder->dir == CCW) {
            encoder->raw = (ENCODER_CPR - encoder->angle);
        } else {
            encoder->raw = encoder->angle;
        }
    }
    /* Linearization */
    int32_t off_1      = encoder->offset_lut[encoder->raw >> 7];                                       // lookup table lower entry
    int32_t off_2      = encoder->offset_lut[((encoder->raw >> 7) + 1) % 128];                         // lookup table higher entry
    int32_t off_interp = off_1 + ((off_2 - off_1) * (encoder->raw - ((encoder->raw >> 7) << 7)) >> 7); // Interpolate between lookup table entries
    int32_t count      = encoder->raw - off_interp;
    /*  Wrap in ENCODER_CPR */
    while (count > ENCODER_CPR)
        count -= ENCODER_CPR;
    while (count < 0)
        count += ENCODER_CPR;
    encoder->count_in_cpr = count;
    /* Delta count */
    int32_t delta_count        = encoder->count_in_cpr - encoder->count_in_cpr_prev;
    encoder->count_in_cpr_prev = encoder->count_in_cpr;
    while (delta_count > +ENCODER_CPR_DIV)
        delta_count -= ENCODER_CPR;
    while (delta_count < -ENCODER_CPR_DIV)
        delta_count += ENCODER_CPR;

    /* Add measured delta to encoder count */
    encoder->shadow_count += delta_count;
    /* Run vel PLL */
    encoder->pos_cpr_counts +=
        CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts;
    float delta_pos_cpr_counts =
        (float)(encoder->count_in_cpr - (int32_t)floor(encoder->pos_cpr_counts));
    while (delta_pos_cpr_counts > +ENCODER_CPR_DIV)
        delta_pos_cpr_counts -= ENCODER_CPR_F;
    while (delta_pos_cpr_counts < -ENCODER_CPR_DIV)
        delta_pos_cpr_counts += ENCODER_CPR_F;
    // pll feedback
    encoder->pos_cpr_counts +=
        CURRENT_MEASURE_PERIOD * pll_kp_ * delta_pos_cpr_counts;
    while (encoder->pos_cpr_counts > ENCODER_CPR)
        encoder->pos_cpr_counts -= ENCODER_CPR_F;
    while (encoder->pos_cpr_counts < 0)
        encoder->pos_cpr_counts += ENCODER_CPR_F;
    encoder->vel_estimate_counts +=
        CURRENT_MEASURE_PERIOD * pll_ki_ * delta_pos_cpr_counts;
    // align delta-sigma on zero to prevent jitter
    bool snap_to_zero_vel = false;
    if (ABS(encoder->vel_estimate_counts) < snap_threshold) // 100
    {
        encoder->vel_estimate_counts = 0.0f; // align delta-sigma on zero to prevent jitter
        snap_to_zero_vel             = true;
    }
    //// run encoder count interpolation
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel) {
        encoder->interpolation = 0.5f;
        // reset interpolation if encoder edge comes
        // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    } else if (delta_count > 0) {
        encoder->interpolation = 0.0f;
    } else if (delta_count < 0) {
        encoder->interpolation = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        encoder->interpolation +=
            CURRENT_MEASURE_PERIOD * encoder->vel_estimate_counts;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (encoder->interpolation > 1.0f)
            encoder->interpolation = 1.0f;
        if (encoder->interpolation < 0.0f)
            encoder->interpolation = 0.0f;
    }

    float interpolated_enc =
        encoder->count_in_cpr - encoder->encoder_offset + encoder->interpolation;
    while (interpolated_enc > ENCODER_CPR)
        interpolated_enc -= ENCODER_CPR;
    while (interpolated_enc < 0)
        interpolated_enc += ENCODER_CPR;

    float shadow_count_f = encoder->shadow_count;
    float turns          = shadow_count_f / ENCODER_CPR_F;
    float residual       = shadow_count_f - turns * ENCODER_CPR_F;
    /* Outputs from Encoder for Controller */
    encoder->pos_estimate = normalize_angle(turns + residual / ENCODER_CPR_F);

    UTILS_LP_MOVING_AVG_APPROX(encoder->vel_estimate,
                               (encoder->vel_estimate_counts / ENCODER_CPR_F), 5);
    encoder->phase     = (interpolated_enc * M_2PI * encoder->pole_pairs) / ENCODER_CPR_F;
    encoder->phase_vel = encoder->vel_estimate * M_2PI * encoder->pole_pairs;
}