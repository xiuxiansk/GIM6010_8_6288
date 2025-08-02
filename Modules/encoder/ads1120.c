#include "ads1120.h"
#include "general_def.h"
#include "spi.h"
#include "stm32g4xx_hal.h"
#include <stdint.h>
void ADS1220SendByte(uint8_t Byte)
{
    HAL_SPI_Transmit(&ADS1120_SPI_Get_HSPI, &Byte, 1, 10);
}

/*
******************************************************************************
 higher level functions
*/

void ADS1220ReadRegister(int StartAddress, uint8_t NumRegs, uint8_t *pData)
{
    /* send the command byte */
    ADS1220SendByte(ADS1220_CMD_RREG | (((StartAddress << 2) & 0x0c) | ((NumRegs - 1) & 0x03)));
    /* get the register content */
    HAL_SPI_Receive(&ADS1120_SPI_Get_HSPI, pData, NumRegs, 0xFFFF);
    return;
}
void ADS1220WriteRegister(int StartAddress, uint8_t NumRegs, uint8_t *pData)
{
    /* send the command byte */
    ADS1220SendByte(ADS1220_CMD_WREG | (((StartAddress << 2) & 0x0c) | ((NumRegs - 1) & 0x03)));
    /* send the data bytes */
    HAL_SPI_Transmit(&ADS1120_SPI_Get_HSPI, pData, NumRegs, 0xFFFF);
    return;
}
void ADS1220SendResetCommand()
{
    ADS1220SendByte(ADS1220_CMD_RESET);
}
void ADS1220SendStartCommand()
{
    ADS1220SendByte(ADS1220_CMD_SYNC);
}
void ADS1220SendShutdownCommand()
{
    ADS1220SendByte(ADS1220_CMD_SHUTDOWN);
}

/* ADS1220 Initial Configuration */
unsigned int test[4];
void ADS1220Init(void)
{
    HAL_Delay(100);
    uint8_t ch_cfg[4] = {ADS1220_MUX_0_G | ADS1220_PGA_BYPASS, ADS1220_CC | ADS1220_DR_1000 | ADS1220_MODE_TURBO, ADS1220_VREF_INT | ADS1220_PSW_SW_OFF, ADS1220_IDAC1_OFF | ADS1220_IDAC2_OFF};
    ADS1220SendResetCommand(); // 复位

    HAL_Delay(100);
    ADS1220WriteRegister(ADS1220_0_REGISTER, 4, ch_cfg); // 配置4个寄存器
    HAL_Delay(100);

    // ADS1220SendShutdownCommand();
    ADS1220SendStartCommand();
}

float ADS1220ReadVolt(void)
{
    static uint8_t rx_buff[3];
    int32_t temp;
#ifdef ADS1120
    ADS1220SendByte(ADS1220_CMD_RDATA);
    HAL_SPI_Receive(&ADS1120_SPI_Get_HSPI, rx_buff, 2, 0xFFFF);
    temp = (((rx_buff[0] & 0x7F) << 9) | ((rx_buff[0] & 0x80) << 1) | rx_buff[1]);
    return temp * 2.048f / ADS_FULL_SCALE; // 转换为电压 单位V
#else
    HAL_SPI_Receive(&ADS1120_SPI_Get_HSPI, rx_buff, 3, 0xFFFF); // 3字节数据
#endif
}

// 选择通道函数
void ADS1220_SelCh(uint8_t ch)
{
    uint8_t cfg[2] = {ADS1220_MUX_0_G | ADS1220_GAIN_1 | ADS1220_PGA_BYPASS};

    switch (ch) {
        case 0: // 选择通道0
            cfg[0] = ADS1220_MUX_0_G | ADS1220_GAIN_1 | ADS1220_PGA_BYPASS;
            ADS1220WriteRegister(ADS1220_0_REGISTER, 1, cfg);
            break;
        case 1: // 选择通道1
            cfg[0] = ADS1220_MUX_1_G | ADS1220_GAIN_1 | ADS1220_PGA_BYPASS;
            ADS1220WriteRegister(ADS1220_0_REGISTER, 1, cfg);
            break;
        case 2: // 选择通道2
            cfg[0] = ADS1220_MUX_2_G | ADS1220_GAIN_1 | ADS1220_PGA_BYPASS;
            ADS1220WriteRegister(ADS1220_0_REGISTER, 1, cfg);
            break;
        case 3: // 选择通道3
            cfg[0] = ADS1220_MUX_3_G | ADS1220_GAIN_1 | ADS1220_PGA_BYPASS;
            ADS1220WriteRegister(ADS1220_0_REGISTER, 1, cfg);
            break;
    }
}
float mt9105_mag[4] = {0}; // cos_p sin_p sin_n cos_n
float Hall_calc_angle(void)
{
    float sin_mag, cos_mag;
    for (int i = 0; i < 4; i++) {
        ADS1220_SelCh(i);
        HAL_Delay(1);
        mt9105_mag[i] = (ADS1220ReadVolt() - 1.65f) * 317.46f;
    }
    sin_mag     = mt9105_mag[0] - mt9105_mag[3];
    cos_mag     = mt9105_mag[1] - mt9105_mag[2];
    float theta = atan2f(sin_mag, cos_mag);

    // 转换为0-360°范围
    if (theta < 0) theta += M_2PI;
    return theta;
}
