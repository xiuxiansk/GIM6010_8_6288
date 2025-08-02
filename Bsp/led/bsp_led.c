#include "bsp_led.h"
#include <stdint.h>

uint8_t LED_Data[3];
uint8_t LED_Mod[3]; // 用于亮度控制
static uint16_t pwmData[(24) + 50] = {0};

void LED_Init(void)
{
    HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint32_t *)pwmData, 74);
}
/*亮度0-255*/
void Set_LED(uint8_t Red, uint8_t Green, uint8_t Blue, uint8_t brightness)
{

    Red   = (Red < 0) ? 0 : (Red > 255) ? 255
                                        : Red;
    Green = (Green < 0) ? 0 : (Green > 255) ? 255
                                            : Green;
    Blue  = (Blue < 0) ? 0 : (Blue > 255) ? 255
                                          : Blue;

    LED_Data[0] = Green;
    LED_Data[1] = Red;
    LED_Data[2] = Blue;

    brightness = (brightness > 100) ? 100 : (brightness < 0) ? 0
                                                             : brightness;
    for (uint8_t i = 0; i < 3; i++) {
        LED_Mod[i] = (LED_Data[i] * brightness) / 255;
    }
    uint32_t indx = 0;
    uint32_t color;

    color = ((LED_Mod[0] << 16) | (LED_Mod[1] << 8) | (LED_Mod[2]));

    for (int j = 23; j >= 0; j--) {
        if (color & (1 << j)) {
            pwmData[indx] = 43; // T1H,高电平持续时间
        } else {
            pwmData[indx] = 17; // T0H,低电平持续时间
        }
        indx++;
    }
}
void LED_Update(uint32_t color)
{
    uint32_t indx = 0;

    for (int j = 23; j >= 0; j--) {
        if (color & (1 << j)) {
            pwmData[indx] = 43; // T1H,高电平持续时间
        } else {
            pwmData[indx] = 17; // T0H,低电平持续时间
        }
        indx++;
    }
}