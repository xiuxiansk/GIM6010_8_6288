#pragma once
#include "main.h"

#define WS2812_RED    0x1000
#define WS2812_GREEN  0x100000
#define WS2812_BLUE   0x10
#define WS2812_YELLOW 0x101000
#define WS2812_OFF    0x0

void LED_Init(void);
void LED_Update(uint32_t color);
void Set_LED(uint8_t Red, uint8_t Green, uint8_t Blue, uint8_t brightness);