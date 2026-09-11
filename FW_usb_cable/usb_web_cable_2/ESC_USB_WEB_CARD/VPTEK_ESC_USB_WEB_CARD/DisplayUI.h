#pragma once

#include <Arduino.h>

// Pin mapping copied from the original Spi_Uart_WifiAp Card firmware.
constexpr uint8_t LCD_CS_PIN   = 7;
constexpr uint8_t LCD_RST_PIN  = 2;
constexpr uint8_t LCD_DC_PIN   = 3;
constexpr uint8_t LCD_CLK_PIN  = 4;
constexpr uint8_t LCD_MOSI_PIN = 6;
constexpr uint8_t LCD_VLED_PIN = 10;

constexpr uint8_t KEY_OK_PIN   = 5;
constexpr uint8_t KEY_DOWN_PIN = 8;
constexpr uint8_t KEY_UP_PIN   = 9;

void DisplayUI_Init();
void DisplayUI_Loop();
