/**
  ******************************************************************************
  * @file    oled.h
  * @brief   软件 I²C 驱动 SSD1306 OLED（128x64，4 行 × 16 字符，8x16 字模）
  *
  * 硬件接线（4 针 I²C OLED 模块）：
  *     VCC —— 3.3V
  *     GND —— GND（必须与单片机共地）
  *     SCL —— PB1
  *     SDA —— PB0
  * 说明：软件 I²C 用普通 GPIO 模拟时序，因此 PB0/PB1 不需要在 CubeMX 里配置，
  *       由本驱动的 OLED_Init() 负责使能 GPIOB 时钟并配置引脚。
  ******************************************************************************
  */

#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* ========================== 软件 I²C 引脚定义 ==========================
 * 接线不同时只改这里即可（SDA/SCL 不要接反）。
 * 模块板上一般自带 4.7k~10k 上拉电阻；SDA 这里配置为开漏输出，便于读应答。
 */
#define OLED_SCL_PIN     GPIO_PIN_1
#define OLED_SCL_PORT    GPIOB
#define OLED_SDA_PIN     GPIO_PIN_0
#define OLED_SDA_PORT    GPIOB

/* 屏幕参数 */
#define OLED_WIDTH       128u   /* 横向像素 */
#define OLED_HEIGHT      64u    /* 纵向像素 */
#define OLED_PAGES       8u     /* 页数 = 64 / 8 */

/**
  * @brief  初始化软件 I²C 引脚与 SSD1306，并清屏、开显示
  * @note   上电后建议先 HAL_Delay(20) 等 OLED 稳定再调用
  */
void OLED_Init(void);

/**
  * @brief  清屏（全部像素熄灭）
  */
void OLED_Clear(void);

/**
  * @brief  点亮自检：整屏全亮 / 全灭
  * @param  on 1 = 全屏点亮（白）；0 = 全屏熄灭（黑）
  * @note   用于首次上板确认"屏 + I²C 链路"：调用 OLED_TestPattern(1)，
  *         屏幕应全白；若仍全黑，说明 I²C 不通（查接线/地址/上拉）。
  */
void OLED_TestPattern(uint8_t on);

/**
  * @brief  在指定行、列显示字符串（8x16 字模）
  * @param  Line   起始行：1 ~ 4
  * @param  Column 起始列：1 ~ 16（每个字符占 1 列，横向 8 像素）
  * @param  String 要显示的 ASCII 字符串（超出屏幕的部分自动截断）
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String);

/**
  * @brief  在指定行、列显示无符号十进制数
  * @param  Line   起始行：1 ~ 4
  * @param  Column 起始列：1 ~ 16
  * @param  Number 要显示的数值
  * @param  Length 显示位数（不足补 0，例如 Length=4、Number=300 显示 "0300"）
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#endif /* __OLED_H */
