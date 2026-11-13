/**
  ******************************************************************************
  * @file    oled.c
  * @brief   软件 I²C 驱动 SSD1306 OLED（128x64）
  *
  * 实现要点：
  *   1) 软件 I²C：用 PB1(SCL)、PB0(SDA) 两个普通 GPIO 模拟起始/停止/字节/应答时序，
  *      不占用 STM32 的 I²C 外设（作业要求"软件 I²C"）；
  *   2) SCL 推挽输出、SDA 开漏输出（可回读应答），模块自带的上拉电阻保证空闲为高；
  *   3) SSD1306 采用"页寻址模式"：屏幕分 8 页（每页 8 行）× 128 列，
  *      每个字节对应一列 8 个像素，bit0 在最上面；
  *   4) 8x16 字模：每个字符占 2 页 × 8 列 = 16 字节，上半页 8 字节 + 下半页 8 字节，
  *      数据来自 OLED_Font.h（来源见该文件头部说明）。
  ******************************************************************************
  */

#include "oled.h"
#include "OLED_Font.h"

#include <stdio.h>

/* ============================== 内部宏定义 ============================== */

#define OLED_I2C_ADDR   0x78u   /* SSD1315/SSD1306 写地址：0x3C << 1
                                   若屏幕不亮，先确认模块 SA0 电平：
                                   常见模块地址为 0x3C（写地址 0x78）或 0x3D（写地址 0x7A） */
#define OLED_CTRL_CMD   0x00u   /* 控制字节：后续为命令 */
#define OLED_CTRL_DATA  0x40u   /* 控制字节：后续为显存数据 */
#define OLED_CHAR_W     8u      /* 8x16 字模宽度（列） */
#define OLED_CHAR_H     16u     /* 8x16 字模高度（行） */
#define OLED_MAX_COL    16u     /* 每行最多显示 16 个字符 */

/* ========================== 驱动芯片选择 ==========================
 * 1 = SSD1315（本工程实物的驱动芯片）
 *     SSD1315 命令与 SSD1306 基本兼容，但多一条 0xAD「内部 IREF 设置」；
 *     不加这条时部分 SSD1315 模块会出现"有电压、全黑"。
 * 0 = 纯 SSD1306 模块（去掉 SSD1315 专有命令）
 */
#define OLED_CONTROLLER_SSD1315  1

/* ============================== 底层时序 ============================== */

/**
  * @brief  软件 I²C 的位延时
  * @note   72MHz 下约 1.6µs，配合 GPIO 操作使 SCL 约 150~200kHz，
  *         明显低于 SSD1306 允许的 400kHz，杜邦线连接也能稳定工作。
  *         若实测屏幕偶发不亮/花屏，可再调大此值；如果嫌刷屏慢可适当调小。
  */
static void OLED_I2C_Delay(void)
{
  volatile uint32_t n = 24u;
  while (n-- > 0u)
  {
    __NOP();
  }
}

/** @brief 设置 SCL 电平 */
static void OLED_SCL_Write(uint8_t level)
{
  HAL_GPIO_WritePin(OLED_SCL_PORT, OLED_SCL_PIN,
                    (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @brief 设置 SDA 电平（开漏输出：写 1 表示释放总线，由上拉电阻拉高） */
static void OLED_SDA_Write(uint8_t level)
{
  HAL_GPIO_WritePin(OLED_SDA_PORT, OLED_SDA_PIN,
                    (level != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/** @brief 读 SDA 电平（用于读应答） */
static uint8_t OLED_SDA_Read(void)
{
  return (HAL_GPIO_ReadPin(OLED_SDA_PORT, OLED_SDA_PIN) == GPIO_PIN_SET) ? 1u : 0u;
}

/** @brief I²C 起始信号：SCL 为高时，SDA 由高变低 */
static void OLED_I2C_Start(void)
{
  OLED_SDA_Write(1u);
  OLED_SCL_Write(1u);
  OLED_I2C_Delay();
  OLED_SDA_Write(0u);
  OLED_I2C_Delay();
  OLED_SCL_Write(0u);
  OLED_I2C_Delay();
}

/** @brief I²C 停止信号：SCL 为高时，SDA 由低变高 */
static void OLED_I2C_Stop(void)
{
  OLED_SDA_Write(0u);
  OLED_I2C_Delay();
  OLED_SCL_Write(1u);
  OLED_I2C_Delay();
  OLED_SDA_Write(1u);
  OLED_I2C_Delay();
}

/**
  * @brief  发送一个字节，并读从机应答
  * @param  byte 待发送字节（高位先发）
  * @retval 0 收到应答(ACK)；1 未收到应答(NACK)
  */
static uint8_t OLED_I2C_SendByte(uint8_t byte)
{
  uint8_t i;
  uint8_t nack;

  for (i = 0u; i < 8u; i++)
  {
    OLED_SDA_Write((uint8_t)((byte & 0x80u) != 0u));
    byte = (uint8_t)(byte << 1);
    OLED_I2C_Delay();
    OLED_SCL_Write(1u);
    OLED_I2C_Delay();
    OLED_SCL_Write(0u);
    OLED_I2C_Delay();
  }

  /* 第 9 个时钟：主机释放 SDA，读从机拉低与否 */
  OLED_SDA_Write(1u);
  OLED_I2C_Delay();
  OLED_SCL_Write(1u);
  OLED_I2C_Delay();
  nack = OLED_SDA_Read();     /* 0 = ACK，1 = NACK */
  OLED_SCL_Write(0u);
  OLED_I2C_Delay();

  return nack;
}

/**
  * @brief  向 OLED 写一条命令
  */
static void OLED_WriteCmd(uint8_t cmd)
{
  (void)OLED_I2C_Start();
  (void)OLED_I2C_SendByte(OLED_I2C_ADDR);   /* 从机地址 + 写 */
  (void)OLED_I2C_SendByte(OLED_CTRL_CMD);   /* 控制字节：命令 */
  (void)OLED_I2C_SendByte(cmd);
  OLED_I2C_Stop();
}

/**
  * @brief  向 OLED 连续写显存数据（一次 I²C 事务写多个字节，提高刷新速度）
  * @param  buf 数据首地址
  * @param  len 字节数
  */
static void OLED_WriteData(const uint8_t *buf, uint16_t len)
{
  uint16_t i;

  (void)OLED_I2C_Start();
  (void)OLED_I2C_SendByte(OLED_I2C_ADDR);   /* 从机地址 + 写 */
  (void)OLED_I2C_SendByte(OLED_CTRL_DATA);  /* 控制字节：显存数据 */
  for (i = 0u; i < len; i++)
  {
    (void)OLED_I2C_SendByte(buf[i]);
  }
  OLED_I2C_Stop();
}

/**
  * @brief  设置显存写入位置（页寻址模式）
  * @param  page   页号 0~7
  * @param  column 列号 0~127
  */
static void OLED_SetCursor(uint8_t page, uint8_t column)
{
  OLED_WriteCmd((uint8_t)(0xB0u | (page & 0x07u)));               /* 设置页地址 */
  OLED_WriteCmd((uint8_t)(0x00u | (column & 0x0Fu)));             /* 列地址低 4 位 */
  OLED_WriteCmd((uint8_t)(0x10u | ((column >> 4) & 0x0Fu)));      /* 列地址高 4 位 */
}

/* ============================== 对外接口 ============================== */

/**
  * @brief  初始化软件 I²C 引脚与 OLED 驱动芯片（SSD1315 / 兼容 SSD1306 命令）
  * @note   上电后需等待 VCC 稳定再发命令（数据手册建议 ≥100ms）
  */
void OLED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 1) 使能 GPIOB 时钟并配置引脚（CubeMX 未配置 PB0/PB1，由驱动负责） */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin   = OLED_SCL_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;   /* SCL 推挽输出 */
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_SCL_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin   = OLED_SDA_PIN;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;   /* SDA 开漏输出，可回读应答 */
  GPIO_InitStruct.Pull  = GPIO_PULLUP;           /* 注意：STM32F1 的 Pull 只在输入模式生效，
                                                    开漏输出时高电平依赖模块自带的上拉电阻
                                                    （常见 4 针模块自带 4.7k~10k；若模块没有，
                                                     需外接 4.7k 上拉到 3.3V，否则屏幕不亮） */
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_SDA_PORT, &GPIO_InitStruct);

  OLED_SCL_Write(1u);
  OLED_SDA_Write(1u);
  HAL_Delay(100u);   /* 上电后等屏幕内部电源稳定：SSD1315 数据手册建议 ≥100ms */

  /* 2) 初始化命令序列（128x64、页寻址模式、内部电荷泵）
   *    兼容 SSD1306；SSD1315 额外需要 0xAD 设置内部 IREF，否则可能"有电全黑" */
  static const uint8_t init_cmds[] =
  {
    0xAEu,             /* 关闭显示 */
    0xD5u, 0x80u,      /* 显示时钟分频 / 振荡频率 */
    0xA8u, 0x3Fu,      /* 多路复用率 = 1/64 */
    0xD3u, 0x00u,      /* 显示偏移 = 0 */
    0x40u,             /* 显示起始行 = 0 */
    0x8Du, 0x14u,      /* 使能内部电荷泵（SSD1315 同样是 0x8D + 0x14） */
    0x20u, 0x02u,      /* 页寻址模式 */
    0xA1u,             /* 段重映射（列 127 -> SEG0），配合模块默认方向 */
    0xC8u,             /* COM 反向扫描，配合模块默认方向 */
    0xDAu, 0x12u,      /* COM 引脚硬件配置（128x64 用 0x12） */
#if OLED_CONTROLLER_SSD1315
    0xADu, 0x30u,      /* SSD1315：内部 IREF 设置（若偏暗可试 0x10/0x20，范围 0x00~0x3F） */
#endif
    0x81u, 0xCFu,      /* 对比度 */
    0xD9u, 0xF1u,      /* 预充电周期（若亮度异常可试 0x22） */
    0xDBu, 0x40u,      /* VCOMH 电压 */
    0x2Eu,             /* 关闭滚动（防止上电残留滚动状态） */
    0xA4u,             /* 输出跟随显存 */
    0xA6u              /* 正常显示（非反色） */
  };
  uint8_t i;

  for (i = 0u; i < (uint8_t)sizeof(init_cmds); i++)
  {
    OLED_WriteCmd(init_cmds[i]);
  }

  OLED_Clear();         /* 先清屏，避免上电显示随机雪花点 */
  OLED_WriteCmd(0xAFu); /* 清屏完成后再打开显示 */
}

/**
  * @brief  点亮自检：把整屏填成全亮或全灭
  * @param  on 1 = 全屏点亮（白）；0 = 全屏熄灭（黑）
  * @note   用于首次上板确认"屏 + I²C 链路"是否正常：
  *         调用 OLED_TestPattern(1) 后屏幕应全白；
  *         若仍是全黑，说明 I²C 没通（查 SCL/SDA、地址 0x78/0x7A、上拉）。
  */
void OLED_TestPattern(uint8_t on)
{
  uint8_t buf[OLED_WIDTH];
  uint8_t page;
  uint16_t i;

  for (i = 0u; i < OLED_WIDTH; i++)
  {
    buf[i] = (on != 0u) ? 0xFFu : 0x00u;
  }

  for (page = 0u; page < OLED_PAGES; page++)
  {
    OLED_SetCursor(page, 0u);
    OLED_WriteData(buf, OLED_WIDTH);
  }
}

/**
  * @brief  清屏：把 8 页 × 128 列全部写 0
  */
void OLED_Clear(void)
{
  uint8_t zero[OLED_WIDTH];
  uint8_t page;
  uint16_t i;

  for (i = 0u; i < OLED_WIDTH; i++)
  {
    zero[i] = 0x00u;
  }

  for (page = 0u; page < OLED_PAGES; page++)
  {
    OLED_SetCursor(page, 0u);
    OLED_WriteData(zero, OLED_WIDTH);
  }
}

/**
  * @brief  在指定行、列显示字符串（8x16 字模）
  * @note   行号 1~4，列号 1~16；每行上下两个"页"，先写上半页再写下半页，
  *         每页用一次 I²C 事务写完整行，速度比逐字符写快很多。
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, const char *String)
{
  uint8_t page;
  uint8_t column;
  uint8_t count = 0u;
  uint8_t i;
  uint8_t ch;
  const uint8_t *glyph;
  uint8_t buf[OLED_WIDTH];   /* 最多 16 字符 × 8 列 = 128 字节 */

  if ((Line < 1u) || (Line > 4u) || (Column < 1u) || (Column > OLED_MAX_COL) ||
      (String == 0))
  {
    return;
  }

  page   = (uint8_t)((Line - 1u) * 2u);
  column = (uint8_t)((Column - 1u) * OLED_CHAR_W);

  /* ① 上半页（字符的第 0~7 行）：把每个字符字模的前 8 字节依次排开 */
  while ((String[count] != '\0') && ((uint8_t)(Column + count) <= OLED_MAX_COL))
  {
    ch = (uint8_t)String[count];
    if ((ch < OLED_FONT_FIRST) || (ch > OLED_FONT_LAST))
    {
      ch = (uint8_t)' ';    /* 字模表外的字符按空格显示，避免越界 */
    }
    glyph = OLED_F8x16[ch - OLED_FONT_FIRST];
    for (i = 0u; i < 8u; i++)
    {
      buf[(count * 8u) + i] = glyph[i];
    }
    count++;
  }

  if (count == 0u)
  {
    return;
  }

  OLED_SetCursor(page, column);
  OLED_WriteData(buf, (uint16_t)(count * 8u));

  /* ② 下半页（字符的第 8~15 行）：同一批字符字模的后 8 字节 */
  for (i = 0u; i < count; i++)
  {
    ch = (uint8_t)String[i];
    if ((ch < OLED_FONT_FIRST) || (ch > OLED_FONT_LAST))
    {
      ch = (uint8_t)' ';
    }
    glyph = OLED_F8x16[ch - OLED_FONT_FIRST];
    for (uint8_t j = 0u; j < 8u; j++)
    {
      buf[(i * 8u) + j] = glyph[8u + j];
    }
  }

  OLED_SetCursor((uint8_t)(page + 1u), column);
  OLED_WriteData(buf, (uint16_t)(count * 8u));
}

/**
  * @brief  在指定行、列显示无符号十进制数（不足位数补 0）
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
  char str[12];

  if ((Length == 0u) || (Length > 10u))
  {
    return;
  }

  (void)snprintf(str, sizeof(str), "%0*u", (int)Length, (unsigned int)Number);
  OLED_ShowString(Line, Column, str);
}
