/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "app_packet.h"   /* 6 字节数据包定义与校验 */
#include "oled.h"         /* 软件 I²C OLED 驱动 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/**
  * @brief 系统运行状态（唯一状态源）
  * @note  按键和串口只负责修改它；LED、OLED、串口反馈都从它读取，
  *        因此"实际灯效 / 屏幕显示 / 串口反馈"三者永远一致。
  *        该结构只在主循环上下文中访问，不需要 volatile。
  */
typedef struct
{
  uint8_t  run;        /* 运行标志：0 暂停，1 运行 */
  uint8_t  dir;        /* 方向：0 正向，1 反向 */
  uint16_t step_ms;    /* 相邻 LED 的切换间隔（ms） */
  uint8_t  led;        /* 当前点亮的 LED 编号：0 ~ 2 */
  uint32_t last_tick;  /* 上一次切换的时刻（HAL_GetTick 时间戳） */
} SysState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LED_NUM              3u   /* 流水灯数量 */
#define LED_ACTIVE_LEVEL     GPIO_PIN_RESET  /* 共阳接法：低电平点亮 */
#define LED_INACTIVE_LEVEL   GPIO_PIN_SET    /* 高电平熄灭 */
#define KEY_DEBOUNCE_MS      20u  /* 按键消抖时间（ms） */
#define TX_BUF_SIZE          64u  /* 串口反馈缓冲区大小 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* —— 唯一状态源 —— */
static SysState g_state =
{
  .run       = 1u,     /* 上电默认：运行 */
  .dir       = 0u,     /* 正向 */
  .step_ms   = 300u,   /* 300ms 切换一次 */
  .led       = 0u,     /* 点亮第 1 颗 */
  .last_tick = 0u
};

/* —— 按键事件：中断里只记录"事件标志 + 时间戳"，消抖放在主循环 —— */
static volatile uint8_t  s_key1_flag = 0u;
static volatile uint8_t  s_key2_flag = 0u;
static volatile uint32_t s_key1_tick = 0u;
static volatile uint32_t s_key2_tick = 0u;

/* —— 串口接收：逐字节接收，收到帧头 0xAA 才开始组包（可自动重新同步） —— */
static uint8_t          s_rx_byte;                 /* 单字节接收目标 */
static PacketRx         s_rx;                      /* 组包状态机（帧头同步） */
static Packet           s_pending_packet;          /* 组包完成后的副本，供主循环取用 */
static volatile uint8_t s_packet_ready = 0u;

/* —— 串口反馈：中断发送 + 忙/待发标志，避免发送过程中改写缓冲区 —— */
static char             s_tx_buf[TX_BUF_SIZE];
static volatile uint8_t s_tx_busy = 0u;    /* 1 = 上一次发送还没完成 */
static uint8_t          s_tx_pending = 0u; /* 1 = 有最新状态待发送 */

/* —— OLED 刷新标记：只有状态变化才置 1，避免每轮循环都刷屏 —— */
static uint8_t s_oled_dirty = 1u;
/* —— OLED 每行"上次显示的内容"缓存：只重画内容真的变了的行 —— */
static char    s_oled_cache[4][17];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Led_Apply(uint8_t idx);
static void Key_Process(void);
static void Packet_Process(void);
static void Led_Tick(void);
static void Oled_Render(void);
static void Oled_UpdateLine(uint8_t line_no, const char *text);
static void Status_Format(char *buf, uint32_t size);
static void Status_RequestFeedback(void);
static void   Tx_Service(void);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief 只点亮第 idx 颗 LED，其余两颗熄灭
  * @param idx 0 ~ 2
  */
static void Led_Apply(uint8_t idx)
{
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin,
                    (idx == 0u) ? LED_ACTIVE_LEVEL : LED_INACTIVE_LEVEL);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                    (idx == 1u) ? LED_ACTIVE_LEVEL : LED_INACTIVE_LEVEL);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
                    (idx == 2u) ? LED_ACTIVE_LEVEL : LED_INACTIVE_LEVEL);
}

/**
  * @brief 处理按键事件（在主循环中调用）
  * @note  中断回调只登记了"按下时刻"，这里等 20ms 后重新读引脚电平：
  *        - 抖动产生的多个下降沿：标志已被清掉，只会算一次；
  *        - 按住不放：下降沿只在按下瞬间产生一次，也不会连续触发；
  *        - 若 20ms 内已经松开（读回高电平），说明是干扰，直接丢弃。
  */
static void Key_Process(void)
{
  uint32_t now = HAL_GetTick();

  /* 按键 1（PA11）：切换正向 / 反向 */
  if ((s_key1_flag != 0u) &&
      ((uint32_t)(now - s_key1_tick) >= KEY_DEBOUNCE_MS))
  {
    s_key1_flag = 0u;
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
      g_state.dir ^= 1u;          /* 方向翻转 */
      s_oled_dirty = 1u;          /* 屏幕需要刷新 */
      Status_RequestFeedback();   /* 方向变化：回传状态 */
    }
  }

  /* 按键 2（PA12）：切换暂停 / 继续 */
  if ((s_key2_flag != 0u) &&
      ((uint32_t)(now - s_key2_tick) >= KEY_DEBOUNCE_MS))
  {
    s_key2_flag = 0u;
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
      g_state.run ^= 1u;
      if (g_state.run != 0u)
      {
        /* 从暂停恢复：重新计时，让新一次切换从完整间隔开始 */
        g_state.last_tick = now;
      }
      s_oled_dirty = 1u;
      Status_RequestFeedback();   /* 运行状态变化：回传状态 */
    }
  }
}

/**
  * @brief 校验并应用收到的数据包
  * @note  只有帧头、校验和、参数范围全部通过才执行；
  *        同一包中的三个设置（运行、方向、间隔）一次性生效，
  *        保证不会出现"只改了一半"的中间状态。
  */
static void Packet_Process(void)
{
  Packet   pkt;
  uint32_t primask;

  if (s_packet_ready == 0u)
  {
    return;
  }

  /* 临界区：取走副本并清标志，避免与接收中断竞争（副本只有 6 字节，很快） */
  primask = __get_PRIMASK();
  __disable_irq();
  pkt = s_pending_packet;
  s_packet_ready = 0u;
  __set_PRIMASK(primask);

  /* 非法包（帧头错 / 校验错 / 参数越界）：不执行任何动作，保持原状态 */
  if (Packet_IsValid(&pkt) == 0u)
  {
    return;
  }

  g_state.run     = pkt.run;
  g_state.dir     = pkt.dir;
  g_state.step_ms = pkt.step_ms;
  g_state.last_tick = HAL_GetTick();   /* 应用新设置后重新计时 */

  s_oled_dirty = 1u;
  Status_RequestFeedback();            /* 应用成功后回传状态 */
}

/**
  * @brief 流水灯调度：比较时间戳，到点才切换（非阻塞，不用 HAL_Delay）
  */
static void Led_Tick(void)
{
  uint32_t now;

  if (g_state.run == 0u)
  {
    return;   /* 暂停：保持当前 LED 点亮不动 */
  }

  now = HAL_GetTick();
  if ((uint32_t)(now - g_state.last_tick) >= g_state.step_ms)
  {
    g_state.last_tick = now;
    /* 正向 +1、反向 +2（对 3 取模等价于 -1），再对灯数取模 */
    g_state.led = (uint8_t)((g_state.led + ((g_state.dir != 0u) ? 2u : 1u)) % LED_NUM);
    Led_Apply(g_state.led);
    s_oled_dirty = 1u;   /* LED 编号变了，屏幕需要刷新 */
  }
}

/**
  * @brief 刷新 OLED 某一行（只有内容变化时才真正写屏）
  * @param line_no 行号 1 ~ 4
  * @param text    要显示的字符串（定宽，16 字符以内）
  * @note  软件 I²C 写一屏需要几毫秒，缓存上次显示内容后，
  *        切灯时只重画 "LED: n/3" 这一行，主循环响应更快。
  */
static void Oled_UpdateLine(uint8_t line_no, const char *text)
{
  size_t len;
  char  *cache = s_oled_cache[line_no - 1u];

  if (strcmp(text, cache) == 0)
  {
    return;
  }

  /* 注意：本函数依赖"每行内容定宽"的前提（见 Oled_Render 的说明），
     只有内容变化才重画；若以后把某行改短，旧像素会残留、缓存也会阻止重画。 */
  OLED_ShowString(line_no, 1u, text);

  len = strlen(text);
  if (len > 16u)
  {
    len = 16u;
  }
  (void)memcpy(cache, text, len);
  cache[len] = '\0';
}

/**
  * @brief 刷新 OLED 四行状态（只在 s_oled_dirty 置位时执行）
  * @note  每行都用"定宽"字符串，保证字段变长/变短时不会残留旧字符：
  *          STATE: RUN   / STATE: PAUSE   （固定 12 字符宽）
  *          DIR: FWD     / DIR: BWD
  *          STEP: 300ms  （%4u 保证 3~4 位对齐）
  *          LED: 1/3
  */
static void Oled_Render(void)
{
  char line[17];   /* 一行 16 个字符 + 结束符 */

  if (s_oled_dirty == 0u)
  {
    return;
  }
  s_oled_dirty = 0u;

  (void)snprintf(line, sizeof(line), "STATE: %-5s",
                 (g_state.run != 0u) ? "RUN" : "PAUSE");
  Oled_UpdateLine(1u, line);

  (void)snprintf(line, sizeof(line), "DIR: %s",
                 (g_state.dir != 0u) ? "BWD" : "FWD");
  Oled_UpdateLine(2u, line);

  (void)snprintf(line, sizeof(line), "STEP:%4ums",
                 (unsigned int)g_state.step_ms);
  Oled_UpdateLine(3u, line);

  (void)snprintf(line, sizeof(line), "LED: %u/%u",
                 (unsigned int)(g_state.led + 1u), (unsigned int)LED_NUM);
  Oled_UpdateLine(4u, line);
}

/**
  * @brief 把当前状态格式化成一行英文反馈
  * @note  格式：STATE=RUN,DIR=FWD,STEP=300,LED=1\r\n
  */
static void Status_Format(char *buf, uint32_t size)
{
  (void)snprintf(buf, (size_t)size, "STATE=%s,DIR=%s,STEP=%u,LED=%u\r\n",
                 (g_state.run != 0u) ? "RUN" : "PAUSE",
                 (g_state.dir != 0u) ? "BWD" : "FWD",
                 (unsigned int)g_state.step_ms,
                 (unsigned int)(g_state.led + 1u));
}

/**
  * @brief 请求发送一帧状态反馈
  * @note  只置"待发"标志，由 Tx_Service 在主循环里真正启动发送；
  *        连续多次变化时只会发送最后一次的状态。
  */
static void Status_RequestFeedback(void)
{
  s_tx_pending = 1u;
}

/**
  * @brief 串口反馈发送服务（在主循环中调用）
  * @note  用中断发送：启动后立即返回，发送完成由 HAL_UART_TxCpltCallback 清标志。
  *        上一次没发完（s_tx_busy）时先不启动，等回调置 0 再发最新状态；
  *        启动失败（返回非 HAL_OK）则保留待发标志，下一轮重试。
  */
static void Tx_Service(void)
{
  uint16_t len;

  if ((s_tx_pending == 0u) || (s_tx_busy != 0u))
  {
    return;
  }

  Status_Format(s_tx_buf, TX_BUF_SIZE);
  s_tx_pending = 0u;
  len = (uint16_t)strlen(s_tx_buf);

  /* 先置忙再启动：避免"发送完成回调抢在置忙之前执行"的极端情况，
     那样会出现"回调清 0 → 这里又置 1"的永久忙死。 */
  s_tx_busy = 1u;
  if (HAL_UART_Transmit_IT(&huart1, (uint8_t *)s_tx_buf, len) != HAL_OK)
  {
    s_tx_busy    = 0u;   /* 没启动成功：放开忙标志 */
    s_tx_pending = 1u;   /* 保留待发，下一轮再试 */
  }
}

/* ============================================================================
 *                       调试探针 DEBUG_PROBE（临时诊断用）
 * ----------------------------------------------------------------------------
 * 排查完把 DEBUG_PROBE 改成 0 即可完全关闭（关闭后不生成任何代码、不占空间）。
 *
 * 诊断输出都走串口 TX（你说"能输出不能输入"，正好用这个通道看结果）：
 *   1) 上电横幅：PA10(RX) 空闲电平 / PB1(SCL)、PB0(SDA) 空闲电平；
 *      —— 正常 UART 空闲为 1；I2C 空闲(pull-up)也为 1；读到 0 说明该线被拉死/短路。
 *   2) 全总线 I2C 扫描（0x08~0x78 偶数地址）：有 ACK 的地址会打印出来；
 *      —— 打印 0x78 说明屏幕在总线上、地址对、通信通；
 *      —— 打印 none 说明 I2C 物理层没通（SCL/SDA 接反、线断、屏坏）。
 *   3) 运行中每 500ms（或收到新字节时）打印：收到多少字节 + 最后 8 个字节的十六进制；
 *      —— rx 一直为 0        → "模块TXD → PA10" 这根线/引脚的问题；
 *      —— rx 在涨但内容不对  → 输入通路正常，问题在"助手发送格式（HEX/文本）"或包内容。
 *
 * 另外打开探针时会上电做一次"全屏点亮 1.5 秒"自检，肉眼即可判断屏活不活。
 * ==========================================================================*/
#define DEBUG_PROBE         0   /* 1 = 打开诊断；诊断完务必改回 0 */
#define DEBUG_PROBE_PERIOD  500u/* 运行中诊断行的最小输出间隔(ms) */

#if DEBUG_PROBE
static volatile uint32_t s_dbg_rx_bytes = 0u;      /* 收到的字节总数（不看包结构） */
static volatile uint8_t  s_dbg_hist[8]  = {0u};    /* 最后 8 个收到的字节 */
static volatile uint32_t s_dbg_poll_bytes = 0u;    /* 轮询 RXNE 数到的字节（中断没生效时才会计数） */
static volatile uint32_t s_dbg_err_bytes   = 0u;   /* 串口错误中断次数（ORE/FE/NE，能抓到波特率不对/噪声） */

/** @brief 阻塞方式输出一串字符（诊断用，频率很低，不影响正常功能） */
static void Dbg_Uart_Raw(const char *s)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 200u);
}

/** @brief 诊断用的粗略延时 */
static void Dbg_Delay(void)
{
  volatile uint32_t n = 60u;
  while (n-- > 0u)
  {
    __NOP();
  }
}

/**
  * @brief 对一条信号线做 4 种电平测试，返回结果位掩码
  * @param port 端口，pin 引脚
  * @retval bit0=内部上拉时读到高  bit1=内部下拉时读到高
  *         bit2=推挽输出高时读到高  bit3=开漏释放后读到高
  * @note   判读要点：
  *          - bit2 = 0 ：线被"硬短路到地"（面包板孔短路、线接错到 GND）
  *          - bit2 = 1 而 bit0 = 0 ：外部有更强的下拉，典型是"屏的 VCC 没供电，
  *            屏板载上拉挂在 0V 上"（此时整条总线恒为低）
  *          - 正常空闲线（有上拉的 I2C / 空闲 UART）= PU1 PD0 HI1 OD1
  */
static uint8_t Dbg_Line_Test(GPIO_TypeDef *port, uint16_t pin)
{
  GPIO_InitTypeDef g = {0};
  uint8_t r = 0u;

  g.Pin   = pin;
  g.Speed = GPIO_SPEED_FREQ_HIGH;

  /* 1) 输入 + 内部上拉 */
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(port, &g);
  Dbg_Delay();
  if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
  {
    r |= 0x01u;
  }

  /* 2) 输入 + 内部下拉：外部有强上拉才会读到高 */
  g.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(port, &g);
  Dbg_Delay();
  if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
  {
    r |= 0x02u;
  }

  /* 3) 推挽输出高：还能读到低，就是硬短路到地 */
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &g);
  Dbg_Delay();
  if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
  {
    r |= 0x04u;
  }

  /* 4) 开漏释放（ODR=1），靠外部上拉拉高 */
  HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
  g.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(port, &g);
  Dbg_Delay();
  if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
  {
    r |= 0x08u;
  }

  return r;
}

/** @brief 打印一条线的 4 项测试结果，并直接给出结论 */
static void Dbg_Line_Report(const char *name, uint8_t m)
{
  char line[96];
  int  n;

  n = snprintf(line, sizeof(line), "[PROBE] %-8s PU=%u PD=%u HI=%u OD=%u",
               name,
               (unsigned)(m & 0x01u),
               (unsigned)((m >> 1) & 0x01u),
               (unsigned)((m >> 2) & 0x01u),
               (unsigned)((m >> 3) & 0x01u));
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)n, 200u);

  if ((m & 0x04u) == 0u)
  {
    Dbg_Uart_Raw("  -> HARD SHORT to GND (线/孔短路或接错)\r\n");
  }
  else if (((m & 0x01u) == 0u) && ((m & 0x08u) == 0u))
  {
    Dbg_Uart_Raw("  -> EXTERNAL PULL-DOWN (屏VCC没供电/电源脚接反?)\r\n");
  }
  else if ((m & 0x01u) == 0u)
  {
    Dbg_Uart_Raw("  -> weak pull-down\r\n");
  }
  else
  {
    Dbg_Uart_Raw("  -> OK idle-high\r\n");
  }
}

/** @brief 总线恢复：SCL 打 9 个时钟 + STOP，把可能"卡住"的从机放回空闲 */
static void Dbg_Bus_Recover(void)
{
  uint8_t i;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  for (i = 0u; i < 9u; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    Dbg_Delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    Dbg_Delay();
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  Dbg_Delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  Dbg_Delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  Dbg_Delay();
}

/**
  * @brief 对一个 8 位写地址（7 位地址左移 1 位）发 START+地址，读应答
  * @retval 1 = 收到 ACK（从机存在且通信正常）；0 = NACK
  */
static uint8_t Dbg_I2C_AckProbe(uint8_t addr8)
{
  uint8_t i;
  uint8_t ack;

  /* START：SCL 高时 SDA 由高变低 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  Dbg_Delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  Dbg_Delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  Dbg_Delay();

  /* 8 位地址，高位先发 */
  for (i = 0u; i < 8u; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,
                      ((addr8 & 0x80u) != 0u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    addr8 = (uint8_t)(addr8 << 1);
    Dbg_Delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    Dbg_Delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    Dbg_Delay();
  }

  /* 第 9 个时钟：释放 SDA，从机拉低即 ACK */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  Dbg_Delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  Dbg_Delay();
  ack = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET) ? 1u : 0u;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  Dbg_Delay();

  /* STOP：SCL 高时 SDA 由低变高 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  Dbg_Delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  Dbg_Delay();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  Dbg_Delay();

  return ack;
}

/**
  * @brief 上电诊断：逐线做 4 种电平测试 + 条件化 I2C 地址扫描
  * @note  必须在 OLED_Init() 之前调用（此时 PB0/PB1 还没有被驱动）
  */
static void Dbg_Boot_Report(void)
{
  char    line[96];
  uint8_t m_pa10;
  uint8_t m_scl;
  uint8_t m_sda;
  uint8_t addr;
  uint8_t found = 0u;
  int     n;

  Dbg_Uart_Raw("\r\n[PROBE] ==== boot diagnostic v2 ====\r\n");
  Dbg_Uart_Raw("[PROBE] PU=上拉读 PD=下拉读 HI=推挽高读 OD=开漏释放读\r\n");
  Dbg_Uart_Raw("[PROBE] 正常空闲线应为 PU=1 PD=0 HI=1 OD=1\r\n");

  m_pa10 = Dbg_Line_Test(GPIOA, GPIO_PIN_10);
  Dbg_Line_Report("PA10/RX", m_pa10);
  m_scl  = Dbg_Line_Test(GPIOB, GPIO_PIN_1);
  Dbg_Line_Report("PB1/SCL", m_scl);
  m_sda  = Dbg_Line_Test(GPIOB, GPIO_PIN_0);
  Dbg_Line_Report("PB0/SDA", m_sda);

  Dbg_Uart_Raw("[PROBE] 自环测试: 拔掉模块TXD线, 短接 PA9<->PA10, 复位后 rx 应 >0\r\n");

  /* 总线不在空闲高电平就不做地址扫描：
     此时 SDA 恒为 0，任何地址都会被误判成"有应答"（上一版就是被这个假象骗了） */
  if (((m_scl & 0x04u) == 0u) || ((m_sda & 0x04u) == 0u))
  {
    Dbg_Uart_Raw("[PROBE] I2C bus 不在空闲高电平 -> 跳过扫描(全ACK是假象)\r\n");
    Dbg_Uart_Raw("[PROBE] 重点：①屏 VCC 脚是否真有 3.3V ②屏丝印引脚顺序有没有接反\r\n");
    return;
  }

  /* 把两条线都释放成开漏（靠屏上拉），再扫描 */
  Dbg_Uart_Raw("[PROBE] i2c scan:");
  for (addr = 0x08u; addr <= 0x78u; addr = (uint8_t)(addr + 2u))
  {
    if (Dbg_I2C_AckProbe(addr) != 0u)
    {
      found = 1u;
      n = snprintf(line, sizeof(line), " 0x%02X", (unsigned)addr);
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)n, 200u);
    }
  }
  if (found == 0u)
  {
    Dbg_Uart_Raw(" none");
  }
  Dbg_Uart_Raw("\r\n");
  Dbg_Bus_Recover();
}

/**
  * @brief 运行中诊断：打印收到的字节总数与最后 8 个字节
  * @note  只在"收到新字节"或"每 5 秒心跳"时输出，避免刷屏；
  *        避开中断发送占用串口的时间窗，减少诊断行丢失。
  */
static void Dbg_Loop_Report(void)
{
  static uint32_t last_ms   = 0u;
  static uint32_t last_cnt  = 0u;
  static uint32_t last_hb   = 0u;

  char     line[96];
  uint32_t now = HAL_GetTick();
  uint32_t cnt;
  uint32_t polln;
  uint32_t errn;
  uint8_t  ie;
  uint8_t  hist[8];
  uint8_t  n;
  uint8_t  k;
  int      pos;
  uint32_t primask;

  /* 接收中断没使能时用轮询兜底：证明"线路上到底有没有字节进来"。
     中断正常时（RXNEIE=1）这里不轮询，不会抢走中断的字节。 */
  if ((huart1.Instance->CR1 & USART_CR1_RXNEIE) == 0u)
  {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET)
    {
      (void)(huart1.Instance->DR);   /* 读 DR 清 RXNE 标志 */
      s_dbg_poll_bytes++;
    }
  }

  if ((uint32_t)(now - last_ms) < DEBUG_PROBE_PERIOD)
  {
    return;
  }
  if ((s_tx_busy != 0u) || (s_tx_pending != 0u))
  {
    return;   /* 正在发状态反馈，下一轮再打 */
  }
  last_ms = now;

  /* 取一份快照（很短，必要时关中断） */
  primask = __get_PRIMASK();
  __disable_irq();
  cnt = s_dbg_rx_bytes;
  polln = s_dbg_poll_bytes;
  errn = s_dbg_err_bytes;
  for (k = 0u; k < 8u; k++)
  {
    hist[k] = s_dbg_hist[k];
  }
  __set_PRIMASK(primask);
  ie = ((huart1.Instance->CR1 & USART_CR1_RXNEIE) != 0u) ? 1u : 0u;

  n = (cnt < 8u) ? (uint8_t)cnt : 8u;

  if ((cnt == last_cnt) && ((uint32_t)(now - last_hb) < 1000u))
  {
    return;   /* 没有新字节，且离上次心跳不到 5s：不输出 */
  }
  last_cnt = cnt;
  last_hb  = now;

  pos = snprintf(line, sizeof(line), "[PROBE] rx=%lu rxp=%lu err=%lu ie=%u pa10=%u last=",
                 (unsigned long)cnt,
                 (unsigned long)polln,
                 (unsigned long)errn,
                 (unsigned)ie,
                 (unsigned)((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET) ? 1u : 0u));
  for (k = 0u; k < n; k++)
  {
    if (pos >= ((int)sizeof(line) - 5))
    {
      break;
    }
    pos += snprintf(&line[pos], sizeof(line) - (size_t)pos,
                    "%02X ", (unsigned)hist[8u - n + k]);
  }
  if (pos < ((int)sizeof(line) - 3))
  {
    line[pos++] = '\r';
    line[pos++] = '\n';
  }
  line[pos] = '\0';
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)pos, 200u);
}
#endif /* DEBUG_PROBE */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 上电初始化：先让流水灯进入确定状态 */
  g_state.last_tick = HAL_GetTick();
  Led_Apply(g_state.led);

#if DEBUG_PROBE
  /* 上电诊断：先看三条线的空闲电平和 I2C 总线扫描结果（必须在 OLED_Init 之前） */
  Dbg_Boot_Report();
#endif

  /* 等 OLED 上电稳定后初始化（软件 I²C + SSD1315/SSD1306 兼容驱动，内部会清屏） */
  HAL_Delay(20);
  OLED_Init();
  s_oled_dirty = 1u;

#if DEBUG_PROBE
  /* 上电自检：屏幕应全白 1.5 秒，然后清屏进入正常显示。
     全白 = 屏和 I2C 链路都正常；仍全黑 = I2C 物理层/地址问题（看上面的扫描结果）。 */
  Dbg_Uart_Raw("[PROBE] oled init done, test pattern -> screen should be WHITE\r\n");
  OLED_TestPattern(1u);
  HAL_Delay(1500u);
  OLED_Clear();
  Dbg_Uart_Raw("[PROBE] test pattern cleared\r\n");
#endif

  /* —— 首次点亮自检（临时取消注释，确认屏和 I²C 链路）：屏幕应全白 ——
  OLED_TestPattern(1u);
  HAL_Delay(500u);
  OLED_Clear();
  —— 自检结束，恢复注释 —— */

  /* 启动串口中断接收：每次收 1 字节，由回调按帧头 0xAA 组包 */
  (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);

  /* 上电先回传一帧状态，便于确认串口链路正常 */
  Status_RequestFeedback();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 主循环五件事，全部非阻塞，任何一步都不会长时间占用 CPU */
    Key_Process();      /* 1. 按键消抖并处理事件（切换方向 / 暂停继续） */
    Packet_Process();   /* 2. 校验串口数据包，合法则一次性更新状态 */
    Led_Tick();         /* 3. 比较时间戳，到点推进流水灯 */
    Oled_Render();      /* 4. 状态有变化才刷新 OLED */
    Tx_Service();       /* 5. 串口空闲且有待发状态时，启动中断发送反馈 */
#if DEBUG_PROBE
    Dbg_Loop_Report();  /* 6. 诊断：打印收到多少字节（诊断完把 DEBUG_PROBE 改 0） */
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief 外部中断回调（KEY1 / KEY2 共用）
  * @param GPIO_Pin 触发中断的引脚
  * @note  只有记录，不做消抖、不刷屏、不延时——这是作业"中断回调只完成
  *        必要的事件记录"的要求。真正的处理在 Key_Process() 中完成。
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now = HAL_GetTick();

  if (GPIO_Pin == KEY1_Pin)
  {
    s_key1_tick = now;
    s_key1_flag = 1u;
  }
  else if (GPIO_Pin == KEY2_Pin)
  {
    s_key2_tick = now;
    s_key2_flag = 1u;
  }
}

/**
  * @brief 串口接收完成回调（每次 1 字节）
  * @param huart 触发回调的串口句柄
  * @note  组包逻辑放在 app_packet.c 的 Packet_RxByte() 里（纯函数、可在主机端测试）：
  *        未对齐帧头时丢弃非 0xAA 字节；对齐后按顺序收满 6 字节，
  *        收满即把整包拷成副本、置标志；每次回调都重新启动 1 字节接收。
  *        这样多发的换行、上电半包、中途丢字节都能在下一个 0xAA 自动重新同步。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
#if DEBUG_PROBE
    /* 诊断：不关心包结构，先统计"到底有没有字节进来" */
    {
      uint8_t k;
      s_dbg_rx_bytes++;
      for (k = 0u; k < 7u; k++)
      {
        s_dbg_hist[k] = s_dbg_hist[k + 1u];
      }
      s_dbg_hist[7] = s_rx_byte;
    }
#endif

    /* 喂入一个字节；凑满一整包时返回 1 */
    if (Packet_RxByte(&s_rx, s_rx_byte, &s_pending_packet) != 0u)
    {
      s_packet_ready = 1u;   /* 通知主循环：有一整包待校验应用 */
    }

    /* 重新启动接收，等待下一个字节 */
    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
  }
}

/**
  * @brief 串口发送完成回调
  * @param huart 触发回调的串口句柄
  * @note  只清"忙"标志，允许主循环启动下一次发送。
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    s_tx_busy = 0u;
  }
}

/**
  * @brief 串口错误回调（如接收溢出 ORE）
  * @param huart 触发回调的串口句柄
  * @note  串口错误回调来自接收侧（ORE/FE/NE），这里丢掉半包、重新从帧头开始同步，
  *        必要时重启接收。**不能在这里清 s_tx_busy**：发送可能仍在进行，
  *        清忙标志会让主循环提前改写正在发送的缓冲区。
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
#if DEBUG_PROBE
    s_dbg_err_bytes++;   /* 诊断：记录一次串口错误（噪声 / 波特率不对 / 断帧） */
#endif
    s_rx.len = 0u;   /* 丢弃半包，等下一个 0xAA 重新组包 */
    if (huart->RxState == HAL_UART_STATE_READY)
    {
      (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1u);
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
