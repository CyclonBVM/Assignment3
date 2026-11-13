/**
  ******************************************************************************
  * @file    app_packet.c
  * @brief   串口数据包校验实现
  ******************************************************************************
  */

#include "app_packet.h"

#include <string.h>

/**
  * @brief  向组包状态机喂入一个字节
  * @param  rx   组包状态
  * @param  byte 刚收到的一个字节
  * @param  out  凑满一整包时的输出缓冲
  * @retval 1 表示凑满一包；0 表示还需要继续接收
  *
  * 组包规则：
  *   1) len == 0 时只接受帧头 0xAA，其它字节一律丢弃（帧同步）；
  *   2) 对齐帧头后按顺序收满 6 字节，中途不再寻找帧头
  *      （因为数据字段本身也可能等于 0xAA，中途重找会破坏正确的包）；
  *   3) 收满后整包拷贝给调用者，并把 len 清零，开始找下一包的帧头。
  */
uint8_t Packet_RxByte(PacketRx *rx, uint8_t byte, Packet *out)
{
  if ((rx == 0) || (out == 0))
  {
    return 0u;
  }

  /* ① 还没对齐帧头：丢弃非帧头字节 */
  if ((rx->len == 0u) && (byte != PACKET_HEADER))
  {
    return 0u;
  }

  /* ② 按顺序存入组包缓冲 */
  rx->buf[rx->len] = byte;
  rx->len++;

  /* ③ 收满一整包：拷贝输出并复位，等待下一包 */
  if (rx->len >= PACKET_SIZE)
  {
    (void)memcpy(out, rx->buf, PACKET_SIZE);
    rx->len = 0u;
    return 1u;
  }

  return 0u;
}

/**
  * @brief  校验一个数据包是否合法
  * @param  pkt 待校验的数据包指针
  * @retval 1 合法；0 非法
  *
  * 校验顺序：帧头 → 校验和 → 参数字段范围
  *   1) 帧头必须是 0xAA；
  *   2) CHECK = (0xAA + RUN + DIR + STEP_L + STEP_H) % 256，
  *      即前 5 个字节累加后只保留低 8 位（uint8_t 自然截断）；
  *   3) RUN、DIR 只能取 0 或 1；间隔必须在 100 ~ 2000 ms。
  * 任意一项不通过都返回 0，由调用者决定"不执行、保持原状态"。
  */
uint8_t Packet_IsValid(const Packet *pkt)
{
  const uint8_t *raw;
  uint8_t sum = 0u;
  uint8_t i;

  if (pkt == 0)
  {
    return 0u;
  }

  /* ① 帧头检查：不符合约定格式的包直接丢弃 */
  if (pkt->header != PACKET_HEADER)
  {
    return 0u;
  }

  /* ② 校验和：把 CHECK 之前的 5 个字节（含帧头）全部相加，保留低 8 位 */
  raw = (const uint8_t *)pkt;
  for (i = 0u; i < (PACKET_SIZE - 1u); i++)
  {
    sum = (uint8_t)(sum + raw[i]);
  }
  if (pkt->checksum != sum)
  {
    return 0u;
  }

  /* ③ 参数范围检查：标志只能是 0/1，间隔必须在协议允许范围内 */
  if ((pkt->run > 1u) || (pkt->dir > 1u))
  {
    return 0u;
  }
  if ((pkt->step_ms < PACKET_STEP_MIN_MS) || (pkt->step_ms > PACKET_STEP_MAX_MS))
  {
    return 0u;
  }

  return 1u;
}
