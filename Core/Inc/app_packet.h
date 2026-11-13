/**
  ******************************************************************************
  * @file    app_packet.h
  * @brief   串口数据包协议定义与校验（作业：6 字节简单数据包）
  *
  * 数据包格式（共 6 字节，十六进制）：
  *     AA  RUN  DIR  STEP_L  STEP_H  CHECK
  *     ─┬─  ─┬─  ─┬─  ──┬───  ──┬───  ──┬──
  *      │    │    │     │       │       └─ 校验和 = 前 5 字节之和 % 256
  *      │    │    │     │       └───────── 间隔高字节
  *      │    │    │     └───────────────── 间隔低字节
  *      │    │    └─────────────────────── 方向：00 正向 / 01 反向
  *      │    └──────────────────────────── 运行：00 暂停 / 01 运行
  *      └───────────────────────────────── 固定帧头
  *
  * 间隔单位 ms，范围 100 ~ 2000，发送时先低字节后高字节（STM32 小端存放）。
  ******************************************************************************
  */

#ifndef __APP_PACKET_H
#define __APP_PACKET_H

#include <stdint.h>

/* 协议常量 */
#define PACKET_HEADER        0xAAu   /* 固定帧头 */
#define PACKET_SIZE          6u      /* 一包总长度（字节） */
#define PACKET_STEP_MIN_MS   100u    /* 切换间隔下限 */
#define PACKET_STEP_MAX_MS   2000u   /* 切换间隔上限 */

/**
  * @brief 接收用的数据包结构体
  * @note  必须加 __attribute__((packed))，否则编译器会在 uint16_t 前补空洞，
  *        结构体变成 8 字节，第 6 个字节的 CHECK 就落不到结构体里，校验永远失败。
  *        加了 packed 后成员与字节流一一对应，uint16_t 成员按小端解释，
  *        正好还原 STEP_L / STEP_H。
  */
typedef struct __attribute__((packed))
{
  uint8_t  header;    /* 帧头，应为 PACKET_HEADER */
  uint8_t  run;       /* 运行标志：0 暂停，1 运行 */
  uint8_t  dir;       /* 方向标志：0 正向，1 反向 */
  uint16_t step_ms;   /* 切换间隔（低字节在前） */
  uint8_t  checksum;  /* 校验和 */
} Packet;

/* 编译期检查：结构体必须恰好 6 字节，防止有人去掉 packed 后静默出错 */
_Static_assert(sizeof(Packet) == PACKET_SIZE, "Packet 必须是 6 字节");

/**
  * @brief  校验一个数据包是否合法
  * @param  pkt 待校验的数据包指针
  * @retval 1 合法；0 非法（帧头错 / 校验和错 / 参数超出范围）
  * @note   本函数只做判断，不修改任何全局状态；
  *         调用者应在主循环中调用，不要在中断回调里解析数据包。
  */
uint8_t Packet_IsValid(const Packet *pkt);

/**
  * @brief  接收组包状态机
  * @note   只在"已经对齐帧头"之后才开始记账：
  *         len == 0 表示当前在等待帧头 0xAA，其余字节直接丢弃。
  *         这样即使发送端多发了换行、上电瞬间收到半包或中途丢字节，
  *         也能在下一个 0xAA 处自动重新同步，不会永久错位。
  */
typedef struct
{
  uint8_t buf[PACKET_SIZE];   /* 组包缓冲 */
  uint8_t len;                /* 已收字节数；0 表示等待帧头 */
} PacketRx;

/**
  * @brief  向组包状态机喂入一个字节
  * @param  rx   组包状态（由调用者持有，初值清零即可）
  * @param  byte 刚收到的一个字节
  * @param  out  凑满一整包时，把整包拷贝到这里
  * @retval 1 表示凑满一包（out 有效）；0 表示还需要继续接收
  * @note   本函数只做"拆包/组包"，不做合法性强校验；
  *         物理层校验仍由 Packet_IsValid() 完成，两者都在主循环/中断的
  *         固定位置调用，便于单元测试。
  */
uint8_t Packet_RxByte(PacketRx *rx, uint8_t byte, Packet *out);

#endif /* __APP_PACKET_H */
