/**
  ******************************************************************************
  * @file    app_tcp.h
  * @brief   饮水机系统 TCP 后端通讯（最终阶段）—— JSON 协议
  * @note    STM32 作为 TCP 客户端，主动连接后端服务器 APP_TCP_SERVER_IP:APP_TCP_PORT。
  *          后端（PC/服务器）作为服务端监听。断线/连接失败自动重连。
  *          使用 LwIP 裸机 raw API（NO_SYS=1，无 RTOS）。
  *          收发均为 JSON 格式，详细协议见 app_tcp.c 文件头。
  ******************************************************************************
  */
#ifndef __APP_TCP_H
#define __APP_TCP_H

#include "stm32f4xx_hal.h"

/* 后端服务器 IP（STM32 作为客户端主动连接它，四个字节分别填） */
#define APP_TCP_SERVER_IP_A        192
#define APP_TCP_SERVER_IP_B        168
#define APP_TCP_SERVER_IP_C        1
#define APP_TCP_SERVER_IP_D        4

/* 后端服务器监听端口 */
#define APP_TCP_PORT               8088

/* 断线/连接失败后重连间隔（毫秒） */
#define APP_TCP_RECONNECT_INTERVAL_MS  3000

/* 握手超时：发出 hello 后超过该时间（毫秒）未收到 hello_ack 则重发 hello */
#define APP_TCP_HELLO_RETRY_MS         3000

/* 定时采集上报周期（秒）。由 LwIP 慢定时器(tcp_poll)驱动；DS18B20 已非阻塞，温度走缓存 */
#define APP_TCP_REPORT_INTERVAL_S  1

/* 命令行接收缓冲区大小（单条 JSON 命令足够） */
#define APP_TCP_RX_LINE_MAX        256

/* 设备唯一 ID（后端按此识别设备，所有上行报文都带 deviceId） */
#define APP_TCP_DEVICE_ID          "WC-001"

void AppTcp_Init(void);      /* 发起首次连接后端服务器 */
void AppTcp_Process(void);   /* 主循环调用：断线重连驱动 */

#endif /* __APP_TCP_H */
