/**
  ******************************************************************************
  * @file    app_host.h
  * @brief   上位机串口模拟（临时联调用）
  * @note    用 UART4(PC10/PC11) 模拟上位机：
  *          - TIM6 定时器产生 1s 心跳，累计到 HOST_COLLECT_INTERVAL_S 采集一次传感器
  *          - 传感器数据通过 printf(UART4) 上报
  *          - 串口接收 ASCII 命令，控制 385 电机 / 蠕动泵 / 加热棒
  ******************************************************************************
  */
#ifndef __APP_HOST_H
#define __APP_HOST_H

#include "stm32f4xx_hal.h"

/* 定时采集周期（秒）。
 * DS18B20 已改为非阻塞状态机（阶段②完成），采集读缓存不再阻塞，可设更短。 */
#define HOST_COLLECT_INTERVAL_S   2

/* 串口接收行缓冲区大小 */
#define HOST_RX_LINE_MAX          64

void AppHost_Init(void);      /* 初始化 TIM6 定时器 + UART4 中断接收 */
void AppHost_Process(void);   /* 主循环调用：定时采集上报 + 串口命令处理 */

#endif /* __APP_HOST_H */
