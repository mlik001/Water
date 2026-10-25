/**
  ******************************************************************************
  * @file    bsp_delay.h
  * @brief   基于 DWT 周期计数器的微秒/毫秒延时（共享基础层）
  * @note    1-Wire(DS18B20) 和软件 I2C 都需要微秒级延时，统一放这里
  ******************************************************************************
  */
#ifndef __BSP_DELAY_H
#define __BSP_DELAY_H

#include "stm32f4xx_hal.h"

void BSP_Delay_Init(void);      /* 初始化 DWT 周期计数器（main 里调用一次） */
void BSP_DelayUs(uint32_t us);  /* 微秒级阻塞延时 */
void BSP_DelayMs(uint32_t ms);  /* 毫秒级阻塞延时（内部走 HAL_Delay） */

#endif /* __BSP_DELAY_H */
