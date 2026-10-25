/**
  ******************************************************************************
  * @file    flowmeter.h
  * @brief   霍尔式流量计驱动（脉冲计数）
  * @note    数据线 = PG15，下降沿触发 EXTI 计数。
  *          EXTI/NVIC 在 FlowMeter_Init() 里手动配置，
  *          不要在 CubeMX 里把 PG15 配成 EXTI（会重复定义中断函数）。
  ******************************************************************************
  */
#ifndef __FLOWMETER_H
#define __FLOWMETER_H

#include "stm32f4xx_hal.h"

#define FLOW_METER_PORT       GPIOF
#define FLOW_METER_PIN        GPIO_PIN_10   /* 数据线 = PF10 */
#define FLOW_METER_IRQn       EXTI15_10_IRQn

/* 流量计系数：每升对应脉冲数（据具体型号调整，ZJ-S401(3.5) 常见 2580 脉冲/升） */
#define FLOW_PULSES_PER_LITER 2580.0f

void      FlowMeter_Init(void);
uint32_t  FlowMeter_GetPulseCount(void);   /* 累计脉冲数 */
void      FlowMeter_Reset(void);           /* 清零累计脉冲 */
float     FlowMeter_GetTotalVolume(void);  /* 累计流量（L） */
float     FlowMeter_GetFlowRate(void);     /* 瞬时流量（L/min），按调用间隔计算 */

#endif /* __FLOWMETER_H */
