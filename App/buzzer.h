/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   蜂鸣器驱动（有源蜂鸣器，GPIO 高低电平开关）
  * @note    PF9 = 蜂鸣器控制脚；TIM7 中断按周期驱动变频蜂鸣（温度报警）
  ******************************************************************************
  */
#ifndef __BUZZER_H
#define __BUZZER_H

#include "stm32f4xx_hal.h"

/* 蜂鸣器引脚 */
#define BUZZER_PORT            GPIOF
#define BUZZER_PIN             GPIO_PIN_9

/* 有效电平：GPIO_PIN_SET=高电平响，GPIO_PIN_RESET=低电平响。
 * ⚠️ 按你的蜂鸣器驱动电路改这个宏即可（三极管驱动通常高电平响） */
#define BUZZER_ACTIVE_LEVEL    GPIO_PIN_SET
#define BUZZER_IDLE_LEVEL      GPIO_PIN_RESET

void Buzzer_Init(void);
void Buzzer_On(void);                               /* 常响 */
void Buzzer_Off(void);                              /* 常停 */
void Buzzer_SetBeepPeriod(uint32_t period_ms);      /* 设定蜂鸣周期(ms)，0=静音 */

#endif /* __BUZZER_H */
