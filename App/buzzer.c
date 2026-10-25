/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   蜂鸣器驱动实现（TIM7 中断变频蜂鸣）
  * @note    TIM7 基础定时器 10ms 中断；Buzzer_SetBeepPeriod 设定周期，
  *          周期内前 80ms 响、其余灭 → 周期越短响得越快；0=静音。
  ******************************************************************************
  */
#include "buzzer.h"

/* ---------------- TIM7 定时器 ---------------- */
static TIM_HandleTypeDef htim7;

/* ---------------- 蜂鸣状态 ---------------- */
static uint32_t g_beep_period_ms = 0;   /* 蜂鸣周期(ms)，0=静音 */
static uint32_t g_beep_tick_ms   = 0;   /* 累计毫秒（ISR 累加） */

#define BUZZER_TICK_MS     10            /* TIM7 中断周期 */
#define BUZZER_BEEP_ON_MS  80            /* 每声"滴"持续时长 */

void Buzzer_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 1) 蜂鸣器引脚 PF9 */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin   = BUZZER_PIN;
    HAL_GPIO_Init(BUZZER_PORT, &gpio);
    Buzzer_Off();   /* 上电默认不响 */

    /* 2) TIM7 基础定时器，10ms 更新中断（84MHz / 8400 / 100 = 10ms） */
    __HAL_RCC_TIM7_CLK_ENABLE();
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 8400 - 1;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 100 - 1;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
    {
        return;   /* 定时器失败：蜂鸣器仍可 On/Off，只是无周期滴答 */
    }

    HAL_NVIC_SetPriority(TIM7_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    HAL_TIM_Base_Start_IT(&htim7);
}

void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_ACTIVE_LEVEL);
}

void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, BUZZER_IDLE_LEVEL);
}

void Buzzer_SetBeepPeriod(uint32_t period_ms)
{
    g_beep_period_ms = period_ms;
    if (period_ms == 0)
    {
        g_beep_tick_ms = 0;
        Buzzer_Off();
    }
}

/**
  * @brief  TIM7 更新中断：按 g_beep_period_ms 驱动蜂鸣器滴答
  */
void TIM7_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);

        if (g_beep_period_ms == 0)
        {
            return;
        }

        g_beep_tick_ms += BUZZER_TICK_MS;

        /* 每个周期内：前 BUZZER_BEEP_ON_MS 响，其余灭 */
        if ((g_beep_tick_ms % g_beep_period_ms) < BUZZER_BEEP_ON_MS)
        {
            Buzzer_On();
        }
        else
        {
            Buzzer_Off();
        }
    }
}
