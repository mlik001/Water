/**
  ******************************************************************************
  * @file    flowmeter.c
  * @brief   霍尔式流量计实现
  * @note    中断里只做脉冲自增（极短），流率换算放在主循环里做。
  ******************************************************************************
  */
#include "flowmeter.h"

static volatile uint32_t flow_pulse_count = 0;

/**
  * @brief  初始化流量计：PG15 配置为下降沿 EXTI 输入并开启中断
  */
void FlowMeter_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    gpio.Pin   = FLOW_METER_PIN;
    gpio.Mode  = GPIO_MODE_IT_FALLING;   /* 霍尔脉冲下降沿触发 */
    gpio.Pull  = GPIO_PULLUP;            /* 内部上拉（霍尔输出多为开集，需上拉） */
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FLOW_METER_PORT, &gpio);

    HAL_NVIC_SetPriority(FLOW_METER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FLOW_METER_IRQn);
}

void FlowMeter_Reset(void)
{
    flow_pulse_count = 0;
}

uint32_t FlowMeter_GetPulseCount(void)
{
    return flow_pulse_count;
}

float FlowMeter_GetTotalVolume(void)
{
    return (float)flow_pulse_count / FLOW_PULSES_PER_LITER;
}

/**
  * @brief  计算瞬时流量（L/min）
  * @note   每调用一次，用本次与上次调用之间的脉冲差和时间差计算。
  *         建议主循环里每 1s 调用一次。
  */
float FlowMeter_GetFlowRate(void)
{
    static uint32_t last_pulses = 0;
    static uint32_t last_tick   = 0;
    uint32_t now    = HAL_GetTick();
    uint32_t dt     = now - last_tick;
    uint32_t pulses = flow_pulse_count;
    uint32_t dp     = pulses - last_pulses;

    last_tick   = now;
    last_pulses = pulses;

    if (dt == 0)
    {
        return 0.0f;
    }
    /* L/min = 脉冲数 / 时间ms * 1000ms/s * 60s/min / 每升脉冲数 */
    return (float)dp * 60000.0f / ((float)dt * FLOW_PULSES_PER_LITER);
}

/**
  * @brief  EXTI15_10 中断服务函数（放本模块，避免改动 stm32f4xx_it.c）
  */
void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(FLOW_METER_PIN) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(FLOW_METER_PIN);
        flow_pulse_count++;
    }
}
