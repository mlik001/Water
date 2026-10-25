/**
  ******************************************************************************
  * @file    bsp_delay.c
  * @brief   基于 DWT 周期计数器的微秒/毫秒延时
  * @note    主频 168MHz，DWT->CYCCNT 每微秒计 168 个数
  ******************************************************************************
  */
#include "bsp_delay.h"

static uint32_t dwt_us_factor = 0;  /* 每微秒对应的周期数 */

/**
  * @brief  初始化 DWT 周期计数器（main 中 HAL_Init 之后调用一次）
  */
void BSP_Delay_Init(void)
{
    /* 使能 DWT（调试跟踪单元）的周期计数器 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    dwt_us_factor = SystemCoreClock / 1000000UL;  /* 168000000 / 1e6 = 168 */
}

/**
  * @brief  微秒级阻塞延时
  * @param  us: 延时时间（微秒）
  * @note   32 位计数器约 25.5s 回绕一次；无符号相减自动处理回绕，
  *         只要 us 远小于回绕周期就安全（微秒级延时完全满足）
  */
void BSP_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * dwt_us_factor;

    while ((DWT->CYCCNT - start) < ticks)
    {
    }
}

/**
  * @brief  毫秒级阻塞延时（复用 HAL 的 SysTick）
  */
void BSP_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}
