/**
  ******************************************************************************
  * @file    relay.c
  * @brief   继电器组实现
  * @note    上电默认全部断开（释放继电器），避免误动作
  ******************************************************************************
  */
#include "relay.h"

static void Relay_Set(GPIO_TypeDef *port, uint16_t pin, RelayState st)
{
    GPIO_PinState level = (st == RELAY_ON) ? RELAY_ACTIVE_LEVEL : RELAY_INACTIVE_LEVEL;
    HAL_GPIO_WritePin(port, pin, level);
}

void Relay_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    gpio.Pin = RELAY_MOTOR_PIN;
    HAL_GPIO_Init(RELAY_MOTOR_PORT, &gpio);

    gpio.Pin = RELAY_PUMP_PIN;
    HAL_GPIO_Init(RELAY_PUMP_PORT, &gpio);

    gpio.Pin = RELAY_HEATER_PIN;
    HAL_GPIO_Init(RELAY_HEATER_PORT, &gpio);

    /* 上电默认全关 */
    Relay_Motor(RELAY_OFF);
    Relay_Pump(RELAY_OFF);
    Relay_Heater(RELAY_OFF);
}

void Relay_Motor(RelayState st)
{
    Relay_Set(RELAY_MOTOR_PORT, RELAY_MOTOR_PIN, st);
}

void Relay_Pump(RelayState st)
{
    Relay_Set(RELAY_PUMP_PORT, RELAY_PUMP_PIN, st);
}

void Relay_Heater(RelayState st)
{
    Relay_Set(RELAY_HEATER_PORT, RELAY_HEATER_PIN, st);
}
