/**
  ******************************************************************************
  * @file    relay.h
  * @brief   继电器组驱动（3 路）
  * @note    RELAY1 = 电机  PE3
  *          RELAY2 = 蠕动泵 PE4
  *          RELAY3 = 加热棒 PG6
  *          高电平吸合（实测：ON=高电平=吸合，OFF=低电平=释放）
  ******************************************************************************
  */
#ifndef __RELAY_H
#define __RELAY_H

#include "stm32f4xx_hal.h"

/* 吸合电平：GPIO_PIN_RESET=低电平吸合，GPIO_PIN_SET=高电平吸合（本板实测为高电平吸合） */
#define RELAY_ACTIVE_LEVEL     GPIO_PIN_SET
#define RELAY_INACTIVE_LEVEL   GPIO_PIN_RESET

#define RELAY_MOTOR_PORT    GPIOE
#define RELAY_MOTOR_PIN     GPIO_PIN_3    /* RELAY1 电机 = PE3 */

#define RELAY_PUMP_PORT     GPIOE
#define RELAY_PUMP_PIN      GPIO_PIN_4    /* RELAY2 蠕动泵 = PE4 */

#define RELAY_HEATER_PORT   GPIOG
#define RELAY_HEATER_PIN    GPIO_PIN_7    /* RELAY3 加热棒 = PG7 */

typedef enum
{
    RELAY_OFF = 1,
    RELAY_ON  = 0,
} RelayState;

void Relay_Init(void);
void Relay_Motor(RelayState st);    /* 电机 */
void Relay_Pump(RelayState st);     /* 蠕动泵 */
void Relay_Heater(RelayState st);   /* 加热棒 */

#endif /* __RELAY_H */
