/**
  ******************************************************************************
  * @file    bsp_soft_i2c.h
  * @brief   软件模拟 I2C（GPIO 位翻转）—— 压力传感器通信
  * @note    引脚：SCL = PB8，SDA = PB9
  *          开漏输出 + 上拉，SDA 读时靠开漏释放读 IDR，无需切换输入/输出模式
  ******************************************************************************
  */
#ifndef __BSP_SOFT_I2C_H
#define __BSP_SOFT_I2C_H

#include "stm32f4xx_hal.h"

#define SOFT_I2C_GPIO_PORT    GPIOB
#define SOFT_I2C_SCL_PIN      GPIO_PIN_8   /* SCL = PB8 */
#define SOFT_I2C_SDA_PIN      GPIO_PIN_9   /* SDA = PB9 */

void     SoftI2C_Init(void);
uint8_t  SoftI2C_WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t data);
uint8_t  SoftI2C_ReadRegs(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint8_t len);
uint8_t  SoftI2C_Probe(uint8_t devAddr7);   /* 探测 7 位地址是否有器件应答，1=有 */

#endif /* __BSP_SOFT_I2C_H */
