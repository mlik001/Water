/**
  ******************************************************************************
  * @file    pressure_sensor.h
  * @brief   P31/P32/P33 I2C 数字压力传感器驱动
  * @note    器件地址 0xB0(写)/0xB1(读)，即 7 位 0x58
  *          24 位 ADC 输出，量程按型号调整（默认 1600 kPa = 1.6 MPa）
  ******************************************************************************
  */
#ifndef __PRESSURE_SENSOR_H
#define __PRESSURE_SENSOR_H

#include "stm32f4xx_hal.h"

#define PRESSURE_ADDR_W        0xB0      /* 器件写地址 */
#define PRESSURE_ADDR_R        0xB1      /* 器件读地址 */

/* 量程（kPa）。P31 = 0~10bar = 0~1000kPa。数据手册公式：P = Inter_P / 2^21 * 满量程 */
#define PRESSURE_FULL_KPA      1000.0f

/* 寄存器定义 */
#define PRESSURE_REG_CTRL      0x01      /* 控制/状态：写 0x01 启动转换，读 bit5=忙 */
#define PRESSURE_REG_DATA_H    0x04      /* 压力数据高字节 */
#define PRESSURE_REG_DATA_M    0x05      /* 压力数据中字节 */
#define PRESSURE_REG_DATA_L    0x06      /* 压力数据低字节 */

void     PressureSensor_Init(void);
uint8_t  PressureSensor_ReadRaw(int32_t *raw);   /* 读 24 位原始值（带符号） */
uint8_t  PressureSensor_ReadKpa(float *kpa);     /* 读压力值（kPa，已减零点偏移） */
int32_t  PressureSensor_CalibrateZero(void);     /* 零点校准：返回零点原始值 */

#endif /* __PRESSURE_SENSOR_H */
