/**
  ******************************************************************************
  * @file    pressure_sensor.c
  * @brief   P31/P32/P33 I2C 压力传感器实现
  * @note    流程：启动转换 -> 轮询忙标志 -> 读 3 字节 -> 24 位符号扩展 -> 换算 kPa
  ******************************************************************************
  */
#include "pressure_sensor.h"
#include "bsp_soft_i2c.h"
#include "bsp_delay.h"

/* 零点校准偏移（原始值，kPa 换算时减掉） */
static int32_t g_zero_raw = 0;

/**
  * @brief  初始化压力传感器（复用软件 I2C 引脚 PB8/PB9）
  */
void PressureSensor_Init(void)
{
    SoftI2C_Init();
}

/**
  * @brief  写控制寄存器启动一次转换
  */
static uint8_t PressureSensor_StartConv(void)
{
    return SoftI2C_WriteReg(PRESSURE_ADDR_W, PRESSURE_REG_CTRL, 0x01);
}

/**
  * @brief  读 24 位原始压力值（带符号）
  * @param  raw: 输出原始值
  * @retval 0=成功，1=启动失败，3=读数据失败
  */
uint8_t PressureSensor_ReadRaw(int32_t *raw)
{
    uint8_t  buf[3];
    uint32_t tmp;

    if (PressureSensor_StartConv() != 0)
    {
        return 1;
    }

    /* 数据手册：启动转换后固定延时 10ms 即可读取（本传感器没有 busy 标志位） */
    BSP_DelayMs(10);

    /* 读压力数据 0x04(高) 0x05(中) 0x06(低) */
    if (SoftI2C_ReadRegs(PRESSURE_ADDR_R, PRESSURE_REG_DATA_H, buf, 3) != 0)
    {
        return 3;
    }

    tmp = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2];

    /* 24 位有符号：数据手册 RAW_P > 8388608(0x800000) 时为负 */
    if (tmp > 0x800000UL)
    {
        tmp -= 0x1000000UL;
    }
    *raw = (int32_t)tmp;
    return 0;
}

/**
  * @brief  读压力值（kPa）
  * @param  kpa: 输出压力（kPa）
  * @retval 同 PressureSensor_ReadRaw
  */
uint8_t PressureSensor_ReadKpa(float *kpa)
{
    int32_t raw;
    uint8_t ret = PressureSensor_ReadRaw(&raw);
    if (ret != 0)
    {
        return ret;
    }
    *kpa = (float)(raw - g_zero_raw) / 2097152.0f * PRESSURE_FULL_KPA;
    return 0;
}

/**
  * @brief  零点校准：采集 8 次取平均，作为零点偏移保存
  * @note   校准前务必让传感器处于 0 压力（气嘴敞口对大气、管道无压力）
  * @retval 零点原始偏移值
  */
int32_t PressureSensor_CalibrateZero(void)
{
    int32_t sum = 0, raw, avg = 0;
    uint8_t i, ok = 0;

    for (i = 0; i < 8; i++)
    {
        if (PressureSensor_ReadRaw(&raw) == 0)
        {
            sum += raw;
            ok++;
        }
        BSP_DelayMs(20);
    }

    if (ok > 0)
    {
        avg = sum / ok;
        g_zero_raw = avg;
    }
    return avg;
}
