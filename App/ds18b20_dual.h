/**
  ******************************************************************************
  * @file    ds18b20_dual.h
  * @brief   温度传感器 DS18B20 ×2（【独立版】两个传感器各占一根 GPIO）
  * @note    上水槽 = PG15，下水槽 = PG9
  *          每根总线只挂一个 DS18B20，用 Skip ROM（无需搜索 ROM）。
  *          【共享版】（两个传感器共用一个 GPIO）见 ds18b20_shared.h/.c，二选一。
  *
  *  ✅ 阶段②已完成：温度采集改为【非阻塞状态机】，主循环调用
  *     DS18B20_Dual_Process() 推进，750ms 转换期间不阻塞 LWIP。
  *     上报/采集直接读缓存（GetTemp / GetStatus），不要再用阻塞的 ReadTemp。
  ******************************************************************************
  */
#ifndef __DS18B20_DUAL_H
#define __DS18B20_DUAL_H

#include "stm32f4xx_hal.h"

#define DS18B20_DUAL_GPIO_PORT    GPIOG
#define DS18B20_DUAL_UPPER_PIN    GPIO_PIN_15   /* 上水槽 = PG15 */
#define DS18B20_DUAL_LOWER_PIN    GPIO_PIN_9    /* 下水槽 = PG9  */

/* 12 位分辨率最长转换时间（ms）。状态机发完转换命令后等待这么久再读暂存器 */
#define DS18B20_CONVERT_TIME_MS   750

typedef enum
{
    DS18B20_DUAL_UPPER = 0,   /* 上水槽 */
    DS18B20_DUAL_LOWER = 1,   /* 下水槽 */
} DS18B20_DualChannel;

void     DS18B20_Dual_Init(void);

/* ---- 非阻塞状态机接口（推荐） ---- */
void     DS18B20_Dual_Process(void);                             /* 主循环每圈调用，推进状态机 */
uint8_t  DS18B20_Dual_IsReady(void);                             /* 是否已至少完成过一次测量（缓存有效） */
uint8_t  DS18B20_Dual_GetStatus(DS18B20_DualChannel ch);         /* 最近一次读取状态，0=成功 1=无器件/失败 */
float    DS18B20_Dual_GetTemp(DS18B20_DualChannel ch);           /* 最近一次读取温度（℃） */

/* ---- 阻塞接口（仅调试/单次用，会停主循环约 750ms，勿在采集主路径调用） ---- */
uint8_t  DS18B20_Dual_ReadTemp(DS18B20_DualChannel ch, float *temp);

#endif /* __DS18B20_DUAL_H */
