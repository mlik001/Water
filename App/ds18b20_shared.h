/**
  ******************************************************************************
  * @file    ds18b20_shared.h
  * @brief   温度传感器 DS18B20 ×2（【共享版】两个传感器共用一个 GPIO）
  * @note    两个 DS18B20 挂在同一根 1-Wire 总线上（默认 PG10），
  *          通过 ROM 搜索 + ROM 匹配（Match ROM）区分两个传感器。
  *          与【独立版】（ds18b20_dual.h，两个独立 GPIO）二选一，不要同时加入 Keil。
  *
  *  使用步骤：
  *   1. 两个传感器并联接同一根线（PG10），每只都接 4.7k 上拉到 3.3V（或 5V）。
  *   2. 先调 DS18B20_SearchRom() 搜出两个 64 位 ROM 码，串口打印出来。
  *   3. 把两个码分别填进 DS18B20_UPPER_ROM / DS18B20_LOWER_ROM 宏。
  *      区分哪个是上/下水槽的办法：只给上水槽加热，看哪个码对应的温度先变。
  *   4. 之后 DS18B20_Shared_ReadTemp() 用 Match ROM 精确读对应传感器。
  ******************************************************************************
  */
#ifndef __DS18B20_SHARED_H
#define __DS18B20_SHARED_H

#include "stm32f4xx_hal.h"

#define DS18B20_BUS_PORT    GPIOG
#define DS18B20_BUS_PIN     GPIO_PIN_10   /* 共用引脚，默认 PG10，可改成 PG9 */

/* 两个传感器的 64 位 ROM 码（8 字节）。先用 DS18B20_SearchRom() 搜出后填这里。
 * 例：{0x28,0xAA,0x12,0x34,0x56,0x78,0x9A,0xBC}（0x28 是 DS18B20 家族码） */
#define DS18B20_UPPER_ROM   {0x28,0,0,0,0,0,0,0}   /* TODO: 填上水槽真实 ROM 码 */
#define DS18B20_LOWER_ROM   {0x28,0,0,0,0,0,0,0}   /* TODO: 填下水槽真实 ROM 码 */

typedef enum
{
    DS18B20_SHARED_UPPER = 0,   /* 上水槽 */
    DS18B20_SHARED_LOWER = 1,   /* 下水槽 */
} DS18B20_SharedChannel;

void     DS18B20_Shared_Init(void);
uint8_t  DS18B20_SearchRom(uint8_t roms[][8], uint8_t max);              /* 搜索总线设备，返回数量 */
uint8_t  DS18B20_Shared_ReadTemp(DS18B20_SharedChannel ch, float *temp); /* 按 ROM 匹配读温度 */

#endif /* __DS18B20_SHARED_H */
