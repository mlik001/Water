/**
  ******************************************************************************
  * @file    ds18b20_shared.c
  * @brief   DS18B20 共享版驱动实现（两个传感器共用一个 GPIO）
  * @note    1-Wire 底层时序与独立版一致（开漏 + 上拉，临界区关中断）。
  *          多设备靠 Match ROM 精确寻址；ROM 搜索用标准二叉树搜索算法。
  ******************************************************************************
  */
#include "ds18b20_shared.h"
#include "bsp_delay.h"

#define DS18B20_CMD_MATCH_ROM        0x55
#define DS18B20_CMD_SKIP_ROM         0xCC
#define DS18B20_CMD_SEARCH_ROM       0xF0
#define DS18B20_CMD_CONVERT          0x44
#define DS18B20_CMD_READ_SCRATCHPAD  0xBE

static const uint8_t upper_rom[8] = DS18B20_UPPER_ROM;
static const uint8_t lower_rom[8] = DS18B20_LOWER_ROM;

/* ---- 1-Wire 底层时序（与独立版相同，微秒级，需关中断保护） ---- */

static uint8_t DS18B20_Reset(uint16_t pin)
{
    uint8_t presence;

    HAL_GPIO_WritePin(DS18B20_BUS_PORT, pin, GPIO_PIN_RESET);
    BSP_DelayUs(480);
    HAL_GPIO_WritePin(DS18B20_BUS_PORT, pin, GPIO_PIN_SET);
    BSP_DelayUs(60);
    presence = HAL_GPIO_ReadPin(DS18B20_BUS_PORT, pin);
    BSP_DelayUs(420);
    return (presence == GPIO_PIN_RESET) ? 0 : 1;
}

static void DS18B20_WriteBit(uint16_t pin, uint8_t bit)
{
    HAL_GPIO_WritePin(DS18B20_BUS_PORT, pin, GPIO_PIN_RESET);
    BSP_DelayUs(2);
    if (bit)
    {
        HAL_GPIO_WritePin(DS18B20_BUS_PORT, pin, GPIO_PIN_SET);
    }
    BSP_DelayUs(60);
    HAL_GPIO_WritePin(DS18B20_BUS_PORT, pin, GPIO_PIN_SET);
    BSP_DelayUs(2);
}

static uint8_t DS18B20_ReadBit(uint16_t pin)
{
    uint8_t bit;

    HAL_GPIO_WritePin(DS18B20_BUS_PORT, pin, GPIO_PIN_RESET);
    BSP_DelayUs(2);
    HAL_GPIO_WritePin(DS18B20_BUS_PORT, pin, GPIO_PIN_SET);
    BSP_DelayUs(6);
    bit = (HAL_GPIO_ReadPin(DS18B20_BUS_PORT, pin) == GPIO_PIN_SET) ? 1 : 0;
    BSP_DelayUs(60);
    return bit;
}

static void DS18B20_WriteByte(uint16_t pin, uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        DS18B20_WriteBit(pin, byte & 0x01);
        byte >>= 1;
    }
}

static uint8_t DS18B20_ReadByte(uint16_t pin)
{
    uint8_t i, byte = 0;
    for (i = 0; i < 8; i++)
    {
        byte >>= 1;
        if (DS18B20_ReadBit(pin))
        {
            byte |= 0x80;
        }
    }
    return byte;
}

/* ---- 初始化 ---- */

void DS18B20_Shared_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOG_CLK_ENABLE();

    gpio.Pin   = DS18B20_BUS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_BUS_PORT, &gpio);

    HAL_GPIO_WritePin(DS18B20_BUS_PORT, DS18B20_BUS_PIN, GPIO_PIN_SET);
}

/* ---- ROM 搜索（发现总线上所有 DS18B20 的 64 位 ROM 码） ---- */

/**
  * @brief  搜索 1-Wire 总线上的所有设备 ROM 码（标准二叉树搜索算法）
  * @param  roms: 输出数组，每个设备 8 字节
  * @param  max:  最多搜索多少个（数组容量）
  * @retval 找到的设备数量
  * @note   串口打印示例：
  *         uint8_t roms[2][8]; uint8_t n = DS18B20_SearchRom(roms, 2);
  *         for (i=0;i<n;i++) printf("ROM%d: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
  *              i, roms[i][0],roms[i][1],roms[i][2],roms[i][3],
  *              roms[i][4],roms[i][5],roms[i][6],roms[i][7]);
  */
uint8_t DS18B20_SearchRom(uint8_t roms[][8], uint8_t max)
{
    uint8_t count = 0;
    uint8_t rom[8] = {0};
    uint8_t lastDiscrepancy = 0;
    uint8_t lastDeviceFlag = 0;
    uint8_t bitNumber;
    uint8_t lastZero;
    uint8_t i, bit, comp, searchDir, ok;

    while (!lastDeviceFlag && count < max)
    {
        if (DS18B20_Reset(DS18B20_BUS_PIN) != 0)
        {
            break;   /* 总线无设备响应 */
        }
        DS18B20_WriteByte(DS18B20_BUS_PIN, DS18B20_CMD_SEARCH_ROM);

        bitNumber = 1;
        lastZero  = 0;
        ok        = 1;

        for (i = 0; i < 64; i++)
        {
            bit  = DS18B20_ReadBit(DS18B20_BUS_PIN);
            comp = DS18B20_ReadBit(DS18B20_BUS_PIN);

            if (bit && comp)
            {
                ok = 0;   /* 两个位都是 1：无设备/总线错误 */
                break;
            }
            else if (bit != comp)
            {
                searchDir = bit;   /* 剩余设备该位都相同 */
            }
            else
            {
                /* 分叉：bit == comp == 0，总线上既有 0 也有 1 */
                if (bitNumber < lastDiscrepancy)
                {
                    searchDir = (rom[i >> 3] >> (i & 7)) & 0x01;
                }
                else
                {
                    searchDir = (bitNumber == lastDiscrepancy) ? 1 : 0;
                }
                if (searchDir == 0)
                {
                    lastZero = bitNumber;
                }
            }

            if (searchDir)
            {
                rom[i >> 3] |= (uint8_t)(1u << (i & 7));
            }
            else
            {
                rom[i >> 3] &= (uint8_t)~(1u << (i & 7));
            }
            DS18B20_WriteBit(DS18B20_BUS_PIN, searchDir);

            bitNumber++;
        }

        if (ok)
        {
            for (i = 0; i < 8; i++)
            {
                roms[count][i] = rom[i];
            }
            count++;
        }

        lastDiscrepancy = lastZero;
        if (lastZero == 0)
        {
            lastDeviceFlag = 1;
        }
    }

    return count;
}

/* ---- 按 ROM 匹配读温度 ---- */

/**
  * @brief  用 Match ROM 精确读某个传感器的温度
  * @param  rom:  目标传感器 64 位 ROM 码
  * @param  temp: 输出温度（℃）
  * @retval 0=成功，1=总线无器件响应
  */
static uint8_t DS18B20_MatchRead(const uint8_t rom[8], float *temp)
{
    uint8_t i, lsb, msb;
    int16_t raw;

    /* 1) 启动转换：Match ROM + Convert */
    __disable_irq();
    if (DS18B20_Reset(DS18B20_BUS_PIN) != 0)
    {
        __enable_irq();
        return 1;
    }
    DS18B20_WriteByte(DS18B20_BUS_PIN, DS18B20_CMD_MATCH_ROM);
    for (i = 0; i < 8; i++)
    {
        DS18B20_WriteByte(DS18B20_BUS_PIN, rom[i]);
    }
    DS18B20_WriteByte(DS18B20_BUS_PIN, DS18B20_CMD_CONVERT);
    __enable_irq();

    /* 2) 等待转换（12 位最长 750ms，中断开放不阻塞 LWIP） */
    BSP_DelayMs(750);

    /* 3) 读暂存器：Match ROM + Read Scratchpad */
    __disable_irq();
    DS18B20_Reset(DS18B20_BUS_PIN);
    DS18B20_WriteByte(DS18B20_BUS_PIN, DS18B20_CMD_MATCH_ROM);
    for (i = 0; i < 8; i++)
    {
        DS18B20_WriteByte(DS18B20_BUS_PIN, rom[i]);
    }
    DS18B20_WriteByte(DS18B20_BUS_PIN, DS18B20_CMD_READ_SCRATCHPAD);
    lsb = DS18B20_ReadByte(DS18B20_BUS_PIN);
    msb = DS18B20_ReadByte(DS18B20_BUS_PIN);
    __enable_irq();

    raw = (int16_t)(((uint16_t)msb << 8) | lsb);
    *temp = raw / 16.0f;
    return 0;
}

/**
  * @brief  按通道读温度（上水槽/下水槽，用各自 ROM 匹配）
  */
uint8_t DS18B20_Shared_ReadTemp(DS18B20_SharedChannel ch, float *temp)
{
    const uint8_t *rom = (ch == DS18B20_SHARED_UPPER) ? upper_rom : lower_rom;
    return DS18B20_MatchRead(rom, temp);
}
