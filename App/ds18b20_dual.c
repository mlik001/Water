/**
  ******************************************************************************
  * @file    ds18b20_dual.c
  * @brief   DS18B20 独立版驱动实现（两个传感器各占一根 GPIO）
  * @note    开漏输出 + 内部上拉（外部还需 4.7k 上拉）。
  *          1-Wire 时序对中断敏感，读写位的临界区临时关中断。
  *
  *          【非阻塞状态机】主循环调用 DS18B20_Dual_Process() 推进：
  *            IDLE ──发两路转换命令(约2ms,关中断)──▶ WAIT_CONVERT
  *            WAIT_CONVERT ──750ms 到点(非阻塞)──▶ 读两路暂存器(约2ms) ──▶ 回 IDLE
  *          750ms 转换期间中断开放、主循环照常跑 LWIP，彻底消除阻塞。
  ******************************************************************************
  */
#include "ds18b20_dual.h"
#include "bsp_delay.h"

#define DS18B20_CMD_SKIP_ROM         0xCC
#define DS18B20_CMD_CONVERT          0x44
#define DS18B20_CMD_READ_SCRATCHPAD  0xBE

/* ---- 非阻塞状态机 ---- */
typedef enum
{
    DS18B20_SM_IDLE = 0,        /* 空闲：下一圈启动一轮转换 */
    DS18B20_SM_WAIT_CONVERT,    /* 已发转换命令，等待 750ms */
} DS18B20_SMState;

static DS18B20_SMState g_sm_state       = DS18B20_SM_IDLE;
static uint32_t        g_convert_start  = 0;          /* 发出转换命令的时刻（HAL_GetTick） */
static uint8_t         g_has_data       = 0;          /* 是否已至少完成过一次测量 */

/* 最近一次读到的结果（读暂存器阶段写入，转换等待期间保持不变） */
static uint8_t g_temp_ok[2] = {1, 1};               /* 0=成功, 1=无器件/读取失败 */
static float   g_temp[2]    = {0.0f, 0.0f};

static uint16_t DS18B20_GetPin(DS18B20_DualChannel ch)
{
    return (ch == DS18B20_DUAL_UPPER) ? DS18B20_DUAL_UPPER_PIN : DS18B20_DUAL_LOWER_PIN;
}

/* ---- 1-Wire 底层时序（微秒级，调用前需保证中断不打断） ---- */

static uint8_t DS18B20_Reset(uint16_t pin)
{
    uint8_t presence;

    HAL_GPIO_WritePin(DS18B20_DUAL_GPIO_PORT, pin, GPIO_PIN_RESET);  /* 拉低 480us */
    BSP_DelayUs(480);
    HAL_GPIO_WritePin(DS18B20_DUAL_GPIO_PORT, pin, GPIO_PIN_SET);    /* 释放 */
    BSP_DelayUs(60);
    presence = HAL_GPIO_ReadPin(DS18B20_DUAL_GPIO_PORT, pin);        /* 0=存在 */
    BSP_DelayUs(420);                                                /* 补足时隙 */
    return (presence == GPIO_PIN_RESET) ? 0 : 1;
}

static void DS18B20_WriteBit(uint16_t pin, uint8_t bit)
{
    HAL_GPIO_WritePin(DS18B20_DUAL_GPIO_PORT, pin, GPIO_PIN_RESET);
    BSP_DelayUs(2);
    if (bit)
    {
        HAL_GPIO_WritePin(DS18B20_DUAL_GPIO_PORT, pin, GPIO_PIN_SET);
    }
    BSP_DelayUs(60);
    HAL_GPIO_WritePin(DS18B20_DUAL_GPIO_PORT, pin, GPIO_PIN_SET);
    BSP_DelayUs(2);
}

static uint8_t DS18B20_ReadBit(uint16_t pin)
{
    uint8_t bit;

    HAL_GPIO_WritePin(DS18B20_DUAL_GPIO_PORT, pin, GPIO_PIN_RESET);
    BSP_DelayUs(2);
    HAL_GPIO_WritePin(DS18B20_DUAL_GPIO_PORT, pin, GPIO_PIN_SET);
    BSP_DelayUs(6);
    bit = (HAL_GPIO_ReadPin(DS18B20_DUAL_GPIO_PORT, pin) == GPIO_PIN_SET) ? 1 : 0;
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

/* ---- 高层命令（每个函数内部含 reset，调用方需关中断保护时序） ---- */

static uint8_t DS18B20_StartConvert(uint16_t pin)
{
    if (DS18B20_Reset(pin) != 0)
    {
        return 1;   /* 总线无器件响应 */
    }
    DS18B20_WriteByte(pin, DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(pin, DS18B20_CMD_CONVERT);
    return 0;
}

static uint8_t DS18B20_ReadScratchpad(uint16_t pin, float *temp)
{
    uint8_t lsb, msb;
    int16_t raw;

    if (DS18B20_Reset(pin) != 0)
    {
        return 1;
    }
    DS18B20_WriteByte(pin, DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(pin, DS18B20_CMD_READ_SCRATCHPAD);
    lsb = DS18B20_ReadByte(pin);
    msb = DS18B20_ReadByte(pin);

    raw = (int16_t)(((uint16_t)msb << 8) | lsb);
    *temp = raw / 16.0f;   /* 12 位分辨率 0.0625℃ */
    return 0;
}

/* ---- 初始化 ---- */

void DS18B20_Dual_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOG_CLK_ENABLE();

    gpio.Pin   = DS18B20_DUAL_UPPER_PIN | DS18B20_DUAL_LOWER_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏（1-Wire 总线） */
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DS18B20_DUAL_GPIO_PORT, &gpio);

    HAL_GPIO_WritePin(DS18B20_DUAL_GPIO_PORT, DS18B20_DUAL_UPPER_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DS18B20_DUAL_GPIO_PORT, DS18B20_DUAL_LOWER_PIN, GPIO_PIN_SET);
}

/* ---- 非阻塞状态机 ---- */

/**
  * @brief  推进温度采集状态机（主循环每圈调用一次）
  * @note   每圈只做一步、立即返回，绝不阻塞：
  *          - IDLE：同时给两路发 Convert 命令（临界区约 2ms）
  *          - WAIT_CONVERT：到 750ms 才读两路暂存器（临界区约 2ms）
  *          转换等待期间主循环照常跑 LWIP / 命令处理。
  */
void DS18B20_Dual_Process(void)
{
    switch (g_sm_state)
    {
    case DS18B20_SM_WAIT_CONVERT:
        /* 2) 750ms 到点 → 读两路暂存器（临界区，临时关中断） */
        if ((uint32_t)(HAL_GetTick() - g_convert_start) >= DS18B20_CONVERT_TIME_MS)
        {
            __disable_irq();
            g_temp_ok[DS18B20_DUAL_UPPER] = DS18B20_ReadScratchpad(DS18B20_DUAL_UPPER_PIN, &g_temp[DS18B20_DUAL_UPPER]);
            g_temp_ok[DS18B20_DUAL_LOWER] = DS18B20_ReadScratchpad(DS18B20_DUAL_LOWER_PIN, &g_temp[DS18B20_DUAL_LOWER]);
            __enable_irq();

            g_has_data = 1;
            g_sm_state = DS18B20_SM_IDLE;   /* 读完回到空闲，下一圈启动新转换 */
        }
        break;

    case DS18B20_SM_IDLE:
    default:
        /* 1) 同时给两路发转换命令（临界区约 2ms，临时关中断保护时序） */
        __disable_irq();
        DS18B20_StartConvert(DS18B20_DUAL_UPPER_PIN);
        DS18B20_StartConvert(DS18B20_DUAL_LOWER_PIN);
        __enable_irq();

        g_convert_start = HAL_GetTick();
        g_sm_state = DS18B20_SM_WAIT_CONVERT;
        break;
    }
}

uint8_t DS18B20_Dual_IsReady(void)
{
    return g_has_data;
}

uint8_t DS18B20_Dual_GetStatus(DS18B20_DualChannel ch)
{
    return g_temp_ok[ch];
}

float DS18B20_Dual_GetTemp(DS18B20_DualChannel ch)
{
    return g_temp[ch];
}

/* ---- 阻塞接口（仅调试/单次用） ---- */

/**
  * @brief  读一路温度（阻塞，约 750ms，仅调试/单次用）
  * @param  ch:   通道（上水槽/下水槽）
  * @param  temp: 输出温度（℃，12 位分辨率 0.0625℃）
  * @retval 0=成功，1=总线无器件响应
  * @note   ⚠️ 会停主循环约 750ms。正常采集请用非阻塞状态机（Process + GetTemp），
  *          不要在本函数运行期间同时用状态机（会抢同一根 1-Wire 总线）。
  */
uint8_t DS18B20_Dual_ReadTemp(DS18B20_DualChannel ch, float *temp)
{
    uint16_t pin = DS18B20_GetPin(ch);

    /* 1) 启动转换（临界区约 1ms，临时关中断保护时序） */
    __disable_irq();
    if (DS18B20_Reset(pin) != 0)
    {
        __enable_irq();
        return 1;
    }
    DS18B20_WriteByte(pin, DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(pin, DS18B20_CMD_CONVERT);
    __enable_irq();

    /* 2) 等待转换（阻塞，仅调试用） */
    BSP_DelayMs(DS18B20_CONVERT_TIME_MS);

    /* 3) 读暂存器（临界区，临时关中断） */
    __disable_irq();
    DS18B20_Reset(pin);
    DS18B20_WriteByte(pin, DS18B20_CMD_SKIP_ROM);
    DS18B20_WriteByte(pin, DS18B20_CMD_READ_SCRATCHPAD);
    {
        uint8_t lsb = DS18B20_ReadByte(pin);
        uint8_t msb = DS18B20_ReadByte(pin);
        int16_t raw = (int16_t)(((uint16_t)msb << 8) | lsb);
        *temp = raw / 16.0f;   /* 12 位分辨率 0.0625℃ */
    }
    __enable_irq();

    return 0;
}
