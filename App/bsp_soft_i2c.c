/**
  ******************************************************************************
  * @file    bsp_soft_i2c.c
  * @brief   软件模拟 I2C 实现
  * @note    SCL/SDA 均为开漏输出 + 内部上拉。
  *          - 写 1 = 释放总线（靠上拉变高，从机也能拉低）
  *          - 写 0 = 拉低总线
  *          - 读 = 直接读 IDR
  *          约 100kHz（半周期延时 5us）
  ******************************************************************************
  */
#include "bsp_soft_i2c.h"
#include "bsp_delay.h"

#define I2C_DELAY_US   5   /* 半周期延时，约 100kHz 时钟 */

#define SCL_H()   HAL_GPIO_WritePin(SOFT_I2C_GPIO_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET)
#define SCL_L()   HAL_GPIO_WritePin(SOFT_I2C_GPIO_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET)
#define SDA_H()   HAL_GPIO_WritePin(SOFT_I2C_GPIO_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET)
#define SDA_L()   HAL_GPIO_WritePin(SOFT_I2C_GPIO_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ()  HAL_GPIO_ReadPin(SOFT_I2C_GPIO_PORT, SOFT_I2C_SDA_PIN)

static void SoftI2C_Delay(void) { BSP_DelayUs(I2C_DELAY_US); }

/**
  * @brief  初始化软件 I2C 引脚
  */
void SoftI2C_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin   = SOFT_I2C_SCL_PIN | SOFT_I2C_SDA_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏（I2C 标准，可安全接 5V 上拉） */
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SOFT_I2C_GPIO_PORT, &gpio);

    SCL_H();
    SDA_H();
}

/* ---- 底层时序 ---- */

static void SoftI2C_Start(void)
{
    SDA_H(); SoftI2C_Delay();
    SCL_H(); SoftI2C_Delay();
    SDA_L(); SoftI2C_Delay();   /* SCL 高时 SDA 下降沿 = 起始 */
    SCL_L(); SoftI2C_Delay();
}

static void SoftI2C_Stop(void)
{
    SDA_L(); SoftI2C_Delay();
    SCL_H(); SoftI2C_Delay();
    SDA_H(); SoftI2C_Delay();   /* SCL 高时 SDA 上升沿 = 停止 */
}

/**
  * @brief  等待从机应答
  * @retval 0=ACK，1=NACK
  */
static uint8_t SoftI2C_WaitAck(void)
{
    uint8_t ack;

    SDA_H(); SoftI2C_Delay();   /* 释放 SDA */
    SCL_H(); SoftI2C_Delay();   /* 第 9 个时钟 */
    ack = SDA_READ();           /* 从机拉低 = ACK(0) */
    SCL_L(); SoftI2C_Delay();

    return (ack == GPIO_PIN_RESET) ? 0 : 1;
}

/**
  * @brief  发送一个字节
  * @retval 0=收到 ACK，1=NACK
  */
static uint8_t SoftI2C_SendByte(uint8_t byte)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (byte & 0x80) SDA_H(); else SDA_L();
        byte <<= 1;
        SoftI2C_Delay();
        SCL_H(); SoftI2C_Delay();
        SCL_L(); SoftI2C_Delay();
    }
    return SoftI2C_WaitAck();
}

/**
  * @brief  读取一个字节
  * @param  ack: 1=主机回 NACK（最后一字节），0=主机回 ACK
  */
static uint8_t SoftI2C_ReadByte(uint8_t ack)
{
    uint8_t i, byte = 0;

    SDA_H();   /* 释放 SDA，由从机驱动数据线 */
    for (i = 0; i < 8; i++)
    {
        SoftI2C_Delay();
        SCL_H(); SoftI2C_Delay();
        byte <<= 1;
        if (SDA_READ() == GPIO_PIN_SET) byte |= 0x01;
        SCL_L();
    }

    /* 主机发送应答位 */
    if (ack) SDA_H(); else SDA_L();
    SoftI2C_Delay();
    SCL_H(); SoftI2C_Delay();
    SCL_L();
    SDA_H();
    return byte;
}

/* ---- 寄存器读写接口 ---- */

/**
  * @brief  写一个寄存器
  * @param  devAddr: 器件地址（7 位左移形式，如 0xB0）
  * @retval 0=成功，非 0=某字节无应答
  */
uint8_t SoftI2C_WriteReg(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    uint8_t err = 0;

    SoftI2C_Start();
    err  = SoftI2C_SendByte(devAddr & 0xFE);  /* 写地址 */
    err |= SoftI2C_SendByte(regAddr);
    err |= SoftI2C_SendByte(data);
    SoftI2C_Stop();

    return err;
}

/**
  * @brief  连续读多个寄存器（先写地址，再 Stop-Start 读，和 P31/P32/P33 参考代码一致）
  * @param  devAddr: 器件地址（如 0xB0）
  * @param  regAddr: 起始寄存器地址
  * @param  buf:     读回数据缓冲
  * @param  len:     读取字节数
  * @retval 0=成功，非 0=无应答
  */
uint8_t SoftI2C_ReadRegs(uint8_t devAddr, uint8_t regAddr, uint8_t *buf, uint8_t len)
{
    uint8_t i, err = 0;

    SoftI2C_Start();
    err = SoftI2C_SendByte(devAddr & 0xFE);  /* 写地址 */
    err |= SoftI2C_SendByte(regAddr);
    SoftI2C_Stop();

    SoftI2C_Start();
    err |= SoftI2C_SendByte(devAddr | 0x01); /* 读地址 */
    for (i = 0; i < len; i++)
    {
        buf[i] = SoftI2C_ReadByte((i == (len - 1)) ? 1 : 0);  /* 最后一字节 NACK */
    }
    SoftI2C_Stop();

    return err;
}

/**
  * @brief  探测 7 位地址是否有器件应答（用于 I2C 总线扫描）
  * @param  devAddr7: 7 位器件地址（如 0x58）
  * @retval 1=有应答，0=无应答
  */
uint8_t SoftI2C_Probe(uint8_t devAddr7)
{
    uint8_t err;

    SoftI2C_Start();
    err = SoftI2C_SendByte((devAddr7 << 1) & 0xFE);  /* 写地址 */
    SoftI2C_Stop();

    return (err == 0) ? 1 : 0;
}
