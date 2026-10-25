# 饮水机系统 —— 模块驱动（App 层）

> 本目录存放饮水机各模块的驱动代码，**自包含**：每个模块的 `Init()` 会自己初始化对应 GPIO，
> 不依赖 CubeMX 重新生成，也不会被 CubeMX 覆盖（位于 `App/`，不在 `Core/` 里）。

## 一、引脚分配总表

| 模块 | 引脚 | 外设 | 说明 |
|------|------|------|------|
| 压力传感器 | PB8 = SCL, PB9 = SDA | 软件 I2C | P31/P32/P33，地址 0xB0/0xB1，24 位 ADC |
| 流量计 | PG15 | EXTI 下降沿计数 | 霍尔脉冲输出 |
| 温度传感器（上水槽） | PG10 | 1-Wire | DS18B20 |
| 温度传感器（下水槽） | PG9 | 1-Wire | DS18B20 |
| 385抽水泵（电机） | PE3 | GPIO 输出 | RELAY1 |
| 蠕动泵 | PE4 | GPIO 输出 | RELAY2 |
| 加热棒 | PG6 | GPIO 输出 | RELAY3 |

## 二、文件清单

| 文件 | 模块 | 作用 |
|------|------|------|
| `bsp_delay.h/.c` | 基础层 | DWT 微秒延时（DS18B20 / 软 I2C 共用） |
| `bsp_soft_i2c.h/.c` | 基础层 | 软件模拟 I2C（PB8/PB9） |
| `pressure_sensor.h/.c` | 压力 | 启动转换 → 读 24 位 → 换算 kPa |
| `ds18b20_dual.h/.c` | 温度（**独立版**） | 2 路 DS18B20 各占一根 GPIO（PG10/PG9） |
| `ds18b20_shared.h/.c` | 温度（**共享版**） | 2 路 DS18B20 共用一个 GPIO，ROM 匹配 |
| `flowmeter.h/.c` | 流量 | 脉冲计数 → 累计流量 / 瞬时流量 |
| `relay.h/.c` | 继电器 | 385泵 / 蠕动泵 / 加热棒 开关控制 |

> ⚠️ **温度驱动有两个版本，二选一**，不要同时加入 Keil：
> - **独立版** `ds18b20_dual`：两个传感器各占一根 GPIO（PG10 / PG9），Skip ROM，最简单。
> - **共享版** `ds18b20_shared`：两个传感器共用一个 GPIO（默认 PG10），需先 ROM 搜索再 Match ROM。

## 三、接入 Keil 要做的事（审阅通过后）

1. 把 `App/` 目录下**用到的** `.c` 文件加入 Keil 工程（Add Group 名为 `App`）。
2. 把 `App/` 加入 Include Paths。
3. 在 `main.c` 中：
   - `#include` 对应头文件（温度按你选的版本 include `ds18b20_dual.h` 或 `ds18b20_shared.h`）
   - `MX_GPIO_Init()` 之后调用各模块 `*_Init()`（顺序：`BSP_Delay_Init` 先，其余任意）。

## 四、几个需要你核对/注意的点 ⚠️

1. **继电器吸合电平**：默认 `RELAY_ACTIVE_LEVEL = GPIO_PIN_RESET`（低电平吸合）。
   如果你的继电器驱动电路是高电平吸合，改 `relay.h` 里这一个宏即可。
2. **压力传感器量程**：默认 `PRESSURE_FULL_KPA = 1600.0f`（1.6 MPa）。请按 P31/P32/P33
   具体型号的数据手册核对。
3. **流量计系数**：默认 `FLOW_PULSES_PER_LITER = 450.0f`。请按你的流量计型号核对。
4. **5V / 3.3V 电平**：压力/流量/温度传感器都是 5V 供电。本驱动所有相关引脚均配置为
   **开漏输出 + 上拉**，PB8/PB9/PG9/PG10/PG15 在 F407 上都是 FT(5V 容忍) 引脚，可直接
   接 5V 上拉总线而无需电平转换。请确认你的模块上拉电阻不超过 ~5.5V。
5. **DS18B20 非阻塞状态机（阶段②已完成）**：`ds18b20_dual.c` 已改为非阻塞状态机，
   主循环调 `DS18B20_Dual_Process()` 推进，750ms 转换期间不阻塞 LWIP；
   上报/采集读缓存 `DS18B20_Dual_GetTemp()`/`GetStatus()`，不要再调用阻塞的 `ReadTemp()`。
6. **PG15 的 EXTI** 已在 `FlowMeter_Init()` 里手动配置，**不要在 CubeMX 里再把 PG15 配成
   EXTI**，否则会重复定义 `EXTI15_10_IRQHandler`。
7. **共享版温度要填 ROM 码**：用共享版时，先调 `DS18B20_SearchRom()` 搜出两个传感器的
   64 位 ROM 码，填进 `ds18b20_shared.h` 的 `DS18B20_UPPER_ROM` / `DS18B20_LOWER_ROM`。
   区分哪个是上/下水槽：只给上水槽加热，看哪个码的温度先变。
