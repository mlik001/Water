/**
  ******************************************************************************
  * @file    app_host.c
  * @brief   上位机串口模拟（临时联调用）实现
  * @note    命令集（换行结束，大小写不敏感）：
  *            MOTOR  ON|OFF   电机(385泵)   别名 M1 / M
  *            PUMP   ON|OFF   蠕动泵        别名 M2 / P
  *            HEATER ON|OFF   加热棒        别名 M3
  *            ALL    ON|OFF   全部开关
  *            STATUS / ST     查看当前继电器状态
  *            DATA   / READ   立即采集并上报一次
  *            CLR             流量计累计清零
  *            HELP   / ?      打印帮助
  *
  *          定时器用 TIM6（APB1，84MHz），每 1s 进一次中断累加心跳计数，
  *          主循环里按 HOST_COLLECT_INTERVAL_S 决定是否采集，采集动作本身
  *          放在主循环（非中断）里，避免在中断里做阻塞的 1-Wire/I2C 读写。
  ******************************************************************************
  */
#include "app_host.h"
#include "usart.h"              /* huart4 */
#include <stdio.h>
#include <string.h>

#include "relay.h"
#include "flowmeter.h"
#include "pressure_sensor.h"
#include "ds18b20_dual.h"
#include "bsp_soft_i2c.h"        /* ISCAN/PRAW 诊断用 */

/* ---------------- 定时器 TIM6 ---------------- */
static TIM_HandleTypeDef htim6;
static volatile uint32_t g_sec_ticks = 0;     /* 1s 心跳计数（ISR 累加） */

/* ---------------- UART4 接收 ---------------- */
static uint8_t  g_rx_byte;                      /* 单字节接收缓冲 */
static char     g_rx_line[HOST_RX_LINE_MAX];    /* 命令行缓冲 */
static volatile uint16_t g_rx_len = 0;          /* 已收字节数 */
static volatile uint8_t  g_cmd_ready = 0;       /* 收到完整一行 */

/* ---------------- 继电器状态（用于上报） ---------------- */
static uint8_t g_motor_on  = 0;   // 默认关闭
static uint8_t g_pump_on   = 0;
static uint8_t g_heater_on = 0;

/* ---------------- 内部函数 ---------------- */
static void AppHost_Tim_Init(void);
static void AppHost_UartRx_Init(void);
static void AppHost_CollectAndReport(void);
static void AppHost_Dispatch(char *line);
static void AppHost_PrintHelp(void);
static void AppHost_PrintStatus(void);
static void AppHost_PrintFixed(float v, int decimals);

/* ================= 公开接口 ================= */

/**
  * @brief  启动定时器 + 串口接收，并打印命令帮助
  */
void AppHost_Init(void)
{
    AppHost_Tim_Init();
    AppHost_UartRx_Init();
    AppHost_PrintHelp();
}

/**
  * @brief  主循环调用：①定时采集上报 ②处理串口命令
  */
void AppHost_Process(void)
{
    /* 注意：定时自动采集上报已迁移到 app_tcp.c（TCP 上位机，JSON 协议）。
       这里仅保留 UART4 调试/回退命令通道（DATA 命令仍可手动采集一次）。
       TIM6 心跳仍在跑，g_sec_ticks 暂未使用，保留备用。 */

    /* 处理串口命令（调试/回退通道） */
    if (g_cmd_ready)
    {
        char line[HOST_RX_LINE_MAX];

        /* 关中断拷出，避免 ISR 与主循环同时读写 g_rx_line */
        __disable_irq();
        g_cmd_ready = 0;
        strncpy(line, g_rx_line, sizeof(line) - 1);
        line[sizeof(line) - 1] = '\0';
        __enable_irq();

        AppHost_Dispatch(line);
    }
}

/* ================= 定时器 ================= */

static void AppHost_Tim_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 8400 - 1;        /* 84MHz / 8400 = 10kHz */
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 10000 - 1;          /* 10kHz / 10000 = 1s */
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    HAL_TIM_Base_Start_IT(&htim6);
}

/**
  * @brief  TIM6 更新中断：每 1s 累加一次心跳计数
  */
void TIM6_DAC_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
        g_sec_ticks++;
    }
}

/* ================= UART4 接收 ================= */

static void AppHost_UartRx_Init(void)
{
    HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
    HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1);
}

/**
  * @brief  UART4 中断入口（调 HAL 通用处理，最终回调 HAL_UART_RxCpltCallback）
  */
void UART4_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart4);
}

/**
  * @brief  UART4 单字节接收完成回调：拼行，遇 \r / \n 视为一行结束
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    char c;

    if (huart->Instance != UART4)
    {
        return;
    }

    c = (char)g_rx_byte;

    if (c == '\r' || c == '\n')
    {
        if (g_rx_len > 0)
        {
            g_rx_line[g_rx_len] = '\0';
            g_rx_len = 0;
            g_cmd_ready = 1;
        }
    }
    else if (g_rx_len < HOST_RX_LINE_MAX - 1)
    {
        g_rx_line[g_rx_len++] = c;
    }
    /* 超出缓冲区长度则丢弃，防止越界 */

    HAL_UART_Receive_IT(&huart4, &g_rx_byte, 1);   /* 重新装载 */
}

/* ================= 采集上报 ================= */

static void AppHost_CollectAndReport(void)
{
    float   t_up = 0.0f, t_dn = 0.0f, pressure = 0.0f;
    uint8_t t1_ok, t2_ok, p_ok;

    /* 非阻塞：直接读状态机后台刷新的温度缓存 */
    t_up  = DS18B20_Dual_GetTemp(DS18B20_DUAL_UPPER);
    t_dn  = DS18B20_Dual_GetTemp(DS18B20_DUAL_LOWER);
    t1_ok = DS18B20_Dual_GetStatus(DS18B20_DUAL_UPPER);
    t2_ok = DS18B20_Dual_GetStatus(DS18B20_DUAL_LOWER);
    p_ok  = PressureSensor_ReadKpa(&pressure);

    printf("[DATA] ");

    if (t1_ok == 0) { printf("T_up="); AppHost_PrintFixed(t_up, 2); printf("C "); }
    else            { printf("T_up=ERR%d ", t1_ok); }

    if (t2_ok == 0) { printf("T_dn="); AppHost_PrintFixed(t_dn, 2); printf("C "); }
    else            { printf("T_dn=ERR%d ", t2_ok); }

    if (p_ok  == 0) { printf("P=");     AppHost_PrintFixed(pressure, 2); printf("kPa "); }
    else            { printf("P=ERR%d ", p_ok); }

    printf("F="); AppHost_PrintFixed(FlowMeter_GetFlowRate(), 2);
    printf("L/min V="); AppHost_PrintFixed(FlowMeter_GetTotalVolume(), 3);
    printf("L M=%s Pmp=%s Heat=%s\r\n",
           g_motor_on  ? "ON" : "OFF",
           g_pump_on   ? "ON" : "OFF",
           g_heater_on ? "ON" : "OFF");
}

/* ================= 命令分发 ================= */

static void AppHost_Dispatch(char *line)
{
    char work[HOST_RX_LINE_MAX];
    char cmd[16], arg[16];
    char *tok, *p;
    uint8_t v;

    /* 调试：打印原始行 */
    printf("[DBG] raw line: '%s'\r\n", line);

    /* 转成大写后解析，原始 line 保留用于报错 */
    strncpy(work, line, sizeof(work) - 1);
    work[sizeof(work) - 1] = '\0';
    for (p = work; *p; p++)
    {
        if (*p >= 'a' && *p <= 'z') *p -= 32;
    }

    tok = strtok(work, " \t");
    if (tok == NULL)
    {
        return;
    }
    strncpy(cmd, tok, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';

    tok = strtok(NULL, " \t");
    strncpy(arg, tok ? tok : "", sizeof(arg) - 1);
    arg[sizeof(arg) - 1] = '\0';

    /* 调试：打印提取的命令和参数 */
    printf("[DBG] cmd='%s', arg='%s'\r\n", cmd, arg);

    /* ---- 帮助 / 状态 / 立即采集 / 清零 ---- */
    if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0)
    {
        AppHost_PrintHelp();
        return;
    }
    if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "ST") == 0)
    {
        AppHost_PrintStatus();
        return;
    }
    if (strcmp(cmd, "DATA") == 0 || strcmp(cmd, "READ") == 0)
    {
        AppHost_CollectAndReport();
        return;
    }
    if (strcmp(cmd, "CLR") == 0 || strcmp(cmd, "RESET") == 0)
    {
        FlowMeter_Reset();
        printf("[OK] flow counter cleared\r\n");
        return;
    }
    if (strcmp(cmd, "ISCAN") == 0)
    {
        uint8_t a, found = 0;
        printf("[ISCAN]");
        for (a = 0x08; a <= 0x77; a++)
        {
            if (SoftI2C_Probe(a))
            {
                printf(" 0x%02X", a);
                found = 1;
            }
        }
        if (!found) printf(" (none)");
        printf("\r\n");
        return;
    }
    if (strcmp(cmd, "PRAW") == 0)
    {
        uint8_t ctrl = 0, buf[3] = {0, 0, 0};
        uint8_t e1, e2;
        e1 = SoftI2C_ReadRegs(PRESSURE_ADDR_R, PRESSURE_REG_CTRL, &ctrl, 1);
        e2 = SoftI2C_ReadRegs(PRESSURE_ADDR_R, PRESSURE_REG_DATA_H, buf, 3);
        printf("[PRAW] ctrl=0x%02X busy=%d ack=%s | data=%02X %02X %02X ack=%s\r\n",
               ctrl, (ctrl >> 5) & 0x01, e1 ? "NACK" : "OK",
               buf[0], buf[1], buf[2], e2 ? "NACK" : "OK");
        return;
    }
    if (strcmp(cmd, "PZERO") == 0)
    {
        int32_t off = PressureSensor_CalibrateZero();
        printf("[PZERO] zero offset raw=%ld\r\n", (long)off);
        return;
    }

    /* ---- 开关类命令（直接字符串比较，不调用任何解析函数） ---- */
    if (strcmp(cmd, "MOTOR") == 0 || strcmp(cmd, "M1") == 0 || strcmp(cmd, "M") == 0)
    {
        if (strcmp(arg, "ON") == 0) {
            v = 1;
        } else if (strcmp(arg, "OFF") == 0) {
            v = 0;
        } else {
            printf("[ERR] usage: MOTOR ON|OFF\r\n");
            return;
        }
        g_motor_on = v;
        Relay_Motor(g_motor_on ? RELAY_ON : RELAY_OFF);
        printf("[OK] Motor %s\r\n", g_motor_on ? "ON" : "OFF");
        return;
    }
    if (strcmp(cmd, "PUMP") == 0 || strcmp(cmd, "M2") == 0 || strcmp(cmd, "P") == 0)
    {
        if (strcmp(arg, "ON") == 0) {
            v = 1;
        } else if (strcmp(arg, "OFF") == 0) {
            v = 0;
        } else {
            printf("[ERR] usage: PUMP ON|OFF\r\n");
            return;
        }
        g_pump_on = v;
        Relay_Pump(g_pump_on ? RELAY_ON : RELAY_OFF);
        printf("[OK] Pump %s\r\n", g_pump_on ? "ON" : "OFF");
        return;
    }
    if (strcmp(cmd, "HEATER") == 0 || strcmp(cmd, "M3") == 0)
    {
        if (strcmp(arg, "ON") == 0) {
            v = 1;
        } else if (strcmp(arg, "OFF") == 0) {
            v = 0;
        } else {
            printf("[ERR] usage: HEATER ON|OFF\r\n");
            return;
        }
        g_heater_on = v;
        Relay_Heater(g_heater_on ? RELAY_ON : RELAY_OFF);
        printf("[OK] Heater %s\r\n", g_heater_on ? "ON" : "OFF");
        return;
    }
    if (strcmp(cmd, "ALL") == 0)
    {
        if (strcmp(arg, "ON") == 0) {
            v = 1;
        } else if (strcmp(arg, "OFF") == 0) {
            v = 0;
        } else {
            printf("[ERR] usage: ALL ON|OFF\r\n");
            return;
        }
        g_motor_on = g_pump_on = g_heater_on = v;
        Relay_Motor(g_motor_on ? RELAY_ON : RELAY_OFF);
        Relay_Pump(g_pump_on ? RELAY_ON : RELAY_OFF);
        Relay_Heater(g_heater_on ? RELAY_ON : RELAY_OFF);
        printf("[OK] All %s\r\n", g_motor_on ? "ON" : "OFF");
        return;
    }

    /* ---- 未知命令 ---- */
    printf("[ERR] unknown: %s (type HELP)\r\n", line);
}

/* ================= 辅助 ================= */

static void AppHost_PrintHelp(void)
{
    printf("================ HOST CMD ================\r\n");
    printf("MOTOR  ON|OFF   motor(385 pump)  [M1/M]\r\n");
    printf("PUMP   ON|OFF   peristaltic pump [M2/P]\r\n");
    printf("HEATER ON|OFF   heater           [M3]\r\n");
    printf("ALL    ON|OFF   all relays\r\n");
    printf("STATUS / ST     show relay state\r\n");
    printf("DATA   / READ   sample & report now\r\n");
    printf("CLR             clear flow volume\r\n");
    printf("ISCAN           scan I2C bus addresses\r\n");
    printf("PRAW            dump pressure regs (ACK test)\r\n");
    printf("PZERO           zero-calibrate pressure (0 bar)\r\n");
    printf("HELP   / ?      this help\r\n");
    printf("==========================================\r\n");
}

static void AppHost_PrintStatus(void)
{
    printf("[STATUS] Motor=%s Pump=%s Heater=%s\r\n",
           g_motor_on  ? "ON" : "OFF",
           g_pump_on   ? "ON" : "OFF",
           g_heater_on ? "ON" : "OFF");
}

/**
  * @brief  打印固定小数位的浮点数（不用 printf 的 %f，兼容 microlib / 标准库）
  * @param  decimals: 支持 1 / 2 / 3 位小数
  */
static void AppHost_PrintFixed(float v, int decimals)
{
    int   ip, fp, div = 1, i;
    float av = v;

    if (av < 0.0f)
    {
        printf("-");
        av = -av;
    }

    for (i = 0; i < decimals; i++) div *= 10;

    ip = (int)av;
    fp = (int)((av - (float)ip) * (float)div + 0.5f);
    if (fp >= div)                       /* 处理四舍五入进位 */
    {
        fp -= div;
        ip += 1;
    }

    printf("%d", ip);
    if (decimals > 0)
    {
        printf(".");
        if (decimals == 1)      printf("%d", fp);
        else if (decimals == 2) printf("%02d", fp);
        else                    printf("%03d", fp);
    }
}