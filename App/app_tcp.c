/**
  ******************************************************************************
  * @file    app_tcp.c
  * @brief   饮水机系统 TCP 后端通讯（最终阶段）—— JSON 协议
  *
  * @note    STM32 作为 TCP 客户端，主动连接后端服务器（IP/端口见 app_tcp.h）。
  *          使用 LwIP 裸机 raw API（NO_SYS=1，无 RTOS）：
  *            tcp_new -> tcp_connect -> AppTcp_Connected
  *            tcp_recv 回调收命令，tcp_write + tcp_output 回包。
  *          断线/连接失败由 AppTcp_Process()（主循环调用）驱动自动重连。
  *
  *  ═══════════════════  JSON 协议（对接 water-cycle 后端）  ═══════════════════
  *  每条报文以 \r\n 结尾，上行报文均带 deviceId。
  *
  *  ◆ 握手：
  *      设备 → 后端:  {"type":"hello","deviceId":"WC-001","device":"water_dispenser",
  *                    "firmware":"1.0.0","ip":"<dhcp_ip>","proto":"json"}
  *      后端 → 设备:  {"type":"hello_ack","deviceId":"WC-001","result":"ok",
  *                    "reportIntervalMs":2000,"heartbeatIntervalMs":5000}
  *
  *  ◆ 遥测（收到 hello_ack 后按 reportIntervalMs 周期上报）：
  *      {"type":"data","deviceId":"WC-001","seq":1,"uptime_s":10,"temp_up":25.5,
  *       "temp_dn":24.8,"pressure_kpa":101.3,"flow_rate":0.0,"flow_volume":0.0,
  *       "motor":0,"pump":0,"heater":0}
  *      {"type":"status","deviceId":"WC-001","motor":0,"pump":0,"heater":0}
  *
  *  ◆ 动作命令（后端 → 设备 → 回 ack）：
  *      {"type":"command","deviceId":"WC-001","commandId":"cmd-001","cmd":"motor","value":"on"}
  *      {"type":"ack","deviceId":"WC-001","commandId":"cmd-001","cmd":"motor","value":"on","result":"ok"}
  *      cmd = motor(上水泵)|pump(循环泵)|heater(加热棒)|all；value = on|off
  *
  *  ◆ 报警配置（后端 → 设备 → 回 config_ack）：
  *      {"type":"config","deviceId":"WC-001","commandId":"config-001",
  *       "temperatureAlarm":{"sensorKey":"temperature_2","threshold":40.5}}   (threshold 为 float，可带小数)
  *      {"type":"config_ack","deviceId":"WC-001","commandId":"config-001","result":"ok"}
  *      sensorKey: temperature_1(上水槽 temp_up) | temperature_2(下水槽 temp_dn)
  *
  *  DS18B20 已改为非阻塞状态机（阶段②完成）：主循环 DS18B20_Dual_Process()
  *      每 ~750ms 后台刷新一路温度缓存，上报直接读缓存（GetTemp/GetStatus），
  *      不再阻塞 LwIP，TCP 无采集延迟。
  ******************************************************************************
  */
#include "app_tcp.h"
#include "lwip/tcp.h"
#include "lwip/pbuf.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include <stdio.h>
#include <string.h>

#include "relay.h"
#include "flowmeter.h"
#include "pressure_sensor.h"
#include "ds18b20_dual.h"
#include "bsp_soft_i2c.h"
#include "buzzer.h"

/* ---------------- TCP 客户端状态 ---------------- */
extern struct netif gnetif;                    /* LwIP 默认网卡（取 DHCP 后的 IP 用于 hello） */
static struct tcp_pcb *g_client_pcb = NULL;   /* 与后端服务器的连接 PCB（单连接） */

/* ---------------- 重连状态 ---------------- */
static uint8_t  g_reconnect_pending = 0;      /* 是否需要重连 */
static uint32_t g_reconnect_time    = 0;      /* 触发重连的时刻（HAL_GetTick） */

/* ---------------- 握手状态 ---------------- */
static uint8_t  g_hello_acked = 0;            /* 是否收到后端 hello_ack（握手成功才允许上报） */
static uint32_t g_hello_sent_time = 0;        /* 上次发送 hello 的时刻（HAL_GetTick，用于超时重发） */

/* ---------------- 上报序列号 ---------------- */
static uint32_t g_data_seq = 0;               /* data 上报序号，每次自增（JSON 里为不带引号/小数点的整数） */

/* ---------------- 命令行缓冲 ---------------- */
static char     g_rx_line[APP_TCP_RX_LINE_MAX];
static uint16_t g_rx_len = 0;

/* ---------------- 继电器状态（语义化：1=ON, 0=OFF） ---------------- */
static uint8_t g_motor_on  = 0;
static uint8_t g_pump_on   = 0;
static uint8_t g_heater_on = 0;

/* ---------------- 上报周期（hello_ack 可下发 reportIntervalMs 覆盖） ---------------- */
static uint32_t g_report_interval_ms    = 1000;   /* 遥测上报周期，默认 2s */
static uint32_t g_heartbeat_interval_ms = 5000;   /* 心跳周期，默认 5s（data 上报即心跳，暂不单发） */
static uint32_t g_last_report_time      = 0;      /* 上次上报 data 的时刻（HAL_GetTick） */

/* ---------------- 报警配置（后端 config 下发） ---------------- */
static uint8_t  g_alarm_configured = 0;           /* 是否已配置报警 */
static uint8_t  g_alarm_sensor     = 0;           /* 0=temperature_1(上水槽) 1=temperature_2(下水槽) */
static float    g_alarm_threshold  = 0.0f;   
/* 报警阈值（℃，float，支持小数） */

/* ---------------- 内部函数 ---------------- */
static err_t AppTcp_Connected(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t AppTcp_Recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void   AppTcp_Error(void *arg, err_t err);
static err_t AppTcp_Poll(void *arg, struct tcp_pcb *tpcb);
static void   AppTcp_Connect(void);
static void   AppTcp_ScheduleReconnect(void);

static void AppTcp_Disconnect(struct tcp_pcb *pcb);
static void AppTcp_SendBuf(const char *buf);
static void AppTcp_SendHello(void);
static void AppTcp_FeedByte(uint8_t c);
static void AppTcp_Dispatch(const char *json);
static void AppTcp_ExecCommand(const char *cmd, const char *value, const char *commandId);
static void AppTcp_HandleConfig(const char *json, const char *commandId);
static void AppTcp_ReportData(void);
static void AppTcp_ReportStatus(void);
static uint32_t AppTcp_AlarmBeepPeriod(float overshoot);
static void AppTcp_Ack(const char *commandId, const char *cmd, const char *value, const char *result, const char *msg);

/* JSON 解析辅助（手写，仅针对本协议的固定 schema，零依赖） */
static const char *JsonSkipWs(const char *p);
static int  JsonGetString(const char *json, const char *key, char *out, int maxlen);
static int  JsonGetInt(const char *json, const char *key, int *out);
static int  JsonGetFloat(const char *json, const char *key, float *out);

/* 数值格式化辅助（不用 %f，兼容 microlib） */
static int  AppTcp_FmtFloat(char *dst, float v, int decimals);
static void AppTcp_StrLower(char *s);

/* ================= 公开接口 ================= */

/**
  * @brief  发起首次连接后端服务器（断线重连由 AppTcp_Process 驱动）
  */
void AppTcp_Init(void)
{
    g_reconnect_pending = 0;
    AppTcp_Connect();
}

/**
  * @brief  主循环调用：断线/连接失败后按 APP_TCP_RECONNECT_INTERVAL_MS 自动重连
  */
void AppTcp_Process(void)
{
    if (g_client_pcb == NULL && g_reconnect_pending)
    {
        if (HAL_GetTick() - g_reconnect_time >= APP_TCP_RECONNECT_INTERVAL_MS)
        {
            g_reconnect_pending = 0;
            AppTcp_Connect();
        }
    }
}

/**
  * @brief  标记需要重连并记录时刻（真正发起由 AppTcp_Process 按间隔执行）
  */
static void AppTcp_ScheduleReconnect(void)
{
    g_reconnect_pending = 1;
    g_reconnect_time    = HAL_GetTick();
}

/**
  * @brief  主动连接后端服务器（tcp_connect 异步，结果在 AppTcp_Connected 回调）
  */
static void AppTcp_Connect(void)
{
    ip_addr_t server_ip;
    err_t     e;

    if (g_client_pcb != NULL)
    {
        return;   /* 已在连接中或已连上 */
    }

    g_client_pcb = tcp_new();
    if (g_client_pcb == NULL)
    {
        AppTcp_ScheduleReconnect();   /* 内存不足，稍后重试 */
        return;
    }

    tcp_arg(g_client_pcb, NULL);
    tcp_err(g_client_pcb, AppTcp_Error);   /* 连接阶段出错（RST/超时）也走这里 */

    IP_ADDR4(&server_ip,
             APP_TCP_SERVER_IP_A, APP_TCP_SERVER_IP_B,
             APP_TCP_SERVER_IP_C, APP_TCP_SERVER_IP_D);

    e = tcp_connect(g_client_pcb, &server_ip, APP_TCP_PORT, AppTcp_Connected);
    if (e != ERR_OK)
    {
        /* 立即失败（无路由/内存等）：释放并稍后重试（tcp_close 不触发 AppTcp_Error） */
        tcp_close(g_client_pcb);
        g_client_pcb = NULL;
        AppTcp_ScheduleReconnect();
    }
}

/**
  * @brief  LwIP 慢定时器轮询回调：定时采集上报
  * @note   由 tcp_poll 注册（interval=1，即 0.5s 触发一次），实际上报周期由 g_report_interval_ms 控制。
  *          不用主循环轮询，也不用硬件定时器。
  */
static err_t AppTcp_Poll(void *arg, struct tcp_pcb *tpcb)
{
    (void)arg;
    (void)tpcb;

    /* 未收到 hello_ack 前不上报 data；超时未握手则重发 hello */
    if (!g_hello_acked)
    {
        if (HAL_GetTick() - g_hello_sent_time >= APP_TCP_HELLO_RETRY_MS)
        {
            g_hello_sent_time = HAL_GetTick();
            AppTcp_SendHello();
        }
        return ERR_OK;
    }

    if (HAL_GetTick() - g_last_report_time >= g_report_interval_ms)
    {
        g_last_report_time = HAL_GetTick();
        AppTcp_ReportData();
    }
    return ERR_OK;
}

/* ================= TCP 回调 ================= */

/**
  * @brief  tcp_connect 完成回调：连上后端服务器后挂接收/上报并发送握手
  * @note   err==ERR_OK 表示连接成功；否则连接失败（LwIP 已释放 PCB），交给重连逻辑
  */
static err_t AppTcp_Connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
    (void)arg;

    if (err != ERR_OK || tpcb == NULL)
    {
        g_client_pcb = NULL;
        AppTcp_ScheduleReconnect();
        return err;
    }

    g_client_pcb = tpcb;
    tcp_setprio(tpcb, TCP_PRIO_MIN);
    tcp_arg(tpcb, NULL);
    tcp_recv(tpcb, AppTcp_Recv);
    tcp_err(tpcb, AppTcp_Error);
    /* 定时上报：由 LwIP 慢定时器(tcp_slowtmr)驱动，interval=1(0.5s)，实际周期由 g_report_interval_ms 控制 */
    tcp_poll(tpcb, AppTcp_Poll, 1);

    /* 握手：先发 hello，收到后端 hello_ack 后才允许上报 data/status */
    g_hello_acked = 0;
    g_hello_sent_time = HAL_GetTick();
    AppTcp_SendHello();

    return ERR_OK;
}

/**
  * @brief  收到数据：逐字节喂给命令行解析器
  */
static err_t AppTcp_Recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    struct pbuf *q;
    u16_t i;

    (void)arg;
    (void)err;

    if (p == NULL)
    {
        /* 对端关闭连接 */
        AppTcp_Disconnect(tpcb);
        return ERR_OK;
    }

    for (q = p; q != NULL; q = q->next)
    {
        uint8_t *d = (uint8_t *)q->payload;
        for (i = 0; i < q->len; i++)
        {
            AppTcp_FeedByte(d[i]);
        }
    }

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}

/**
  * @brief  连接出错：LwIP 会释放 PCB，这里只需清空指针
  */
static void AppTcp_Error(void *arg, err_t err)
{
    (void)arg;
    (void)err;
    /* LwIP 出错后会自行释放 PCB，这里只清指针并安排重连 */
    g_client_pcb = NULL;
    AppTcp_ScheduleReconnect();
}

/**
  * @brief  断开当前连接（优雅关闭）
  */
static void AppTcp_Disconnect(struct tcp_pcb *pcb)
{
    if (g_client_pcb == pcb)
    {
        g_client_pcb = NULL;
    }

    if (pcb != NULL)
    {
        tcp_arg(pcb, NULL);
        tcp_recv(pcb, NULL);
        tcp_err(pcb, NULL);
        tcp_poll(pcb, NULL, 0);   /* 停掉定时上报，避免在关闭中的 PCB 上再触发 */
        tcp_close(pcb);
    }

    AppTcp_ScheduleReconnect();   /* 后端断开后自动重连 */
}

/* ================= 发送 ================= */

/**
  * @brief  向后端服务器发送一段字符串
  */
static void AppTcp_SendBuf(const char *buf)
{
    u16_t len;

    if (g_client_pcb == NULL || buf == NULL)
    {
        return;
    }

    len = (u16_t)strlen(buf);
    if (len == 0)
    {
        return;
    }

    /* 发送缓冲不足则丢弃（小报文一般不会触发），避免阻塞 */
    if (tcp_sndbuf(g_client_pcb) < len)
    {
        return;
    }

    if (tcp_write(g_client_pcb, buf, len, TCP_WRITE_FLAG_COPY) == ERR_OK)
    {
        tcp_output(g_client_pcb);
    }
}

/**
  * @brief  发送 hello 握手包（含 deviceId/firmware，后端回 hello_ack 后握手完成）
  */
static void AppTcp_SendHello(void)
{
    char line[128];
    sprintf(line, "{\"type\":\"hello\",\"deviceId\":\"%s\",\"device\":\"water_dispenser\",\"firmware\":\"1.0.0\",\"ip\":\"%s\",\"proto\":\"json\"}\r\n",
            APP_TCP_DEVICE_ID, ip4addr_ntoa(netif_ip4_addr(&gnetif)));
    AppTcp_SendBuf(line);
}

/* ================= 接收解析 ================= */

/**
  * @brief  逐字节拼行，遇 \r / \n 视为一条命令结束
  */
static void AppTcp_FeedByte(uint8_t c)
{
    if (c == '\r' || c == '\n')
    {
        if (g_rx_len > 0)
        {
            g_rx_line[g_rx_len] = '\0';
            AppTcp_Dispatch(g_rx_line);
            g_rx_len = 0;
        }
    }
    else if (g_rx_len < APP_TCP_RX_LINE_MAX - 1)
    {
        g_rx_line[g_rx_len++] = (char)c;
    }
    /* 超出缓冲区长度则丢弃，防止越界 */
}

/**
  * @brief  解析一条 JSON 命令并执行
  */
static void AppTcp_Dispatch(const char *json)
{
    char type[16]      = "";
    char cmd[16]       = "";
    char value[16]     = "";
    char commandId[65] = "";

    /* 调试：打印后端下发的原始报文，在 UART4(115200) 观察后端实际发了什么 */
    printf("[TCP RX] %s\r\n", json);

    if (!JsonGetString(json, "type", type, sizeof(type)))
    {
        /* 无 type 字段：兼容旧式 {"cmd":...} 直发命令（老测试脚本） */
        if (JsonGetString(json, "cmd", cmd, sizeof(cmd)))
        {
            AppTcp_StrLower(cmd);
            JsonGetString(json, "value", value, sizeof(value));
            AppTcp_StrLower(value);
            AppTcp_ExecCommand(cmd, value, "");
        }
        else
        {
            AppTcp_Ack("", "unknown", "", "err", "missing type and cmd field");
        }
        return;
    }

    AppTcp_StrLower(type);

    /* 握手应答：后端返回 hello_ack 后握手成功，并按需更新上报/心跳周期 */
    if (strcmp(type, "hello_ack") == 0)
    {
        int v;
        g_hello_acked = 1;
        if (JsonGetInt(json, "reportIntervalMs", &v) && v > 0)
        {
            g_report_interval_ms = (uint32_t)v;
        }
        if (JsonGetInt(json, "heartbeatIntervalMs", &v) && v > 0)
        {
            g_heartbeat_interval_ms = (uint32_t)v;
        }
        return;
    }

    /* 后端心跳/探测：回 pong，不要当成命令报错 */
    if (strcmp(type, "ping") == 0 || strcmp(type, "heartbeat") == 0 ||
        strcmp(type, "keepalive") == 0)
    {
        AppTcp_SendBuf("{\"type\":\"pong\"}\r\n");
        return;
    }

    /* 普通动作命令：{"type":"command","commandId":...,"cmd":"motor","value":"on"} */
    if (strcmp(type, "command") == 0)
    {
        JsonGetString(json, "commandId", commandId, sizeof(commandId));
        if (!JsonGetString(json, "cmd", cmd, sizeof(cmd)))
        {
            AppTcp_Ack(commandId, "unknown", "", "err", "missing cmd field");
            return;
        }
        AppTcp_StrLower(cmd);
        JsonGetString(json, "value", value, sizeof(value));
        AppTcp_StrLower(value);
        AppTcp_ExecCommand(cmd, value, commandId);
        return;
    }

    /* 报警配置：{"type":"config","commandId":...,"temperatureAlarm":{...}} */
    if (strcmp(type, "config") == 0)
    {
        JsonGetString(json, "commandId", commandId, sizeof(commandId));
        AppTcp_HandleConfig(json, commandId);
        return;
    }

    /* 未知 type：回错误 ack（cmd 用非空，避免后端判 cmd 为空） */
    AppTcp_Ack("", "unknown", "", "err", "unknown type");
}

/**
  * @brief  执行一条动作命令，并回 ack（带 deviceId/commandId/value）
  */
static void AppTcp_ExecCommand(const char *cmd, const char *value, const char *commandId)
{
    /* ---- 查询 / 帮助 ---- */
    if (strcmp(cmd, "help") == 0)
    {
        AppTcp_SendBuf("{\"type\":\"help\",\"cmds\":\"motor|pump|heater|all value=on/off; status; data; clr; iscan; praw; pzero\"}\r\n");
        return;
    }
    if (strcmp(cmd, "status") == 0)
    {
        AppTcp_ReportStatus();
        return;
    }
    if (strcmp(cmd, "data") == 0)
    {
        AppTcp_ReportData();
        return;
    }
    if (strcmp(cmd, "clr") == 0)
    {
        FlowMeter_Reset();
        AppTcp_Ack(commandId, "clr", "", "ok", "");
        return;
    }
    if (strcmp(cmd, "iscan") == 0)
    {
        uint8_t a, found = 0;
        char line[160];
        int off = 0;
        off += sprintf(line + off, "{\"type\":\"iscan\",\"addrs\":[");
        for (a = 0x08; a <= 0x77; a++)
        {
            if (SoftI2C_Probe(a))
            {
                if (found) off += sprintf(line + off, ",");
                off += sprintf(line + off, "\"0x%02X\"", a);
                found = 1;
            }
        }
        off += sprintf(line + off, "]}\r\n");
        AppTcp_SendBuf(line);
        return;
    }
    if (strcmp(cmd, "praw") == 0)
    {
        uint8_t ctrl = 0, buf[3] = {0, 0, 0};
        uint8_t e1, e2;
        char line[128];
        e1 = SoftI2C_ReadRegs(PRESSURE_ADDR_R, PRESSURE_REG_CTRL, &ctrl, 1);
        e2 = SoftI2C_ReadRegs(PRESSURE_ADDR_R, PRESSURE_REG_DATA_H, buf, 3);
        sprintf(line, "{\"type\":\"praw\",\"ctrl\":%u,\"busy\":%u,\"data\":[%u,%u,%u],\"ack\":\"%s\"}\r\n",
                ctrl, (ctrl >> 5) & 0x01, buf[0], buf[1], buf[2], (e1 || e2) ? "nack" : "ok");
        AppTcp_SendBuf(line);
        return;
    }
    if (strcmp(cmd, "pzero") == 0)
    {
        int32_t off = PressureSensor_CalibrateZero();
        char line[64];
        sprintf(line, "{\"type\":\"pzero\",\"zero_raw\":%ld}\r\n", (long)off);
        AppTcp_SendBuf(line);
        return;
    }

    /* ---- 开关类命令 ---- */
    if (strcmp(cmd, "motor") == 0 || strcmp(cmd, "m1") == 0 || strcmp(cmd, "m") == 0)
    {
        if (strcmp(value, "on") == 0)
        {
            g_motor_on = 1;
            Relay_Motor(RELAY_ON);
            AppTcp_Ack(commandId, "motor", "on", "ok", "");
        }
        else if (strcmp(value, "off") == 0)
        {
            g_motor_on = 0;
            Relay_Motor(RELAY_OFF);
            AppTcp_Ack(commandId, "motor", "off", "ok", "");
        }
        else
        {
            AppTcp_Ack(commandId, "motor", value, "err", "value must be on/off");
        }
        return;
    }
    if (strcmp(cmd, "pump") == 0 || strcmp(cmd, "m2") == 0 || strcmp(cmd, "p") == 0)
    {
        if (strcmp(value, "on") == 0)
        {
            g_pump_on = 1;
            Relay_Pump(RELAY_ON);
            AppTcp_Ack(commandId, "pump", "on", "ok", "");
        }
        else if (strcmp(value, "off") == 0)
        {
            g_pump_on = 0;
            Relay_Pump(RELAY_OFF);
            AppTcp_Ack(commandId, "pump", "off", "ok", "");
        }
        else
        {
            AppTcp_Ack(commandId, "pump", value, "err", "value must be on/off");
        }
        return;
    }
    if (strcmp(cmd, "heater") == 0 || strcmp(cmd, "m3") == 0)
    {
        if (strcmp(value, "on") == 0)
        {
            g_heater_on = 1;
            Relay_Heater(RELAY_ON);
            AppTcp_Ack(commandId, "heater", "on", "ok", "");
        }
        else if (strcmp(value, "off") == 0)
        {
            g_heater_on = 0;
            Relay_Heater(RELAY_OFF);
            AppTcp_Ack(commandId, "heater", "off", "ok", "");
        }
        else
        {
            AppTcp_Ack(commandId, "heater", value, "err", "value must be on/off");
        }
        return;
    }
    if (strcmp(cmd, "all") == 0)
    {
        if (strcmp(value, "on") == 0)
        {
            g_motor_on = g_pump_on = g_heater_on = 1;
            Relay_Motor(RELAY_ON);
            Relay_Pump(RELAY_ON);
            Relay_Heater(RELAY_ON);
            AppTcp_Ack(commandId, "all", "on", "ok", "");
        }
        else if (strcmp(value, "off") == 0)
        {
            g_motor_on = g_pump_on = g_heater_on = 0;
            Relay_Motor(RELAY_OFF);
            Relay_Pump(RELAY_OFF);
            Relay_Heater(RELAY_OFF);
            AppTcp_Ack(commandId, "all", "off", "ok", "");
        }
        else
        {
            AppTcp_Ack(commandId, "all", value, "err", "value must be on/off");
        }
        return;
    }

    /* ---- 未知命令 ---- */
    AppTcp_Ack(commandId, cmd, value, "err", "unknown cmd");
}

/**
  * @brief  处理报警配置：{"type":"config","temperatureAlarm":{"sensorKey":...,"threshold":...}}
  * @note   存储阈值并回 config_ack；温度越限的告警上报动作协议未定义，后续再加。
  */
static void AppTcp_HandleConfig(const char *json, const char *commandId)
{
    char  sensorKey[24] = "";
    float threshold = 0.0f;
    int   ok = 0;
    char  th[16] = "";
    char  line[192];

    if (JsonGetString(json, "sensorKey", sensorKey, sizeof(sensorKey)) &&
        JsonGetFloat(json, "threshold", &threshold))
    {
        g_alarm_configured = 1;
        g_alarm_threshold  = threshold;

        AppTcp_StrLower(sensorKey);
        if (strcmp(sensorKey, "temperature_2") == 0)
        {
            g_alarm_sensor = 1;   /* 下水槽 temp_dn */
        }
        else
        {
            g_alarm_sensor = 0;   /* 上水槽 temp_up（默认 temperature_1） */
        }

        ok = 1;
        AppTcp_FmtFloat(th, threshold, 1);
        printf("[CONFIG] alarm sensor=%s threshold=%s\r\n", sensorKey, th);
    }

    sprintf(line, "{\"type\":\"config_ack\",\"deviceId\":\"%s\",\"commandId\":\"%s\",\"result\":\"%s\"}\r\n",
            APP_TCP_DEVICE_ID, commandId, ok ? "ok" : "err");
    AppTcp_SendBuf(line);
}

/**
  * @brief  由超温量映射蜂鸣周期：越限越多，周期越短（响得越快）
  * @param  overshoot: 超出阈值的温度（℃，可为负）
  * @retval 蜂鸣周期（ms）；0 = 未越限，静音
  */
static uint32_t AppTcp_AlarmBeepPeriod(float overshoot)
{
    if (overshoot <= 0.0f) return 0;      /* 未越限 */
    if (overshoot < 2.0f)  return 1000;   /* 越限 <2℃：1s 一声 */
    if (overshoot < 5.0f)  return 500;    /* 2~5℃：2Hz */
    if (overshoot < 10.0f) return 250;    /* 5~10℃：4Hz */
    return 120;                            /* >=10℃：~8Hz 快速 */
}

/* ================= 上报 ================= */

/**
  * @brief  采集一次传感器并以 JSON 上报
  */
static void AppTcp_ReportData(void)
{
    float   t_up = 0.0f, t_dn = 0.0f, pressure = 0.0f, fr, vol;
    uint8_t t1_ok, t2_ok, p_ok;
    char    line[256];
    int     off = 0;

    /* 非阻塞：直接读状态机后台刷新的温度缓存，不再阻塞 LwIP */
    t_up  = DS18B20_Dual_GetTemp(DS18B20_DUAL_UPPER);
    t_dn  = DS18B20_Dual_GetTemp(DS18B20_DUAL_LOWER);
    t1_ok = DS18B20_Dual_GetStatus(DS18B20_DUAL_UPPER);
    t2_ok = DS18B20_Dual_GetStatus(DS18B20_DUAL_LOWER);
    p_ok  = PressureSensor_ReadKpa(&pressure);
    fr    = FlowMeter_GetFlowRate();
    vol   = FlowMeter_GetTotalVolume();

    /* 温度报警：收到 config 后，越限越多蜂鸣越快（周期由超温量决定） */
    if (g_alarm_configured)
    {
        float   alarm_temp;
        uint8_t alarm_ok;

        if (g_alarm_sensor == 1) { alarm_ok = t2_ok; alarm_temp = t_dn; }   /* temperature_2 下水槽 */
        else                     { alarm_ok = t1_ok; alarm_temp = t_up; }   /* temperature_1 上水槽 */

        if (alarm_ok == 0)
        {
            Buzzer_SetBeepPeriod(AppTcp_AlarmBeepPeriod(alarm_temp - g_alarm_threshold));
        }
        else 
        {
            Buzzer_SetBeepPeriod(0);   /* 读取失败，静音 */
        }
    }
    else
    {
        Buzzer_SetBeepPeriod(0);
    }

    off += sprintf(line + off, "{\"type\":\"data\",\"deviceId\":\"%s\",\"seq\":%lu,\"uptime_s\":%lu",
                   APP_TCP_DEVICE_ID,
                   (unsigned long)(++g_data_seq),
                   (unsigned long)(HAL_GetTick() / 1000));

    if (t1_ok == 0) { off += sprintf(line + off, ",\"temp_up\":"); off += AppTcp_FmtFloat(line + off, t_up, 2); }
    else            { off += sprintf(line + off, ",\"temp_up\":null"); }

    if (t2_ok == 0) { off += sprintf(line + off, ",\"temp_dn\":"); off += AppTcp_FmtFloat(line + off, t_dn, 2); }
    else            { off += sprintf(line + off, ",\"temp_dn\":null"); }

    if (p_ok  == 0) { off += sprintf(line + off, ",\"pressure_kpa\":"); off += AppTcp_FmtFloat(line + off, pressure, 2); }
    else            { off += sprintf(line + off, ",\"pressure_kpa\":null"); }

    off += sprintf(line + off, ",\"flow_rate\":");
    off += AppTcp_FmtFloat(line + off, fr, 2);

    off += sprintf(line + off, ",\"flow_volume\":");
    off += AppTcp_FmtFloat(line + off, vol, 3);

    off += sprintf(line + off, ",\"motor\":%u,\"pump\":%u,\"heater\":%u}\r\n",
                   g_motor_on, g_pump_on, g_heater_on);

    AppTcp_SendBuf(line);
}

/**
  * @brief  上报当前继电器状态
  */
static void AppTcp_ReportStatus(void)
{
    char line[96];

    sprintf(line, "{\"type\":\"status\",\"deviceId\":\"%s\",\"motor\":%u,\"pump\":%u,\"heater\":%u}\r\n",
            APP_TCP_DEVICE_ID, g_motor_on, g_pump_on, g_heater_on);
    AppTcp_SendBuf(line);
}

/**
  * @brief  命令应答
  */
static void AppTcp_Ack(const char *commandId, const char *cmd, const char *value, const char *result, const char *msg)
{
    char       line[256];
    const char *p;
    int        off = 0;

    off += sprintf(line + off, "{\"type\":\"ack\",\"deviceId\":\"%s\",\"commandId\":\"%s\",\"cmd\":\"%s\",\"value\":\"%s\",\"result\":\"%s\"",
                   APP_TCP_DEVICE_ID, commandId, cmd, value, result);

    if (msg[0] != '\0')
    {
        off += sprintf(line + off, ",\"msg\":\"");

        p = msg;
        while (*p != '\0' && off < (int)sizeof(line) - 6)
        {
            if (*p == '"' || *p == '\\')
            {
                line[off++] = '\\';
            }
            else if (*p == '\r')
            {
                line[off++] = '\\';
                line[off++] = 'r';
                p++;
                continue;
            }
            else if (*p == '\n')
            {
                line[off++] = '\\';
                line[off++] = 'n';
                p++;
                continue;
            }

            line[off++] = *p++;
        }

        off += sprintf(line + off, "\"");
    }

    off += sprintf(line + off, "}\r\n");

    AppTcp_SendBuf(line);
}

/* ================= JSON 解析辅助 ================= */

static const char *JsonSkipWs(const char *p)
{
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
    {
        p++;
    }
    return p;
}

/**
  * @brief  在 JSON 串里找 "key":"value"，提取 value 字符串
  * @retval 1=成功；0=没找到或格式不符
  * @note   仅支持字符串值（本协议 cmd/value 均为字符串），容忍空白和字段顺序
  */
static int JsonGetString(const char *json, const char *key, char *out, int maxlen)
{
    char        pat[32];
    const char *p, *start, *end;
    int         len;

    sprintf(pat, "\"%s\"", key);
    p = strstr(json, pat);
    if (p == NULL)
    {
        return 0;
    }

    p = strchr(p, ':');
    if (p == NULL)
    {
        return 0;
    }
    p = JsonSkipWs(p + 1);

    if (*p != '"')
    {
        return 0;
    }
    start = p + 1;
    end = strchr(start, '"');
    if (end == NULL)
    {
        return 0;
    }

    len = (int)(end - start);
    if (len >= maxlen)
    {
        len = maxlen - 1;
    }
    memcpy(out, start, len);
    out[len] = '\0';
    return 1;
}

/**
  * @brief  在 JSON 串里找 "key":<整数>，提取整数值
  * @retval 1=成功；0=没找到或格式不符
  */
static int JsonGetInt(const char *json, const char *key, int *out)
{
    char        pat[32];
    const char *p;
    int         sign = 1, val = 0;

    sprintf(pat, "\"%s\"", key);
    p = strstr(json, pat);
    if (p == NULL)
    {
        return 0;
    }
    p = strchr(p, ':');
    if (p == NULL)
    {
        return 0;
    }
    p = JsonSkipWs(p + 1);

    if (*p == '-')
    {
        sign = -1;
        p++;
    }
    if (*p < '0' || *p > '9')
    {
        return 0;
    }
    while (*p >= '0' && *p <= '9')
    {
        val = val * 10 + (*p - '0');
        p++;
    }
    *out = sign * val;
    return 1;
}

/**
  * @brief  在 JSON 串里找 "key":<数字>，提取浮点数值（支持整数/小数/正负号）
  * @retval 1=成功；0=没找到或格式不符
  * @note   兼容 microlib（不用 atof/sscanf %f），如 "threshold":40.5 / 40 / -3.5
  */
static int JsonGetFloat(const char *json, const char *key, float *out)
{
    char        pat[32];
    const char *p;
    float       sign = 1.0f, val = 0.0f, frac = 0.0f, scale = 0.1f;

    sprintf(pat, "\"%s\"", key);
    p = strstr(json, pat);
    if (p == NULL)
    {
        return 0;
    }
    p = strchr(p, ':');
    if (p == NULL)
    {
        return 0;
    }
    p = JsonSkipWs(p + 1);

    if (*p == '-')
    {
        sign = -1.0f;
        p++;
    }
    else if (*p == '+')
    {
        p++;
    }

    if (*p < '0' || *p > '9')
    {
        return 0;   /* 至少一位整数 */
    }
    while (*p >= '0' && *p <= '9')
    {
        val = val * 10.0f + (float)(*p - '0');
        p++;
    }
    if (*p == '.')
    {
        p++;
        while (*p >= '0' && *p <= '9')
        {
            frac += (float)(*p - '0') * scale;
            scale *= 0.1f;
            p++;
        }
    }

    *out = sign * (val + frac);
    return 1;
}

/* ================= 数值格式化 ================= */

/**
  * @brief  把浮点数格式化成字符串（不用 %f，兼容 microlib）
  * @retval 写入的字符数
  */
static int AppTcp_FmtFloat(char *dst, float v, int decimals)
{
    int   ip, fp, div = 1, i, n = 0;
    float av = v;

    if (av < 0.0f)
    {
        dst[n++] = '-';
        av = -av;
    }

    for (i = 0; i < decimals; i++)
    {
        div *= 10;
    }

    ip = (int)av;
    fp = (int)((av - (float)ip) * (float)div + 0.5f);
    if (fp >= div)                 /* 四舍五入进位 */
    {
        fp -= div;
        ip += 1;
    }

    n += sprintf(dst + n, "%d", ip);
    if (decimals > 0)
    {
        dst[n++] = '.';
        if (decimals == 1)      n += sprintf(dst + n, "%d", fp);
        else if (decimals == 2) n += sprintf(dst + n, "%02d", fp);
        else                    n += sprintf(dst + n, "%03d", fp);
    }
    dst[n] = '\0';
    return n;
}

static void AppTcp_StrLower(char *s)
{
    while (*s)
    {
        if (*s >= 'A' && *s <= 'Z')
        {
            *s += 32;
        }
        s++;
    }
}
