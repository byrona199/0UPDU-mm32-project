/**
 * @file Thread.c
 * @brief RTOS threads for MM32 PDU node.
 *
 * Node liveness is entirely poll-driven: the Linux master (can-meter)
 * sends POLL_REQ every ~5 s. This node responds with a fixed burst:
 *   POWER_METRICS → OUTLET_STATE → OUTLET_METRICS
 * No heartbeat is sent. The master detects failure by counting
 * consecutive poll timeouts (miss ≥ 2 → TIMEOUT, miss ≥ 4 → BUS_OFF).
 *
 * Thread architecture:
 * - Thread_CAN_RX:           Receive POLL_REQ / RELAY_CMD / CONNECT_ACK / WHO_IS_ONLINE
 * - Thread_CAN_Connect:     Send CONNECT_REQ on boot (30s) and heartbeat (5 min)
 * - Thread_Metering:         ADC sampling → update shared metrics snapshot
 * - Thread_UART_outlet_stat: UART meter communication (reserved)
 */

#include "uart_txrx_interrupt.h"
#include "can.h"
#include <stdio.h>
#include <stdarg.h>

/*============================================================================
 * UART2 Debug Log (temporary)
 *============================================================================*/
static char dbg_buf[256];

static void dbg_log(const char *fmt, ...)
{
    va_list ap;
    int len;
    va_start(ap, fmt);
    len = vsnprintf(dbg_buf, sizeof(dbg_buf), fmt, ap);
    va_end(ap);
    if (len > 0) {
        if (len >= (int)sizeof(dbg_buf)) len = sizeof(dbg_buf) - 1;
        UART2_Send_Group((u8 *)dbg_buf, (u16)len);
    }
}

/*============================================================================
 * Configuration
 *============================================================================*/
#define MAX_OUTLET          48

/*============================================================================
 * Thread Declarations
 *============================================================================*/
void Thread_CAN_RX(void const *argument);
void Thread_CAN_Connect(void const *argument);
void Thread_Metering(void const *argument);
void Thread_UART_outlet_stat(void const *argument);

osThreadId tid_CAN_RX;
osThreadId tid_CAN_Connect;
osThreadId tid_Metering;
osThreadId tid_UART_outlet_stat;

osThreadDef(Thread_CAN_RX,          osPriorityAboveNormal, 1, 1536);
osThreadDef(Thread_CAN_Connect,     osPriorityNormal,      1, 512);
osThreadDef(Thread_Metering,        osPriorityNormal,      1, 1024);
osThreadDef(Thread_UART_outlet_stat, osPriorityNormal,     1, 1280);

/*============================================================================
 * Synchronization
 *============================================================================*/
osSemaphoreId uart_sem;
osSemaphoreDef(uart_sem);
osMutexId uart_mutex;
osMutexDef(uart_mutex);
osMutexId can_mutex;
osMutexDef(can_mutex);
osMutexId data_mutex;
osMutexDef(data_mutex);

/*============================================================================
 * Connection State (managed by Thread_CAN_Connect, flags set by Thread_CAN_RX)
 *============================================================================*/
typedef enum {
    CONN_DISCONNECTED = 0,  /* 未收到 ACK，每 30s 發送 CONNECT_REQ */
    CONN_CONNECTED          /* 已收到 ACK，每 5 分鐘心跳 */
} conn_state_t;

static volatile conn_state_t g_conn_state        = CONN_DISCONNECTED;
static volatile uint8_t      g_who_is_online_flag = 0; /* WHO_IS_ONLINE 收到時由 CAN_RX 設 1 */
static volatile uint8_t      g_ack_received_flag  = 0; /* CONNECT_ACK 收到時由 CAN_RX 設 1 */

extern u8 sSendBuf[SENDBUFLENGTH];
extern u8 sRecvBuf[RECVBUFLENGTH];
extern u16 RecvLen;
extern u16 SendLen;

/*============================================================================
 * Shared Metering Data (protected by data_mutex)
 *
 * All values stored in CAN fixed-point format.
 * Thread_Metering writes; Thread_CAN_RX reads on POLL_REQ.
 *============================================================================*/

/** @brief Per-phase input power metrics */
static can_metrics_t phase_metrics[CAN_TOTAL_PHASES];  /* [0]=total, [1]=L1, [2]=L2, [3]=L3 */
static uint16_t      frequency_fp;                     /* ×0.01Hz */
static uint8_t       phase_type;                       /* 0=single, 1=three-phase */

/** @brief Per-outlet metrics */
static can_metrics_t outlet_metrics[MAX_OUTLET];

/** @brief Outlet states: 1=ON, 0=OFF */
static uint8_t outlet_state[MAX_OUTLET];

/** @brief Outlet phase mapping: 0=L1, 1=L2, 2=L3 */
static uint8_t outlet_phase[MAX_OUTLET];

/** @brief Relay hardware state (actual GPIO) */
static uint8_t relay_hw_state[MAX_OUTLET];

/*============================================================================
 * Relay GPIO Control
 *============================================================================*/

/** @brief Map outlet ID to GPIO pin. Placeholder — must match PCB routing. */
static void relay_set(uint8_t outlet_id, uint8_t on)
{
    if (outlet_id >= MAX_OUTLET) return;

    /* TODO: Map outlet_id to actual GPIO port/pin per PCB layout.
     * Current board has 12 relays wired to specific pins (see main.c).
     * For full 48-outlet, this will be expanded with shift registers or I2C expanders. */
    relay_hw_state[outlet_id] = on ? 1 : 0;
    outlet_state[outlet_id] = on ? 1 : 0;
}

/*============================================================================
 * Send CONNECT_REQ to master
 *
 * 用於上電公告與心跳。can-meter 收到後回覆 CONNECT_ACK。
 *============================================================================*/
static void send_connect_req(void)
{
    can_connect_req_t req;
    memset(&req, 0, sizeof(req));
    req.firmware_version = 1;  /* TODO: replace with actual firmware version */

    osMutexWait(can_mutex, osWaitForever);
    CAN_SendSingleFrame(CAN_MSG_CONNECT_REQ, (const uint8_t *)&req, sizeof(req));
    osMutexRelease(can_mutex);

    dbg_log("[CONNECT] CONNECT_REQ sent (node=%u state=%s)\r\n",
            MY_NODE_ID,
            (g_conn_state == CONN_CONNECTED) ? "CONNECTED" : "DISCONNECTED");
}

/*============================================================================
 * Init_Thread: create all RTOS threads
 *============================================================================*/
int Init_Thread(void)
{
    uint8_t i;

    uart_sem  = osSemaphoreCreate(osSemaphore(uart_sem), 1);
    uart_mutex = osMutexCreate(osMutex(uart_mutex));
    can_mutex  = osMutexCreate(osMutex(can_mutex));
    data_mutex = osMutexCreate(osMutex(data_mutex));

    /* Initialize default outlet phase mapping (all L1) */
    for (i = 0; i < MAX_OUTLET; i++) {
        outlet_phase[i] = 0;  /* L1 */
    }

    tid_UART_outlet_stat = osThreadCreate(osThread(Thread_UART_outlet_stat), NULL);
    if (!tid_UART_outlet_stat) return -1;

    tid_Metering = osThreadCreate(osThread(Thread_Metering), NULL);
    if (!tid_Metering) return -1;

    tid_CAN_RX = osThreadCreate(osThread(Thread_CAN_RX), NULL);
    if (!tid_CAN_RX) return -1;

    tid_CAN_Connect = osThreadCreate(osThread(Thread_CAN_Connect), NULL);
    if (!tid_CAN_Connect) return -1;

    return 0;
}

/*============================================================================
 * Build and send POWER_METRICS response (multi-frame, 47 bytes)
 *============================================================================*/
static void send_power_metrics(void)
{
    can_power_payload_t pwr;

    osMutexWait(data_mutex, osWaitForever);
    pwr.frequency  = frequency_fp;
    pwr.phase_type = phase_type;
    pwr.total      = phase_metrics[0];
    pwr.phases[0]  = phase_metrics[1];
    pwr.phases[1]  = phase_metrics[2];
    pwr.phases[2]  = phase_metrics[3];
    osMutexRelease(data_mutex);

    osMutexWait(can_mutex, osWaitForever);
    CAN_SendMultiFrame(CAN_MSG_POWER_METRICS, (const uint8_t *)&pwr, sizeof(pwr));
    osMutexRelease(can_mutex);
}

/*============================================================================
 * Build and send OUTLET_STATE response (multi-frame, 18 bytes)
 *
 * Bit packing: 3 bits per outlet (state[0] + phase[2:1])
 *============================================================================*/
static void send_outlet_state(void)
{
    uint8_t buf[CAN_OUTLET_STATE_DATA_SIZE];
    uint16_t bit_offset = 0;
    uint8_t i;

    memset(buf, 0, sizeof(buf));

    osMutexWait(data_mutex, osWaitForever);
    for (i = 0; i < MAX_OUTLET; i++) {
        uint8_t packed = CAN_OUTLET_PACK(outlet_state[i], outlet_phase[i]);
        uint16_t byte_idx = bit_offset / 8;
        uint8_t  bit_pos  = bit_offset % 8;

        /* Pack 3 bits, may span 2 bytes */
        buf[byte_idx] |= (packed << bit_pos) & 0xFF;
        if (bit_pos > 5) {
            buf[byte_idx + 1] |= packed >> (8 - bit_pos);
        }
        bit_offset += 3;
    }
    osMutexRelease(data_mutex);

    osMutexWait(can_mutex, osWaitForever);
    CAN_SendMultiFrame(CAN_MSG_OUTLET_STATE, buf, sizeof(buf));
    osMutexRelease(can_mutex);
}

/*============================================================================
 * Build and send OUTLET_METRICS response (multi-frame, 528 bytes)
 *============================================================================*/
static void send_outlet_metrics(void)
{
    uint8_t buf[CAN_OUTLET_METRICS_DATA_SIZE];

    osMutexWait(data_mutex, osWaitForever);
    memcpy(buf, outlet_metrics, sizeof(buf));
    osMutexRelease(data_mutex);

    osMutexWait(can_mutex, osWaitForever);
    CAN_SendMultiFrame(CAN_MSG_OUTLET_METRICS, buf, sizeof(buf));
    osMutexRelease(can_mutex);
}

/*============================================================================
 * Handle RELAY_CMD from master
 *============================================================================*/
static void handle_relay_cmd(const uint8_t *data, uint8_t len)
{
    can_relay_cmd_t cmd;
    can_relay_ack_t ack;
    uint8_t i;

    if (len < sizeof(can_relay_cmd_t)) return;
    memcpy(&cmd, data, sizeof(cmd));

    ack.command   = cmd.command;
    ack.outlet_id = cmd.outlet_id;
    ack.result    = CAN_RELAY_ACK_OK;
    memset(ack.reserved, 0, sizeof(ack.reserved));

    switch (cmd.command) {
        case CAN_RELAY_CMD_OFF:
            if (cmd.outlet_id < MAX_OUTLET) {
                relay_set(cmd.outlet_id, 0);
            } else {
                ack.result = CAN_RELAY_ACK_FAIL;
            }
            break;

        case CAN_RELAY_CMD_ON:
            if (cmd.outlet_id < MAX_OUTLET) {
                if (cmd.delay > 0) {
                    /* Delayed ON: delay is in units of 100ms */
                    osDelay(cmd.delay * (CAN_SCALE_DELAY));
                }
                relay_set(cmd.outlet_id, 1);
            } else {
                ack.result = CAN_RELAY_ACK_FAIL;
            }
            break;

        case CAN_RELAY_CMD_CYCLE:
            if (cmd.outlet_id < MAX_OUTLET) {
                relay_set(cmd.outlet_id, 0);
                if (cmd.delay > 0) {
                    osDelay(cmd.delay * (CAN_SCALE_DELAY));
                } else {
                    osDelay(1000);  /* default 1s off */
                }
                relay_set(cmd.outlet_id, 1);
            } else {
                ack.result = CAN_RELAY_ACK_FAIL;
            }
            break;

        case CAN_RELAY_CMD_ALL_OFF:
            for (i = 0; i < MAX_OUTLET; i++) {
                relay_set(i, 0);
            }
            break;

        case CAN_RELAY_CMD_ALL_ON:
            for (i = 0; i < MAX_OUTLET; i++) {
                relay_set(i, 1);
            }
            break;

        default:
            ack.result = CAN_RELAY_ACK_FAIL;
            break;
    }

    /* Send RELAY_ACK */
    osMutexWait(can_mutex, osWaitForever);
    CAN_SendSingleFrame(CAN_MSG_RELAY_ACK, (const uint8_t *)&ack, sizeof(ack));
    osMutexRelease(can_mutex);
}

/*============================================================================
 * Thread_CAN_RX: Process incoming CAN messages
 *
 * Polls the flag set by CAN_IRQHandler. When a frame is received,
 * checks MSG_TYPE and dispatches accordingly.
 *============================================================================*/
void Thread_CAN_RX(void const *argument)
{
    (void)argument;

    dbg_log("[CAN_RX] Thread started, NODE_ID=%u\r\n", MY_NODE_ID);

    for (;;) {
        if (flag) {
            flag = 0;

            /* CAN_Peli_Receive() already extracts the 11-bit Standard ID
             * into gCanPeliRxMsgBuff.ID — use it directly. */
            uint16_t can_id = (uint16_t)gCanPeliRxMsgBuff.ID;
            uint8_t msg_type = CAN_GET_MSG_TYPE(can_id);
            uint8_t node_id  = CAN_GET_NODE_ID(can_id);

            dbg_log("[CAN_RX] Frame: can_id=0x%03X msg=%u node=%u DLC=%u\r\n",
                    can_id, msg_type, node_id, gCanPeliRxMsgBuff.DLC);

            /* Only process frames addressed to us or broadcast */
            if (node_id != MY_NODE_ID && node_id != CAN_NODE_ID_BROADCAST) {
                dbg_log("[CAN_RX] Ignored (not for us)\r\n");
                continue;
            }

            /* Extract payload (skip transport header byte) */
            uint8_t *payload = &gCanPeliRxMsgBuff.Data[1];
            uint8_t  payload_len = gCanPeliRxMsgBuff.DLC > 1 ?
                                   gCanPeliRxMsgBuff.DLC - 1 : 0;

            switch (msg_type) {
                case CAN_MSG_CONNECT_ACK:
                    /* Master 確認連線：通知 Thread_CAN_Connect 切換心跳模式 */
                    dbg_log("[CAN_RX] CONNECT_ACK received\r\n");
                    g_ack_received_flag = 1;
                    break;

                case CAN_MSG_WHO_IS_ONLINE:
                    /* Master 廣播發現（node_id=0）：立即觸發送出 CONNECT_REQ */
                    dbg_log("[CAN_RX] WHO_IS_ONLINE received, triggering CONNECT_REQ\r\n");
                    g_who_is_online_flag = 1;
                    break;

                case CAN_MSG_POLL_REQ:
                    dbg_log("[CAN_RX] POLL_REQ -> sending burst\r\n");
                    send_power_metrics();
                    dbg_log("[CAN_RX]   power_metrics sent\r\n");
                    send_outlet_state();
                    dbg_log("[CAN_RX]   outlet_state sent\r\n");
                    send_outlet_metrics();
                    dbg_log("[CAN_RX]   outlet_metrics sent\r\n");
                    break;

                case CAN_MSG_RELAY_CMD:
                    dbg_log("[CAN_RX] RELAY_CMD len=%u\r\n", payload_len);
                    handle_relay_cmd(payload, payload_len);
                    dbg_log("[CAN_RX]   relay_cmd handled\r\n");
                    break;

                case CAN_MSG_CONFIG_WRITE:
                    dbg_log("[CAN_RX] CONFIG_WRITE (TODO)\r\n");
                    break;

                default:
                    dbg_log("[CAN_RX] Unknown msg_type=%u\r\n", msg_type);
                    break;
            }
        }
        osDelay(1);  /* Yield to other threads */
    }
}

/*============================================================================
 * Thread_Metering: ADC sampling → update metrics snapshot
 *
 * This thread reads the ADC/meter IC data and updates the shared
 * metrics arrays. Currently uses placeholder values for testing.
 *============================================================================*/
void Thread_Metering(void const *argument)
{
    (void)argument;
    uint8_t i;
    static uint32_t energy_acc = 0;  /* simulated energy accumulator */

    dbg_log("[METER] Thread started, filling fake data\r\n");

    for (;;) {
        energy_acc += 10;  /* +0.10 kWh per cycle */

        osMutexWait(data_mutex, osWaitForever);

        /*--- Fake input power metrics ---*/
        frequency_fp = 6000;   /* 60.00 Hz */
        phase_type   = 1;      /* three-phase */

        /* Total */
        phase_metrics[0].current = 1520;   /* 15.20 A */
        phase_metrics[0].voltage = 2200;   /* 220.0 V */
        phase_metrics[0].power   = 3344;   /* 3344 W  */
        phase_metrics[0].pf      = 98;     /* 0.98    */
        phase_metrics[0].energy  = energy_acc;

        /* L1 */
        phase_metrics[1].current = 520;    /* 5.20 A  */
        phase_metrics[1].voltage = 2201;   /* 220.1 V */
        phase_metrics[1].power   = 1144;   /* 1144 W  */
        phase_metrics[1].pf      = 99;
        phase_metrics[1].energy  = energy_acc / 3;

        /* L2 */
        phase_metrics[2].current = 480;    /* 4.80 A  */
        phase_metrics[2].voltage = 2198;   /* 219.8 V */
        phase_metrics[2].power   = 1055;   /* 1055 W  */
        phase_metrics[2].pf      = 97;
        phase_metrics[2].energy  = energy_acc / 3;

        /* L3 */
        phase_metrics[3].current = 510;    /* 5.10 A  */
        phase_metrics[3].voltage = 2205;   /* 220.5 V */
        phase_metrics[3].power   = 1125;   /* 1125 W  */
        phase_metrics[3].pf      = 99;
        phase_metrics[3].energy  = energy_acc / 3;

        /*--- Fake per-outlet metrics (first 12 outlets active) ---*/
        for (i = 0; i < MAX_OUTLET; i++) {
            if (i < 12) {
                outlet_metrics[i].current = 100 + i * 20;   /* 1.00-1.22 A */
                outlet_metrics[i].voltage = 2200;
                outlet_metrics[i].power   = 220 + i * 5;
                outlet_metrics[i].pf      = 95 + (i % 6);
                outlet_metrics[i].energy  = energy_acc / 12;
                outlet_state[i] = 1;   /* ON */
                outlet_phase[i] = i % 3;  /* round-robin L1/L2/L3 */
            } else {
                memset(&outlet_metrics[i], 0, sizeof(can_metrics_t));
                outlet_state[i] = 0;   /* OFF */
                outlet_phase[i] = 0;
            }
        }

        osMutexRelease(data_mutex);

        osDelay(100);  /* 10 Hz sampling rate */
    }
}

/*============================================================================
 * Thread_UART_outlet_stat: UART meter board communication (reserved)
 *============================================================================*/
void Thread_UART_outlet_stat(void const *argument)
{
    (void)argument;

    for (;;) {
        /* TODO: UART communication with external meter sub-boards */
        osDelay(500);
    }
}

/*============================================================================
 * Thread_CAN_Connect: 管理 CONNECT_REQ 發送時機
 *
 * 未連線狀態：啟動後立即發送，此後每 30s 發送一次直到收到 ACK。
 * 已連線狀態：每 5 分鐘發送一次心跳。
 * WHO_IS_ONLINE 收到時不論狀態都立即發送 CONNECT_REQ。
 *============================================================================*/
void Thread_CAN_Connect(void const *argument)
{
    uint32_t interval_ms;
    uint32_t elapsed_ms;
    (void)argument;

    dbg_log("[CONNECT] Thread started, sending initial CONNECT_REQ\r\n");

    /* 啟動後立即發送 */
    send_connect_req();

    for (;;) {
        elapsed_ms  = 0;
        interval_ms = (g_conn_state == CONN_CONNECTED)
                      ? ((uint32_t)CAN_HEARTBEAT_INTERVAL_S * 1000u)  /* 300s */
                      : ((uint32_t)CAN_CONNECT_RETRY_S     * 1000u);  /* 30s  */

        /* 以 100ms 為最小察測單位，對強影響可忽略 */
        while (elapsed_ms < interval_ms) {
            osDelay(100);
            elapsed_ms += 100;

            /* CONNECT_ACK 收到：切換心跳模式，重置計時 */
            if (g_ack_received_flag) {
                g_ack_received_flag = 0;
                if (g_conn_state != CONN_CONNECTED) {
                    g_conn_state = CONN_CONNECTED;
                    dbg_log("[CONNECT] ACK received, switching to heartbeat (%us)\r\n",
                            CAN_HEARTBEAT_INTERVAL_S);
                }
                elapsed_ms  = 0;
                interval_ms = (uint32_t)CAN_HEARTBEAT_INTERVAL_S * 1000u;
            }

            /* WHO_IS_ONLINE 收到：不論狀態立即發送 */
            if (g_who_is_online_flag) {
                g_who_is_online_flag = 0;
                dbg_log("[CONNECT] WHO_IS_ONLINE: sending CONNECT_REQ immediately\r\n");
                break;
            }
        }

        send_connect_req();
    }
}
