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
#include "modbus_meter.h"
#include "ht1621.h"
#include "buttons.h"
#include "pdu_role.h"
#include "env_source.h"
#include "dbg_log.h"
#include <stdio.h>
#include <stdarg.h>

/*============================================================================
 * UART1 Debug Log (compile-time switch via dbg_log.h)
 *============================================================================*/
#if ENABLE_DBG_LOG
osMutexId uart_mutex;
osMutexDef(uart_mutex);

void dbg_log(const char *fmt, ...)
{
    /* Static buffer is safe here because uart_mutex below serializes
     * both the formatting and the UART1 transmit. Keeping it static
     * avoids a 256-byte stack hit on every caller thread. */
    static char dbg_buf[256];
    va_list ap;
    int len;

    /* Prevent multi-threaded logs from mixing on UART1,
     * and protect the shared dbg_buf above. */
    if (uart_mutex) {
        osMutexWait(uart_mutex, osWaitForever);
    }

    va_start(ap, fmt);
    len = vsnprintf(dbg_buf, sizeof(dbg_buf), fmt, ap);
    va_end(ap);

    if (len > 0) {
        if (len >= (int)sizeof(dbg_buf)) len = sizeof(dbg_buf) - 1;
        UART1_Send_Group((u8 *)dbg_buf, (u16)len);
    }

    if (uart_mutex) {
        osMutexRelease(uart_mutex);
    }
}
#endif /* ENABLE_DBG_LOG */

/*============================================================================
 * Configuration
 *============================================================================*/
#define MAX_OUTLET          48
#define CONNECT_ACK_MISS_MAX 2

/*============================================================================
 * Thread Declarations
 *============================================================================*/
void Thread_CAN_RX(void const *argument);
void Thread_CAN_Connect(void const *argument);
void Thread_Metering(void const *argument);
void Thread_UART_outlet_stat(void const *argument);
void Thread_Display(void const *argument);

osThreadId tid_CAN_RX;
osThreadId tid_CAN_Connect;
osThreadId tid_Metering;
osThreadId tid_UART_outlet_stat;
osThreadId tid_Display;

osThreadDef(Thread_CAN_RX,          osPriorityAboveNormal, 1, 1280);
osThreadDef(Thread_CAN_Connect,     osPriorityNormal,      1, 256);
osThreadDef(Thread_Metering,        osPriorityNormal,      1, 768);
osThreadDef(Thread_UART_outlet_stat, osPriorityNormal,     1, 1792);
osThreadDef(Thread_Display,          osPriorityLow,        1, 512);

/*============================================================================
 * Synchronization
 *============================================================================*/
osSemaphoreId uart_sem;
osSemaphoreDef(uart_sem);
osMutexId can_mutex;
osMutexDef(can_mutex);
osMutexId data_mutex;
osMutexDef(data_mutex);

/*============================================================================
 * Thread Liveness Heartbeats (for IWDG-driven crash recovery)
 *
 * 每個受監控執行緒只在完成一輪正常工作、沒有卡在鎖/等待時遞增自己的計數器。
 * Thread_Health_OK() 由 main() 週期呼叫；只要有任一計數器超過允許的最大靜止
 * 時間沒有前進，就回傳不健康，main() 停止餵狗，讓 IWDG 硬體逾時重置 MCU。
 *============================================================================*/
#define HEALTH_CHECK_INTERVAL_MS   300u
#define HEALTH_MAX_STALE_CAN_MS    2000u   /* Thread_CAN_RX / Thread_CAN_Connect */
#define HEALTH_MAX_STALE_UART_MS   5000u   /* Thread_UART_outlet_stat（含開機 2s 暖機 + 首次
                                             * Modbus 交易最差約 2s ≈ 4s，僅約 1s 安全餘裕；
                                             * 若調整 MB_MAX_RETRIES/MB_RESPONSE_TIMEOUT_MS/
                                             * MB_BUS_QUIET_MAX_MS 或開機暖機時間，須重新檢查
                                             * 此門檻是否仍足夠，避免開機誤觸發 watchdog 重置 */

static volatile uint32_t g_hb_can_rx      = 0u;
static volatile uint32_t g_hb_can_connect = 0u;
static volatile uint32_t g_hb_uart        = 0u;

/**
 * @brief  由 main() idle loop 週期呼叫（間隔須為 HEALTH_CHECK_INTERVAL_MS）。
 * @retval 1 = 三個執行緒都健康；0 = 至少一個已卡死超過門檻。
 */
int Thread_Health_OK(void)
{
    static uint32_t last_can_rx      = 0xFFFFFFFFu;
    static uint32_t last_can_connect = 0xFFFFFFFFu;
    static uint32_t last_uart        = 0xFFFFFFFFu;
    static uint32_t stale_can_rx_ms      = 0u;
    static uint32_t stale_can_connect_ms = 0u;
    static uint32_t stale_uart_ms        = 0u;

    uint32_t cur_can_rx      = g_hb_can_rx;
    uint32_t cur_can_connect = g_hb_can_connect;
    uint32_t cur_uart        = g_hb_uart;

    if (cur_can_rx != last_can_rx) {
        stale_can_rx_ms = 0u;
        last_can_rx     = cur_can_rx;
    } else {
        stale_can_rx_ms += HEALTH_CHECK_INTERVAL_MS;
    }

    if (cur_can_connect != last_can_connect) {
        stale_can_connect_ms = 0u;
        last_can_connect     = cur_can_connect;
    } else {
        stale_can_connect_ms += HEALTH_CHECK_INTERVAL_MS;
    }

    if (cur_uart != last_uart) {
        stale_uart_ms = 0u;
        last_uart     = cur_uart;
    } else {
        stale_uart_ms += HEALTH_CHECK_INTERVAL_MS;
    }

    if (stale_can_rx_ms      > HEALTH_MAX_STALE_CAN_MS)  return 0;
    if (stale_can_connect_ms > HEALTH_MAX_STALE_CAN_MS)  return 0;
    if (stale_uart_ms        > HEALTH_MAX_STALE_UART_MS) return 0;
    return 1;
}

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

static int16_t  env_temp_x10;   /* ×0.1°C，有號（env_source 提供）*/
static uint16_t env_humi_x10;   /* ×0.1%RH */
static uint8_t  env_valid;      /* CAN_ENV_VALID_* bits，0=無感測器 */

/** @brief Per-outlet metrics */
static can_metrics_t outlet_metrics[MAX_OUTLET];

/** @brief Outlet states: 1=ON, 0=OFF */
static uint8_t outlet_state[MAX_OUTLET];

/** @brief Outlet phase mapping: 0=L1, 1=L2, 2=L3 */
static uint8_t outlet_phase[MAX_OUTLET];

/*============================================================================
 * Relay Command Queue
 *
 * CAN_RX thread pushes commands; UART_outlet_stat thread consumes them
 * and sends Modbus FC06 to the metering board.
 *============================================================================*/
#define RELAY_QUEUE_SIZE  16

typedef struct {
    uint8_t  command;       /* CAN_RELAY_CMD_* */
    uint8_t  outlet_id;     /* 0-47 for single, ignored for ALL_ON/ALL_OFF */
    uint16_t delay;         /* delay in units of 100ms */
} relay_queue_item_t;

static relay_queue_item_t relay_queue[RELAY_QUEUE_SIZE];
static volatile uint8_t   relay_q_head = 0;
static volatile uint8_t   relay_q_tail = 0;
static osMutexId           relay_q_mutex;
osMutexDef(relay_q_mutex);

/** @brief Push a relay command (called from CAN_RX thread). Returns 0 on success. */
static int relay_queue_push(uint8_t command, uint8_t outlet_id, uint16_t delay)
{
    uint8_t next;
    int ret = -1;
    osMutexWait(relay_q_mutex, osWaitForever);
    next = (relay_q_head + 1) % RELAY_QUEUE_SIZE;
    if (next != relay_q_tail) {
        relay_queue[relay_q_head].command   = command;
        relay_queue[relay_q_head].outlet_id = outlet_id;
        relay_queue[relay_q_head].delay     = delay;
        relay_q_head = next;
        ret = 0;
    }
    osMutexRelease(relay_q_mutex);
    return ret;
}

/** @brief Pop a relay command (called from UART_outlet_stat thread). Returns 0 on success. */
static int relay_queue_pop(relay_queue_item_t *item)
{
    int ret = -1;
    osMutexWait(relay_q_mutex, osWaitForever);
    if (relay_q_head != relay_q_tail) {
        *item = relay_queue[relay_q_tail];
        relay_q_tail = (relay_q_tail + 1) % RELAY_QUEUE_SIZE;
        ret = 0;
    }
    osMutexRelease(relay_q_mutex);
    return ret;
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
#if ENABLE_DBG_LOG
    uart_mutex = osMutexCreate(osMutex(uart_mutex));
#endif
    can_mutex  = osMutexCreate(osMutex(can_mutex));
    data_mutex = osMutexCreate(osMutex(data_mutex));
    relay_q_mutex = osMutexCreate(osMutex(relay_q_mutex));

    /* Initialize default outlet phase mapping (all L1) */
    for (i = 0; i < MAX_OUTLET; i++) {
        outlet_phase[i] = 0;  /* L1 */
    }

    /* CAN / modbus / metering threads: only when a valid role is configured.
     * role=0 (unconfigured) runs Thread_Display only; user must commission
     * via the button UI before CAN is active. */
    if (g_my_node_id != 0u) {
        tid_UART_outlet_stat = osThreadCreate(osThread(Thread_UART_outlet_stat), NULL);
        if (!tid_UART_outlet_stat) return -1;

        tid_Metering = osThreadCreate(osThread(Thread_Metering), NULL);
        if (!tid_Metering) return -1;

        tid_CAN_RX = osThreadCreate(osThread(Thread_CAN_RX), NULL);
        if (!tid_CAN_RX) return -1;

        tid_CAN_Connect = osThreadCreate(osThread(Thread_CAN_Connect), NULL);
        if (!tid_CAN_Connect) return -1;
    }

    tid_Display = osThreadCreate(osThread(Thread_Display), NULL);
    if (!tid_Display) return -1;

    return 0;
}

/*============================================================================
 * Build full burst snapshot (called before sending, under data_mutex)
 *
 * Snapshots all shared data into local buffers so the can_mutex hold
 * during the burst does not block Thread_Metering / Thread_UART_outlet_stat.
 *============================================================================*/
static can_power_payload_t  can_power_payload_burst;
static uint8_t              outlet_state_burst[CAN_OUTLET_STATE_DATA_SIZE];
static uint8_t              outlet_metrics_burst[CAN_OUTLET_METRICS_DATA_SIZE];

static void prepare_burst_snapshot(void)
{
    uint16_t bit_offset = 0;
    uint8_t i;

    osMutexWait(data_mutex, osWaitForever);

    /* Power payload */
    can_power_payload_burst.frequency  = frequency_fp;
    can_power_payload_burst.phase_type = phase_type;
    can_power_payload_burst.total      = phase_metrics[0];
    can_power_payload_burst.phases[0]  = phase_metrics[1];
    can_power_payload_burst.phases[1]  = phase_metrics[2];
    can_power_payload_burst.phases[2]  = phase_metrics[3];
    can_power_payload_burst.temperature = env_temp_x10;
    can_power_payload_burst.humidity    = env_humi_x10;
    can_power_payload_burst.env_valid   = env_valid;

    /* Outlet state: 3 bits per outlet, LSB-first */
    memset(outlet_state_burst, 0, sizeof(outlet_state_burst));
    for (i = 0; i < MAX_OUTLET; i++) {
        uint8_t packed    = CAN_OUTLET_PACK(outlet_state[i], outlet_phase[i]);
        uint16_t byte_idx = bit_offset / 8;
        uint8_t  bit_pos  = bit_offset % 8;
        outlet_state_burst[byte_idx] |= (packed << bit_pos) & 0xFF;
        if (bit_pos > 5) {
            outlet_state_burst[byte_idx + 1] |= packed >> (8 - bit_pos);
        }
        bit_offset += 3;
    }

    /* Outlet metrics */
    memcpy(outlet_metrics_burst, outlet_metrics, sizeof(outlet_metrics_burst));

    osMutexRelease(data_mutex);
}

/*============================================================================
 * Handle RELAY_CMD from master
 *
 * Pushes command to relay queue for async Modbus execution.
 * Sends optimistic RELAY_ACK immediately (does not wait for Modbus response).
 *============================================================================*/
static void handle_relay_cmd(const uint8_t *data, uint8_t len)
{
    can_relay_cmd_t cmd;
    can_relay_ack_t ack;
    uint8_t copy_len;

    /* Thread_CAN_RX already strips the 1-byte transport header.
     * Here len is pure payload length, so compare directly with payload struct size. */
    if (len < sizeof(can_relay_cmd_t)) {
        dbg_log("[RELAY] Short payload len=%u\r\n", len);
        return;
    }

    memset(&cmd, 0, sizeof(cmd));
    copy_len = (len < sizeof(cmd)) ? len : (uint8_t)sizeof(cmd);
    memcpy(&cmd, data, copy_len);

    ack.command   = cmd.command;
    ack.outlet_id = cmd.outlet_id;
    ack.result    = CAN_RELAY_ACK_OK;
    memset(ack.reserved, 0, sizeof(ack.reserved));

    dbg_log("[RELAY] cmd=%u outlet=%u delay=%u\r\n",
            cmd.command, cmd.outlet_id, cmd.delay);

    /* Push to queue; on failure (queue full), report FAIL */
    if (relay_queue_push(cmd.command, cmd.outlet_id, cmd.delay) != 0) {
        dbg_log("[RELAY] Queue full!\r\n");
        ack.result = CAN_RELAY_ACK_FAIL;
    }

    /* Send RELAY_ACK immediately (optimistic) */
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
                    /* Snapshot data first (takes data_mutex, does not hold can_mutex). */
                    prepare_burst_snapshot();
                    /* Hold can_mutex across the full burst so no other thread
                     * (e.g. Thread_CAN_Connect) can insert a frame between
                     * POWER_METRICS, OUTLET_STATE, and OUTLET_METRICS. */
                    dbg_log("[CAN_RX] POLL_REQ -> sending burst\r\n");
                    osMutexWait(can_mutex, osWaitForever);
                    CAN_SendMultiFrame(CAN_MSG_POWER_METRICS,
                                      (const uint8_t *)&can_power_payload_burst,
                                      sizeof(can_power_payload_burst));
                    CAN_SendMultiFrame(CAN_MSG_OUTLET_STATE,
                                      outlet_state_burst,
                                      sizeof(outlet_state_burst));
                    CAN_SendMultiFrame(CAN_MSG_OUTLET_METRICS,
                                      outlet_metrics_burst,
                                      sizeof(outlet_metrics_burst));
                    osMutexRelease(can_mutex);
                    dbg_log("[CAN_RX] burst sent (power+state+metrics)\r\n");
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
        g_hb_can_rx++;
        osDelay(1);  /* Yield to other threads */
    }
}

/*============================================================================
 * Thread_Metering: Environment polling + reserved for future ADC sampling
 *
 * Metering data is now populated by Thread_UART_outlet_stat via Modbus.
 * This thread polls env_source (temperature/humidity) once per second and
 * is kept as a placeholder for local ADC / meter IC reads.
 *============================================================================*/
void Thread_Metering(void const *argument)
{
    (void)argument;

    dbg_log("[METER] Thread started (env polling + placeholder for local ADC)\r\n");

    for (;;) {
        int16_t  t;
        uint16_t h;
        uint8_t  valid = env_source_read(&t, &h);

        osMutexWait(data_mutex, osWaitForever);
        if (valid & CAN_ENV_VALID_TEMP) env_temp_x10 = t;
        if (valid & CAN_ENV_VALID_HUMI) env_humi_x10 = h;
        env_valid = valid;
        osMutexRelease(data_mutex);

        osDelay(1000);
    }
}

/*============================================================================
 * Process pending relay commands from the queue via Modbus
 *============================================================================*/

/**
 * @brief  osDelay() 分成 <=100ms 一格並同時遞增 g_hb_uart，避免 relay 指令
 *         內建的延遲（例如錯開開機的 CAN_RELAY_CMD_ON delay）讓 UART 執行緒
 *         心跳計數器長時間停滯，被 watchdog 誤判為卡死。
 */
static void relay_delay_with_heartbeat(uint32_t delay_ms)
{
    uint32_t waited_ms = 0u;
    while (waited_ms < delay_ms) {
        uint32_t chunk = (delay_ms - waited_ms > 100u) ? 100u : (delay_ms - waited_ms);
        osDelay(chunk);
        waited_ms += chunk;
        g_hb_uart++;
    }
}

static void process_relay_queue(void)
{
    relay_queue_item_t item;
    meter_outlet_map_t map;
    uint8_t i;

    while (relay_queue_pop(&item) == 0) {
        switch (item.command) {
            case CAN_RELAY_CMD_OFF:
                if (item.outlet_id < MAX_OUTLET) {
                    meter_outlet_to_board(item.outlet_id, &map);
                    meter_set_relay(map.slave_id, map.channel, 0);
                    g_hb_uart++;
                }
                break;

            case CAN_RELAY_CMD_ON:
                if (item.outlet_id < MAX_OUTLET) {
                    if (item.delay > 0) {
                        relay_delay_with_heartbeat((uint32_t)item.delay * CAN_SCALE_DELAY);
                    }
                    meter_outlet_to_board(item.outlet_id, &map);
                    meter_set_relay(map.slave_id, map.channel, 1);
                    g_hb_uart++;
                }
                break;

            case CAN_RELAY_CMD_CYCLE:
                if (item.outlet_id < MAX_OUTLET) {
                    meter_outlet_to_board(item.outlet_id, &map);
                    meter_set_relay(map.slave_id, map.channel, 0);
                    g_hb_uart++;
                    if (item.delay > 0) {
                        relay_delay_with_heartbeat((uint32_t)item.delay * CAN_SCALE_DELAY);
                    } else {
                        relay_delay_with_heartbeat(1000u);
                    }
                    meter_set_relay(map.slave_id, map.channel, 1);
                    g_hb_uart++;
                }
                break;

            case CAN_RELAY_CMD_ALL_OFF:
                for (i = 0; i < MAX_OUTLET; i++) {
                    meter_outlet_to_board(i, &map);
                    meter_set_relay(map.slave_id, map.channel, 0);
                    g_hb_uart++;
                }
                break;

            case CAN_RELAY_CMD_ALL_ON:
                for (i = 0; i < MAX_OUTLET; i++) {
                    meter_outlet_to_board(i, &map);
                    meter_set_relay(map.slave_id, map.channel, 1);
                    g_hb_uart++;
                }
                break;

            default:
                dbg_log("[RELAY] Unknown cmd=%u\r\n", item.command);
                break;
        }
    }
}

/*============================================================================
 * Thread_UART_outlet_stat: Modbus RTU polling loop
 *
 * Polls total power meter (slave ID 1, FC03) and each metering board
 * (slave ID 2..13, FC04) via RS-485 Modbus:
 *   1. Process any pending relay commands from the queue
 *   2. Poll total power meter for 3-phase metrics
 *   3. Read all 38 registers (FC04) from each outlet board
 *   4. Convert and store into shared outlet_metrics[]/outlet_state[]
 *   5. Update phase_metrics[] from total meter data
 *============================================================================*/
void Thread_UART_outlet_stat(void const *argument)
{
    (void)argument;
    uint8_t board;
    uint8_t ch;
    uint8_t outlet_id;
    meter_board_t board_data;
    can_metrics_t can_ch;
    int rc;

    /* Total power meter */
    total_meter_data_t total_data;
    int total_rc;

    dbg_log("[MODBUS] Thread started, polling %u boards + total meter\r\n", METER_BOARD_COUNT);

    /* Wait a bit for metering boards to be ready after power-on */
    osDelay(2000);

    for (;;) {
        /* Process relay commands first (high priority) */
        process_relay_queue();

        /* ---- Poll total power meter (slave 1, FC03) ---- */
        total_rc = total_meter_read(&total_data);
        if (total_rc != MB_OK) {
            dbg_log("[MODBUS] Total meter offline\r\n");
        }
        g_hb_uart++;  /* 每次 Modbus 交易嘗試（含離線）都算進度，避免全離線時卡在單一 cycle 計數器誤判 */

        /* ---- Poll each outlet board ---- */
        for (board = 0; board < METER_BOARD_COUNT; board++) {
            uint8_t slave_id = board + METER_SLAVE_ID_BASE;

            /* Check for relay commands between boards */
            process_relay_queue();

            rc = meter_read_all(slave_id, &board_data);
            g_hb_uart++;  /* 每次 Modbus 交易嘗試（含離線）都算進度 */
            if (rc != MB_OK) {
                /* Board offline or error — skip this board */
                dbg_log("[MODBUS] Board %u (slave %u) offline\r\n", board, slave_id);
                continue;
            }

            /* Update shared data for each channel */
            osMutexWait(data_mutex, osWaitForever);

            for (ch = 0; ch < METER_CHANNELS_PER_BOARD; ch++) {
                outlet_id = board * METER_CHANNELS_PER_BOARD + ch;
                if (outlet_id >= MAX_OUTLET) break;

                /* Convert 1084 data to CAN format */
                meter_channel_to_can(&board_data.channels[ch], &can_ch);
                outlet_metrics[outlet_id] = can_ch;

                /* Update outlet state from relay state bits */
                outlet_state[outlet_id] = (board_data.relay_state >> ch) & 0x01;
            }

            osMutexRelease(data_mutex);
        }

        /* ---- Update phase metrics from total meter ---- */
        osMutexWait(data_mutex, osWaitForever);
        if (total_rc == MB_OK) {
            total_meter_to_can(&total_data,
                               &phase_metrics[0],   /* total */
                               &phase_metrics[1]);   /* L1/L2/L3 array */
            frequency_fp = total_data.frequency;
            phase_type = 1;  /* three-phase */
        } else {
            /* No total meter data: publish empty power snapshot and avoid stale values. */
            memset(phase_metrics, 0, sizeof(phase_metrics));
            frequency_fp = 0;
            phase_type = 0;
        }
        osMutexRelease(data_mutex);

        /* Inter-poll delay: remaining time in 10s cycle.
         * Each board takes ~150ms, 12 boards ≈ 1.8s. Sleep the rest. */
        osDelay(500);
    }
}

/*============================================================================
 * Bus-Off 偵測與自動復原（由 Thread_CAN_Connect 的 100ms tick 呼叫）
 *
 * 2026-07-13 實測：CAN 控制器被錯誤風暴打進 bus-off 後韌體無復原邏輯，
 * 執行緒全部存活（watchdog 盲區），只能斷電。此處輪詢 SR 的 BS bit：
 *   BS 置起 → 取 can_mutex 重跑 CAN_NVIC_Init（重建全部組態含驗收濾波器，
 *   結尾清 RM 觸發 128×11 recessive bits 復原）。
 *   軟復原之間退避 5s；連續 5 次後 BS 仍置起 → NVIC_SystemReset() 保底。
 *   復原成功（BS 讀到清除）→ 強制 CONN_DISCONNECTED + who_is_online_flag，
 *   复用既有 jitter 重連路徑，數秒內重新被主機發現。
 *============================================================================*/
#define BUSOFF_BACKOFF_TICKS      50u  /* 5s @ 100ms tick */
#define BUSOFF_SOFT_RECOVER_MAX    5u  /* 連續軟復原失敗上限，超過即整顆重開 */

static void busoff_check_tick(void)
{
    static uint32_t backoff_ticks    = 0u;
    static uint8_t  soft_fail_count  = 0u;
    static uint8_t  recovery_pending = 0u;

    if (backoff_ticks > 0u) {
        backoff_ticks--;
    }

    if (CAN_GetFlagStatus(CAN1, CAN_STATUS_BS) == RESET) {
        /* bus 正常。若剛做過軟復原，視為成功：歸零計數並重新宣告連線 */
        if (recovery_pending) {
            recovery_pending = 0u;
            soft_fail_count  = 0u;
            g_conn_state         = CONN_DISCONNECTED;
            g_who_is_online_flag = 1;
            dbg_log("[BUSOFF] recovered, re-announcing\r\n");
        }
        return;
    }

    /* BS 置起（bus-off）。退避期間不重複嘗試 */
    if (backoff_ticks > 0u) {
        return;
    }

    if (soft_fail_count >= BUSOFF_SOFT_RECOVER_MAX) {
        /* 軟復原連續失敗，升級整顆重開（走既有開機公告流程） */
        NVIC_SystemReset();
    }

    dbg_log("[BUSOFF] detected, soft recovery attempt %u\r\n",
            (unsigned)(soft_fail_count + 1u));

    osMutexWait(can_mutex, osWaitForever);
    /* 完整重跑 CAN 初始化而非裸 ResetMode toggle：bus-off 瞬間硬體自動
     * 置 RM=1（UM 25.5.2），而 0x40-0x5C 在復位模式下對映 ACR/AMR
     * （UM 表 73）——RM=1 窗口內任何 TX 緩衝器寫入都會摧毀驗收濾波器
     * （2026-07-14 實測：node1/node20 風暴後 TX 活 RX 死、僅斷電可救）。
     * CAN_NVIC_Init 會重寫 CDR/BTR/ACR/AMR/IER，其結尾清 RM 的動作即
     * 觸發協定規定的 128×11 recessive bits bus-off 復原序列。 */
    CAN_NVIC_Init();
    osMutexRelease(can_mutex);

    soft_fail_count++;
    recovery_pending = 1u;
    backoff_ticks    = BUSOFF_BACKOFF_TICKS;
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
    uint8_t waiting_ack = 0;
    uint8_t missed_ack_count = 0;
    (void)argument;

    dbg_log("[CONNECT] Thread started, sending initial CONNECT_REQ\r\n");

    /* Stagger initial CONNECT_REQ by NODE_ID to prevent simultaneous
     * transmissions when multiple nodes power on at the same time.
     * Node N waits N * 500ms before its first send (max 10s for node 20).
     * 以 100ms tick 呼叫 g_hb_can_connect++，避免此延遲期間心跳計數器停滯，
     * 造成 watchdog 誤判此執行緒卡死（尤其 node_id 較大時延遲較長）。 */
    {
        uint32_t stagger_ms = (uint32_t)MY_NODE_ID * 500u;
        uint32_t waited_ms  = 0u;
        while (waited_ms < stagger_ms) {
            osDelay(100);
            waited_ms += 100;
            g_hb_can_connect++;
        }
    }

    /* 啟動後立即發送 */
    send_connect_req();
    waiting_ack = 1;

    for (;;) {
        elapsed_ms  = 0;
        interval_ms = (g_conn_state == CONN_CONNECTED)
                      ? ((uint32_t)CAN_HEARTBEAT_INTERVAL_S * 1000u)  /* 300s */
                      : ((uint32_t)CAN_CONNECT_RETRY_S     * 1000u);  /* 30s  */

        /* 以 100ms 為最小察測單位，對強影響可忽略 */
        while (elapsed_ms < interval_ms) {
            osDelay(100);
            elapsed_ms += 100;
            g_hb_can_connect++;
            busoff_check_tick();

            /* CONNECT_ACK 收到：切換心跳模式，重置計時 */
            if (g_ack_received_flag) {
                g_ack_received_flag = 0;
                waiting_ack = 0;
                missed_ack_count = 0;
                if (g_conn_state != CONN_CONNECTED) {
                    g_conn_state = CONN_CONNECTED;
                    dbg_log("[CONNECT] ACK received, switching to heartbeat (%us)\r\n",
                            CAN_HEARTBEAT_INTERVAL_S);
                }
                elapsed_ms  = 0;
                interval_ms = (uint32_t)CAN_HEARTBEAT_INTERVAL_S * 1000u;
            }

            /* WHO_IS_ONLINE 收到：不論狀態立即發送，但仍加 NODE_ID jitter
             * 避免多節點同時搶 CAN bus。以 100ms 分格 tick g_hb_can_connect，
             * 與 stagger 延遲同樣做法，避免 jitter 期間心跳計數器停滯。 */
            if (g_who_is_online_flag) {
                uint32_t jitter_ms    = (uint32_t)MY_NODE_ID * 50u;
                uint32_t jittered_ms  = 0u;
                g_who_is_online_flag = 0;
                dbg_log("[CONNECT] WHO_IS_ONLINE: sending CONNECT_REQ (jitter %ums)\r\n",
                        (unsigned)jitter_ms);
                while (jittered_ms < jitter_ms) {
                    uint32_t chunk = (jitter_ms - jittered_ms > 100u) ? 100u : (jitter_ms - jittered_ms);
                    osDelay(chunk);
                    jittered_ms += chunk;
                    g_hb_can_connect++;
                }
                break;
            }
        }

        /* 已連線狀態下，若上一個 CONNECT_REQ 在整個心跳週期內都沒收到 ACK，
         * 視為連線遺失，回切為 DISCONNECTED（改為 30s 重試）。 */
        if (g_conn_state == CONN_CONNECTED && waiting_ack) {
            if (missed_ack_count < 0xFF) {
                missed_ack_count++;
            }
            if (missed_ack_count >= CONNECT_ACK_MISS_MAX) {
                g_conn_state = CONN_DISCONNECTED;
                dbg_log("[CONNECT] ACK timeout, switching to DISCONNECTED (retry %us)\r\n",
                        CAN_CONNECT_RETRY_S);
            }
        }

        send_connect_req();
        waiting_ack = 1;
    }
}

/*============================================================================
 * Display helpers — role string and current formatting
 *============================================================================*/

/**
 * @brief  Format role number into a 4-char display string.
 *
 *   role 0     : "----"  (unconfigured)
 *   role 1..19 : "C1XY"  bus 1, node 01..19
 *   role 20    : "End1"  + HT1621_DP3  → shows "End.1"
 *   role 21..39: "C2XY"  bus 2, node 01..19
 *   role 40    : "End2"  + HT1621_DP3  → shows "End.2"
 *
 * @param  role     1..40 or 0 (unconfigured).
 * @param  buf4     4-element char buffer (NOT null-terminated).
 * @param  dp_mask  Output dot_mask for ht1621_show_text().
 */
static void format_role_str(uint8_t role, char *buf4, uint8_t *dp_mask)
{
    uint8_t bus_id, node_id;

    *dp_mask = HT1621_DP_NONE;
    if (role == 0u) {
        buf4[0] = '-'; buf4[1] = '-'; buf4[2] = '-'; buf4[3] = '-';
        return;
    }
    bus_id  = (uint8_t)((role - 1u) / 20u + 1u);
    node_id = (uint8_t)((role - 1u) % 20u + 1u);
    if (node_id == 20u) {
        /* "End.1" or "End.2": text "End<bus>", dp3 places the dot after 'd' */
        buf4[0] = 'E'; buf4[1] = 'n'; buf4[2] = 'd';
        buf4[3] = (char)('0' + bus_id);
        *dp_mask = HT1621_DP3;
    } else {
        /* "C1XY" or "C2XY" */
        buf4[0] = 'C';
        buf4[1] = (char)('0' + bus_id);
        buf4[2] = (char)('0' + node_id / 10u);
        buf4[3] = (char)('0' + node_id % 10u);
    }
}

/**
 * @brief  Format raw current (x0.01 A) as a 4-char display string.
 *
 *   raw <  1000  →  X.XXX  (dp1)  e.g. raw=234  → "2340" + DP1 → "2.340 A"
 *   raw < 10000  →  XX.XX  (dp2)  e.g. raw=1234 → "1234" + DP2 → "12.34 A"
 *   raw >= 10000 →  XXX.X  (dp3)  e.g. raw=10050→ "1005" + DP3 → "100.5 A"
 *
 * @param  raw      Current in x0.01 A units (phase_metrics[n].current).
 * @param  buf4     4-element char buffer (NOT null-terminated).
 * @param  dp_mask  Output dot_mask for ht1621_show_text().
 */
static void format_current(uint16_t raw, char *buf4, uint8_t *dp_mask)
{
    uint32_t val;

    if (raw < 1000u) {
        /* Scale to x0.001 A for X.XXX display */
        val      = (uint32_t)raw * 10u;
        buf4[0]  = (char)('0' + val / 1000u);
        buf4[1]  = (char)('0' + (val / 100u) % 10u);
        buf4[2]  = (char)('0' + (val / 10u)  % 10u);
        buf4[3]  = (char)('0' + val % 10u);
        *dp_mask = HT1621_DP1;
    } else if (raw < 10000u) {
        /* XX.XX: use raw directly */
        buf4[0]  = (char)('0' + raw / 1000u);
        buf4[1]  = (char)('0' + (raw / 100u) % 10u);
        buf4[2]  = (char)('0' + (raw / 10u)  % 10u);
        buf4[3]  = (char)('0' + raw % 10u);
        *dp_mask = HT1621_DP2;
    } else {
        /* Scale to x0.1 A for XXX.X display */
        val      = raw / 10u;
        buf4[0]  = (char)('0' + val / 1000u);
        buf4[1]  = (char)('0' + (val / 100u) % 10u);
        buf4[2]  = (char)('0' + (val / 10u)  % 10u);
        buf4[3]  = (char)('0' + val % 10u);
        *dp_mask = HT1621_DP3;
    }
}

/**
 * @brief  Build display content for one carousel page.
 *
 *   Page 0: role string         Page 1: "L1  "   Page 2: L1 current
 *   Page 3: "L2  "              Page 4: L2 current
 *   Page 5: "L3  "              Page 6: L3 current
 *
 * @param  page     0..6
 * @param  role     Current configured role (0 = unconfigured).
 * @param  curr3    3-element array of x0.01 A currents: [0]=L1, [1]=L2, [2]=L3.
 * @param  buf4     Output 4-char display buffer (NOT null-terminated).
 * @param  dp_mask  Output dot_mask.
 */
static void format_page(uint8_t page, uint8_t role, const uint16_t *curr3,
                        char *buf4, uint8_t *dp_mask)
{
    switch (page) {
        case 0:
            format_role_str(role, buf4, dp_mask);
            break;
        case 1:
            buf4[0] = 'L'; buf4[1] = '1'; buf4[2] = ' '; buf4[3] = ' ';
            *dp_mask = HT1621_DP_NONE;
            break;
        case 2:
            format_current(curr3[0], buf4, dp_mask);
            break;
        case 3:
            buf4[0] = 'L'; buf4[1] = '2'; buf4[2] = ' '; buf4[3] = ' ';
            *dp_mask = HT1621_DP_NONE;
            break;
        case 4:
            format_current(curr3[1], buf4, dp_mask);
            break;
        case 5:
            buf4[0] = 'L'; buf4[1] = '3'; buf4[2] = ' '; buf4[3] = ' ';
            *dp_mask = HT1621_DP_NONE;
            break;
        default: /* case 6 */
            format_current(curr3[2], buf4, dp_mask);
            break;
    }
}

/*============================================================================
 * Thread_Display: LCD 顯示與按鈕狀態機
 *
 * 50 ms tick 主迴圈。
 *
 * NORMAL 狀態：
 *   7 頁輪播，每頁 2 s (= 40 ticks)。
 *   role=0 時持續顯示 "----"，等待 commissioning。
 *   雙鍵同時按住 >= 1.5 s (30 ticks) → 進入 SETTING。
 *
 * SETTING 狀態：
 *   固定顯示候選 role 字串。
 *   UP / DOWN 單擊循環切換 role 1..40。
 *   雙鍵同時按住 >= 1.5 s → pdu_role_save() + NVIC_SystemReset()。
 *============================================================================*/
void Thread_Display(void const *argument)
{
    typedef enum { STATE_NORMAL = 0, STATE_SETTING } disp_state_t;

    disp_state_t state        = STATE_NORMAL;
    uint8_t      current_role = g_my_role;   /* set by main() before Init_Thread */
    uint8_t      candidate    = current_role ? current_role : 1u;
    uint8_t      page         = 0u;
    uint16_t     page_ticks   = 0u;          /* ticks on current page (0..39) */
    uint16_t     both_ticks   = 0u;          /* consecutive ticks with both buttons held */
    uint16_t     setting_timeout_ticks = 0u; /* ticks for timeout in STATE_SETTING */
    uint8_t      disp_dirty   = 1u;          /* 1 = re-render needed */
    char         buf4[5]      = { 0 };       /* [4] unused, just for safety */
    uint8_t      dp           = HT1621_DP_NONE;
    uint16_t     curr3[3]     = { 0u, 0u, 0u };

    (void)argument;

    /* PB3 = HT1621 LCD power enable. Must be driven HIGH before the
     * HT1621 is initialised/powered on. */
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        GPIO_SetBits(GPIOB, GPIO_Pin_3);
    }

    ht1621_power_on();
    buttons_init();
    ht1621_clear();

    dbg_log("[DISP] Thread started, role=%u node=%u bus=%u\r\n",
            (unsigned)current_role, (unsigned)g_my_node_id, (unsigned)g_my_bus_id);

    for (;;) {
        osDelay(50u);      /* 50 ms tick */
        btn_tick();        /* update debounce state machines */

        /* Snapshot phase currents under data_mutex */
        osMutexWait(data_mutex, osWaitForever);
        curr3[0] = phase_metrics[1].current;  /* L1 */
        curr3[1] = phase_metrics[2].current;  /* L2 */
        curr3[2] = phase_metrics[3].current;  /* L3 */
        osMutexRelease(data_mutex);

        /* Dual-button long-press accumulator (capped to prevent overflow) */
        if (btn_is_held(BTN_UP) && btn_is_held(BTN_DOWN)) {
            if (both_ticks < 0xFFFFu) { both_ticks++; }
        } else {
            both_ticks = 0u;
        }

        /* ================================================================
         * NORMAL state: 7-page carousel
         * ================================================================ */
        if (state == STATE_NORMAL) {

            /* Drain single-press events — not used in NORMAL mode */
            (void)btn_just_pressed(BTN_UP);
            (void)btn_just_pressed(BTN_DOWN);

            if (current_role == 0u) {
                /* Unconfigured: show "----" until user commissions the node */
                if (disp_dirty) {
                    ht1621_show_text("----", HT1621_DP_NONE);
                    disp_dirty = 0u;
                }
            } else {
                /* Advance page every 2 s */
                if (++page_ticks >= 40u) {
                    page_ticks = 0u;
                    page       = (uint8_t)((page + 1u) % 7u);
                    disp_dirty = 1u;
                }
                if (disp_dirty) {
                    format_page(page, current_role, curr3, buf4, &dp);
                    ht1621_show_text(buf4, dp);
                    disp_dirty = 0u;
                }
            }

            /* Transition: dual hold >= 1.5 s -> SETTING */
            if (both_ticks >= 30u) {
                both_ticks = 0u;
                candidate  = current_role ? current_role : 1u;
                page_ticks = 0u;
                setting_timeout_ticks = 0u; /* Reset 20s timeout counter on entry */
                state      = STATE_SETTING;
                disp_dirty = 1u;
                /* Clear any accumulated single-press events before entering SETTING */
                (void)btn_just_pressed(BTN_UP);
                (void)btn_just_pressed(BTN_DOWN);
                dbg_log("[DISP] -> SETTING (candidate=%u)\r\n", (unsigned)candidate);
            }

        /* ================================================================
         * SETTING state: role selection
         * ================================================================ */
        } else {
            setting_timeout_ticks++;

            /* Single-press navigation — only when not in a dual-hold sequence */
            if (both_ticks == 0u) {
                if (btn_just_pressed(BTN_UP)) {
                    candidate  = (candidate < 40u) ? (uint8_t)(candidate + 1u) : 1u;
                    disp_dirty = 1u;
                    setting_timeout_ticks = 0u; /* Reset timeout upon keypress */
                    dbg_log("[SETTING] UP   -> candidate=%u\r\n", (unsigned)candidate);
                }
                if (btn_just_pressed(BTN_DOWN)) {
                    candidate  = (candidate > 1u) ? (uint8_t)(candidate - 1u) : 40u;
                    disp_dirty = 1u;
                    setting_timeout_ticks = 0u; /* Reset timeout upon keypress */
                    dbg_log("[SETTING] DOWN -> candidate=%u\r\n", (unsigned)candidate);
                }
            } else {
                setting_timeout_ticks = 0u; /* Reset timeout while keys are held down */
                /* Drain events during hold to prevent spurious presses after release */
                (void)btn_just_pressed(BTN_UP);
                (void)btn_just_pressed(BTN_DOWN);
                /* Log dual-hold progress every 0.5 s (10 ticks) */
                if ((both_ticks % 10u) == 0u) {
                    dbg_log("[SETTING] both held %u ticks (%u ms), UP=%u DOWN=%u\r\n",
                            (unsigned)both_ticks,
                            (unsigned)(both_ticks * 50u),
                            (unsigned)btn_is_held(BTN_UP),
                            (unsigned)btn_is_held(BTN_DOWN));
                }
            }

            /* Confirm: dual hold >= 1.5 s -> save to Flash + reset */
            if (both_ticks >= 30u) {
                dbg_log("[SETTING] confirm: saving role=%u to Flash addr=0x0800FC00\r\n",
                        (unsigned)candidate);
                ht1621_show_text("----", HT1621_DP_NONE);
                if (pdu_role_save(candidate) == 0) {
                    dbg_log("[SETTING] save OK, rebooting...\r\n");
                    osDelay(500u);          /* brief pause before reset */
                    NVIC_SystemReset();     /* reboot with new role */
                }
                /* Save failed: clear counter, mark dirty to re-render candidate */
                dbg_log("[SETTING] pdu_role_save FAILED, returning to selection\r\n");
                both_ticks = 0u;
                disp_dirty = 1u;
                continue;                   /* skip render this tick */
            }

            /* 20 seconds timeout exit (20s / 50ms = 400 ticks) */
            if (setting_timeout_ticks >= 400u) {
                dbg_log("[SETTING] timeout 20s reached, returning to STATE_NORMAL without saving\r\n");
                state      = STATE_NORMAL;
                page       = 0u;
                page_ticks = 0u;
                disp_dirty = 1u;
                both_ticks = 0u;
                continue;
            }
    
            /* Render candidate role */
            if (disp_dirty) {
                format_role_str(candidate, buf4, &dp);
                ht1621_show_text(buf4, dp);
                dbg_log("[SETTING] render: '%c%c%c%c' dp=0x%02X\r\n",
                        buf4[0], buf4[1], buf4[2], buf4[3], (unsigned)dp);
                disp_dirty = 0u;
            }
        }
    }
}
