/**
 * @file modbus_rtu.c
 * @brief Modbus RTU Master implementation over UART1/RS-485.
 *
 * TX: polling (UART_SendData + wait TXC flag)
 * RX: interrupt ring buffer (UART1_IRQHandler in uart_txrx_interrupt.c)
 * DE: PA8 GPIO for RS-485 direction control
 *
 * All raw TX/RX bytes are logged via dbg_log() for protocol debugging.
 */

#include "modbus_rtu.h"
#include "uart_txrx_interrupt.h"
#include <string.h>
#include <cmsis_os.h>

/*============================================================================
 * Debug log (defined in Thread.c)
 *============================================================================*/
extern void dbg_log(const char *fmt, ...);

/*============================================================================
 * RX Ring Buffer (shared with UART1_IRQHandler)
 *============================================================================*/
volatile uint8_t  mb_rx_buf[MB_RX_BUF_SIZE];
volatile uint16_t mb_rx_head = 0;
volatile uint16_t mb_rx_tail = 0;

/*============================================================================
 * CRC16 Lookup Table (Modbus polynomial 0xA001)
 *============================================================================*/
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

uint16_t modbus_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    for (i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc16_table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

/*============================================================================
 * Internal: Ring buffer helpers
 *============================================================================*/

/** @brief Number of bytes available to read from ring buffer */
static uint16_t mb_rx_available(void)
{
    uint16_t h = mb_rx_head;
    uint16_t t = mb_rx_tail;
    return (h >= t) ? (h - t) : (MB_RX_BUF_SIZE - t + h);
}

/** @brief Read one byte from ring buffer. Caller must check available first. */
static uint8_t mb_rx_read_byte(void)
{
    uint8_t b = mb_rx_buf[mb_rx_tail];
    mb_rx_tail = (mb_rx_tail + 1) % MB_RX_BUF_SIZE;
    return b;
}

/** @brief Flush the RX ring buffer */
static void mb_rx_flush(void)
{
    mb_rx_tail = mb_rx_head;
}

/*============================================================================
 * Internal: UART1 TX (polling mode) with DE control
 *============================================================================*/

static void mb_send_frame(const uint8_t *data, uint16_t len)
{
    uint16_t i;

    MB_DE_HIGH();  /* Enable TX on RS-485 */

    /* Small delay for DE setup time (~10us at 9600 baud is negligible) */

    for (i = 0; i < len; i++) {
        UART_SendData(UART1, data[i]);
        while (!UART_GetFlagStatus(UART1, UART_FLAG_TXEPT));
    }

    /* Wait for last byte to fully shift out */
    while (!UART_GetFlagStatus(UART1, UART_CSR_TXC));

    MB_DE_LOW();   /* Switch to RX mode */
}

/*============================================================================
 * Internal: Wait for response bytes with timeout
 *============================================================================*/

/**
 * @brief Wait until at least 'min_bytes' are in the RX buffer, or timeout.
 * @param min_bytes  Minimum bytes to collect
 * @param timeout_ms Timeout in ms
 * @return Number of bytes available, or 0 on timeout
 */
static uint16_t mb_wait_response(uint16_t min_bytes, uint32_t timeout_ms)
{
    uint32_t elapsed = 0;
    uint16_t avail;
    uint16_t last_avail = 0;
    uint32_t idle_ms = 0;

    while (elapsed < timeout_ms) {
        osDelay(1);
        elapsed++;

        avail = mb_rx_available();

        if (avail >= min_bytes) {
            /* Got enough, but wait a bit more for end-of-frame silence
             * (3.5 char times ~ 4ms at 9600) */
            if (avail == last_avail) {
                idle_ms++;
                if (idle_ms >= MB_INTER_FRAME_MS) {
                    return avail;
                }
            } else {
                idle_ms = 0;
            }
        }
        last_avail = avail;
    }
    return mb_rx_available();
}

/*============================================================================
 * Internal: Drain the bus until it is quiet
 *
 * After a timeout / CRC error the slave may still be transmitting the tail of
 * its (now-orphaned) response. If we issue the next request immediately we
 * collide with that trailing burst, so the next "RX" is in fact leftover from
 * the previous frame (e.g. "Bad slave: expected 1 got 0"). Wait until no new
 * bytes have arrived for `MB_BUS_QUIET_MS`, then flush.
 *============================================================================*/

static void mb_drain_bus(void)
{
    uint32_t elapsed = 0;
    uint32_t quiet = 0;
    uint16_t last = mb_rx_available();
    uint16_t now;

    while (elapsed < MB_BUS_QUIET_MAX_MS) {
        osDelay(1);
        elapsed++;
        now = mb_rx_available();
        if (now == last) {
            quiet++;
            if (quiet >= MB_BUS_QUIET_MS) {
                break;
            }
        } else {
            quiet = 0;
            last = now;
        }
    }
    mb_rx_flush();
}

/*============================================================================
 * Internal: Debug hex dump
 *============================================================================*/

static void mb_log_hex(const char *prefix, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    dbg_log("[MB] %s (%u bytes):", prefix, len);
    for (i = 0; i < len && i < 100; i++) {
        dbg_log(" %02X", data[i]);
    }
    dbg_log("\r\n");
}

/*============================================================================
 * Internal: Read response into local buffer and validate
 *============================================================================*/

/**
 * @brief Collect response from RX buffer into a linear array.
 * @param buf      Output buffer
 * @param max_len  Buffer size
 * @return Number of bytes read
 */
static uint16_t mb_collect_response(uint8_t *buf, uint16_t max_len)
{
    uint16_t count = 0;
    while (mb_rx_available() > 0 && count < max_len) {
        buf[count++] = mb_rx_read_byte();
    }
    return count;
}

/*============================================================================
 * modbus_init
 *============================================================================*/

void modbus_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Initialize UART1 for 9600/8N1 */
    {
        UART_InitTypeDef UART_InitStructure;
        NVIC_InitTypeDef NVIC_InitStructure;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_UART1, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBENR_GPIOA, ENABLE);

        /* NVIC for UART1 RX interrupt */
        NVIC_InitStructure.NVIC_IRQChannel = UART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPriority = 2;  /* Higher than UART2 (3) */
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        /* UART1 config: 9600 / 8N1 */
        UART_StructInit(&UART_InitStructure);
        UART_InitStructure.BaudRate = MB_BAUDRATE;
        UART_InitStructure.WordLength = UART_WordLength_8b;
        UART_InitStructure.StopBits = UART_StopBits_1;
        UART_InitStructure.Parity = UART_Parity_No;
        UART_InitStructure.HWFlowControl = UART_HWFlowControl_None;
        UART_InitStructure.Mode = UART_Mode_Rx | UART_Mode_Tx;

        UART_Init(UART1, &UART_InitStructure);
        /* Only enable RX interrupt; TX uses polling */
        UART_ITConfig(UART1, UART_IT_RXIEN, ENABLE);
        UART_Cmd(UART1, ENABLE);

        /* GPIO: PA9=TX (AF_PP), PA10=RX (FLOATING) */
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

        GPIO_StructInit(&GPIO_InitStructure);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_FLOATING;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    }

    /* PA8 = DE pin (Push-Pull output, default LOW = RX mode) */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = MB_DE_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(MB_DE_PORT, &GPIO_InitStructure);
    MB_DE_LOW();

    /* Flush RX buffer */
    mb_rx_head = 0;
    mb_rx_tail = 0;
}

/*============================================================================
 * modbus_read_regs — Read Registers (FC03 Holding / FC04 Input)
 *
 * Request frame (8 bytes):
 *   [slave_id] [FC] [start_hi] [start_lo] [qty_hi] [qty_lo] [crc_lo] [crc_hi]
 *
 * Response frame (3 + 2*N + 2 bytes):
 *   [slave_id] [FC] [byte_count] [reg0_hi] [reg0_lo] ... [crc_lo] [crc_hi]
 *============================================================================*/

int modbus_read_regs(uint8_t slave_id, uint8_t fc, uint16_t start_reg,
                     uint16_t num_regs, uint16_t *out_regs)
{
    uint8_t req[8];
    uint8_t resp[256];
    uint16_t crc;
    uint16_t resp_len;
    uint16_t expected_bytes;
    uint16_t i;
    int retry;

    /* Build request */
    req[0] = slave_id;
    req[1] = fc;
    req[2] = (uint8_t)(start_reg >> 8);
    req[3] = (uint8_t)(start_reg & 0xFF);
    req[4] = (uint8_t)(num_regs >> 8);
    req[5] = (uint8_t)(num_regs & 0xFF);
    crc = modbus_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    expected_bytes = 3 + num_regs * 2 + 2;  /* header + data + CRC */

    for (retry = 0; retry <= MB_MAX_RETRIES; retry++) {
        /* On retries the slave may still be transmitting the tail of the
         * previous (orphaned) response. Wait for the bus to be quiet, then
         * flush, so we never mistake leftover bytes for the new reply. */
        if (retry == 0) {
            mb_rx_flush();
        } else {
            mb_drain_bus();
        }

        /* Log TX */
        mb_log_hex("TX", req, 8);

        /* Send request */
        mb_send_frame(req, 8);

        /* Wait for inter-frame gap */
        osDelay(MB_INTER_FRAME_MS);

        /* Wait for response */
        mb_wait_response(expected_bytes, MB_RESPONSE_TIMEOUT_MS);
        resp_len = mb_collect_response(resp, sizeof(resp));

        /* Log RX raw */
        if (resp_len > 0) {
            mb_log_hex("RX", resp, resp_len);
        } else {
            dbg_log("[MB] RX timeout (slave=%u retry=%d)\r\n", slave_id, retry);
            continue;
        }

        /* Validate: minimum length */
        if (resp_len < 5) {
            dbg_log("[MB] RX too short: %u bytes\r\n", resp_len);
            continue;
        }

        /* Check for exception response */
        if (resp[1] & 0x80) {
            dbg_log("[MB] Exception: FC=0x%02X code=0x%02X\r\n", resp[1], resp[2]);
            return MB_ERR_EXCEPTION;
        }

        /* Validate slave ID */
        if (resp[0] != slave_id) {
            dbg_log("[MB] Bad slave: expected %u got %u\r\n", slave_id, resp[0]);
            continue;
        }

        /* Validate function code */
        if (resp[1] != fc) {
            dbg_log("[MB] Bad FC: expected 0x%02X got 0x%02X\r\n", fc, resp[1]);
            continue;
        }

        /* Validate byte count */
        if (resp[2] != num_regs * 2) {
            dbg_log("[MB] Bad byte count: expected %u got %u\r\n",
                    num_regs * 2, resp[2]);
            continue;
        }

        /* Validate total length */
        if (resp_len < expected_bytes) {
            dbg_log("[MB] Incomplete: expected %u got %u\r\n",
                    expected_bytes, resp_len);
            continue;
        }

        /* Validate CRC */
        crc = modbus_crc16(resp, expected_bytes - 2);
        if (resp[expected_bytes - 2] != (uint8_t)(crc & 0xFF) ||
            resp[expected_bytes - 1] != (uint8_t)(crc >> 8)) {
            dbg_log("[MB] CRC error: calc=0x%04X got=0x%02X%02X\r\n",
                    crc, resp[expected_bytes - 1], resp[expected_bytes - 2]);
            continue;
        }

        /* Parse register values (big-endian in Modbus) */
        for (i = 0; i < num_regs; i++) {
            out_regs[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
        }

        return MB_OK;
    }

    return MB_ERR_TIMEOUT;
}

/*============================================================================
 * modbus_write_single_reg (FC06)
 *
 * Request frame (8 bytes):
 *   [slave_id] [0x06] [reg_hi] [reg_lo] [val_hi] [val_lo] [crc_lo] [crc_hi]
 *
 * Response: echo of request (8 bytes)
 *============================================================================*/

int modbus_write_single_reg(uint8_t slave_id, uint16_t reg_addr, uint16_t value)
{
    uint8_t req[8];
    uint8_t resp[16];
    uint16_t crc;
    uint16_t resp_len;
    int retry;

    /* Build request */
    req[0] = slave_id;
    req[1] = MB_FC_WRITE_SINGLE_REG;
    req[2] = (uint8_t)(reg_addr >> 8);
    req[3] = (uint8_t)(reg_addr & 0xFF);
    req[4] = (uint8_t)(value >> 8);
    req[5] = (uint8_t)(value & 0xFF);
    crc = modbus_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    for (retry = 0; retry <= MB_MAX_RETRIES; retry++) {
        if (retry == 0) {
            mb_rx_flush();
        } else {
            mb_drain_bus();
        }

        mb_log_hex("TX", req, 8);
        mb_send_frame(req, 8);
        osDelay(MB_INTER_FRAME_MS);

        mb_wait_response(8, MB_RESPONSE_TIMEOUT_MS);
        resp_len = mb_collect_response(resp, sizeof(resp));

        if (resp_len > 0) {
            mb_log_hex("RX", resp, resp_len);
        } else {
            dbg_log("[MB] RX timeout FC06 (slave=%u retry=%d)\r\n", slave_id, retry);
            continue;
        }

        /* Check for exception */
        if (resp_len >= 5 && (resp[1] & 0x80)) {
            dbg_log("[MB] Exception FC06: code=0x%02X\r\n", resp[2]);
            return MB_ERR_EXCEPTION;
        }

        /* FC06 response should echo the request exactly */
        if (resp_len < 8) {
            dbg_log("[MB] FC06 short response: %u bytes\r\n", resp_len);
            continue;
        }

        /* Validate CRC */
        crc = modbus_crc16(resp, 6);
        if (resp[6] != (uint8_t)(crc & 0xFF) ||
            resp[7] != (uint8_t)(crc >> 8)) {
            dbg_log("[MB] FC06 CRC error\r\n");
            continue;
        }

        /* Validate echo */
        if (resp[0] != slave_id || resp[1] != MB_FC_WRITE_SINGLE_REG) {
            dbg_log("[MB] FC06 echo mismatch\r\n");
            continue;
        }

        return MB_OK;
    }

    return MB_ERR_TIMEOUT;
}

/*============================================================================
 * modbus_write_multi_regs (FC10)
 *
 * Request frame (7 + 2*N + 2 bytes):
 *   [slave] [0x10] [start_hi] [start_lo] [qty_hi] [qty_lo] [byte_count]
 *   [val0_hi] [val0_lo] ... [crc_lo] [crc_hi]
 *
 * Response (8 bytes):
 *   [slave] [0x10] [start_hi] [start_lo] [qty_hi] [qty_lo] [crc_lo] [crc_hi]
 *============================================================================*/

int modbus_write_multi_regs(uint8_t slave_id, uint16_t start_reg,
                            uint16_t num_regs, const uint16_t *values)
{
    uint8_t req[64];  /* Max: 7 + 2*26 + 2 = 61 bytes (26 regs) */
    uint8_t resp[16];
    uint16_t crc;
    uint16_t resp_len;
    uint16_t req_len;
    uint16_t i;
    int retry;

    if (num_regs > 26) return MB_ERR_BAD_FC;  /* Buffer limit */

    /* Build request */
    req[0] = slave_id;
    req[1] = MB_FC_WRITE_MULTI_REGS;
    req[2] = (uint8_t)(start_reg >> 8);
    req[3] = (uint8_t)(start_reg & 0xFF);
    req[4] = (uint8_t)(num_regs >> 8);
    req[5] = (uint8_t)(num_regs & 0xFF);
    req[6] = (uint8_t)(num_regs * 2);  /* byte count */

    for (i = 0; i < num_regs; i++) {
        req[7 + i * 2] = (uint8_t)(values[i] >> 8);
        req[8 + i * 2] = (uint8_t)(values[i] & 0xFF);
    }
    req_len = 7 + num_regs * 2;
    crc = modbus_crc16(req, req_len);
    req[req_len]     = (uint8_t)(crc & 0xFF);
    req[req_len + 1] = (uint8_t)(crc >> 8);
    req_len += 2;

    for (retry = 0; retry <= MB_MAX_RETRIES; retry++) {
        mb_rx_flush();

        mb_log_hex("TX", req, req_len);
        mb_send_frame(req, req_len);
        osDelay(MB_INTER_FRAME_MS);

        mb_wait_response(8, MB_RESPONSE_TIMEOUT_MS);
        resp_len = mb_collect_response(resp, sizeof(resp));

        if (resp_len > 0) {
            mb_log_hex("RX", resp, resp_len);
        } else {
            dbg_log("[MB] RX timeout FC10 (slave=%u retry=%d)\r\n", slave_id, retry);
            continue;
        }

        if (resp_len >= 5 && (resp[1] & 0x80)) {
            dbg_log("[MB] Exception FC10: code=0x%02X\r\n", resp[2]);
            return MB_ERR_EXCEPTION;
        }

        if (resp_len < 8) {
            dbg_log("[MB] FC10 short response: %u bytes\r\n", resp_len);
            continue;
        }

        crc = modbus_crc16(resp, 6);
        if (resp[6] != (uint8_t)(crc & 0xFF) ||
            resp[7] != (uint8_t)(crc >> 8)) {
            dbg_log("[MB] FC10 CRC error\r\n");
            continue;
        }

        if (resp[0] != slave_id || resp[1] != MB_FC_WRITE_MULTI_REGS) {
            dbg_log("[MB] FC10 echo mismatch\r\n");
            continue;
        }

        return MB_OK;
    }

    return MB_ERR_TIMEOUT;
}
