/**
 * @file modbus_rtu.h
 * @brief Modbus RTU Master driver for UART1/RS-485.
 *
 * Hardware: PA9=TX, PA10=RX, PA8=DE (RS-485 direction)
 * Baud rate: 9600, 8N1
 * TX: polling mode
 * RX: interrupt-driven ring buffer
 */

#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#include "hal_conf.h"
#include <stdint.h>

/*============================================================================
 * Configuration
 *============================================================================*/

#define MB_BAUDRATE             9600
#define MB_RESPONSE_TIMEOUT_MS  200     /* Max wait for slave response */
#define MB_MAX_RETRIES          2       /* Retries on timeout/CRC error */
#define MB_INTER_FRAME_MS       5       /* 3.5 char silence at 9600 ~ 4ms, use 5ms */
#define MB_RX_BUF_SIZE          256     /* Ring buffer for UART1 RX ISR */

/*============================================================================
 * Modbus Function Codes
 *============================================================================*/

#define MB_FC_READ_INPUT_REGS   0x04
#define MB_FC_WRITE_SINGLE_REG  0x06
#define MB_FC_WRITE_MULTI_REGS  0x10

/*============================================================================
 * Error Codes
 *============================================================================*/

#define MB_OK                   0
#define MB_ERR_TIMEOUT         -1
#define MB_ERR_CRC             -2
#define MB_ERR_EXCEPTION       -3
#define MB_ERR_BAD_SLAVE       -4
#define MB_ERR_BAD_FC          -5
#define MB_ERR_SHORT_RESP      -6

/*============================================================================
 * RS-485 DE Pin Control (PA8)
 *============================================================================*/

#define MB_DE_PORT              GPIOA
#define MB_DE_PIN               GPIO_Pin_8

#define MB_DE_HIGH()            GPIO_SetBits(MB_DE_PORT, MB_DE_PIN)
#define MB_DE_LOW()             GPIO_ResetBits(MB_DE_PORT, MB_DE_PIN)

/*============================================================================
 * RX Ring Buffer (written by UART1 ISR, read by modbus_rtu.c)
 *============================================================================*/

extern volatile uint8_t  mb_rx_buf[MB_RX_BUF_SIZE];
extern volatile uint16_t mb_rx_head;    /* ISR writes here */
extern volatile uint16_t mb_rx_tail;    /* Consumer reads here */

/*============================================================================
 * API
 *============================================================================*/

/**
 * @brief Initialize UART1 for Modbus RTU (9600/8N1) + PA8 DE pin.
 *        Call once from main() before osKernelStart().
 */
void modbus_init(void);

/**
 * @brief Read Input Registers (FC04).
 * @param slave_id  Slave address (1-247)
 * @param start_reg Starting register address
 * @param num_regs  Number of registers to read (1-125)
 * @param out_regs  Output buffer for register values (caller allocates)
 * @return MB_OK on success, negative error code on failure.
 *         On success, out_regs[0..num_regs-1] contain the register values.
 */
int modbus_read_input_regs(uint8_t slave_id, uint16_t start_reg,
                           uint16_t num_regs, uint16_t *out_regs);

/**
 * @brief Write Single Register (FC06).
 * @param slave_id  Slave address
 * @param reg_addr  Register address
 * @param value     Value to write
 * @return MB_OK on success, negative error code on failure.
 */
int modbus_write_single_reg(uint8_t slave_id, uint16_t reg_addr, uint16_t value);

/**
 * @brief Write Multiple Registers (FC10).
 * @param slave_id  Slave address
 * @param start_reg Starting register address
 * @param num_regs  Number of registers (1-123)
 * @param values    Register values to write
 * @return MB_OK on success, negative error code on failure.
 */
int modbus_write_multi_regs(uint8_t slave_id, uint16_t start_reg,
                            uint16_t num_regs, const uint16_t *values);

/**
 * @brief Compute Modbus CRC16.
 * @param data  Data buffer
 * @param len   Data length
 * @return CRC16 value (low byte first in Modbus frame)
 */
uint16_t modbus_crc16(const uint8_t *data, uint16_t len);

#endif /* __MODBUS_RTU_H */
