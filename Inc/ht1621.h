////////////////////////////////////////////////////////////////////////////////
/// @file    ht1621.h
/// @author  PDU Project
/// @brief   HT1621 4-digit 7-segment LCD driver (3-wire bit-bang interface).
///
/// LCD wiring (4 digits, COM0..COM3, SEG0..SEG7 = LCD pins 5..12):
///   addr(2N)  : bit3=A  bit2=F  bit1=E  bit0=D
///   addr(2N+1): bit3=B  bit2=G  bit1=C  bit0=<next-digit decimal point>
///
/// Decimal points available: 2H at addr1/bit0, 3H at addr3/bit0, 4H at addr5/bit0.
/// Digit 1 has NO decimal point segment in hardware.
////////////////////////////////////////////////////////////////////////////////
/// @attention
///
/// THE EXISTING FIRMWARE IS ONLY FOR REFERENCE, WHICH IS DESIGNED TO PROVIDE
/// CUSTOMERS WITH CODING INFORMATION ABOUT THEIR PRODUCTS SO THEY CAN SAVE
/// TIME. THEREFORE, MINDMOTION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
/// CONSEQUENTIAL DAMAGES ABOUT ANY CLAIMS ARISING OUT OF THE CONTENT OF SUCH
/// HARDWARE AND/OR THE USE OF THE CODING INFORMATION CONTAINED HEREIN IN
/// CONNECTION WITH PRODUCTS MADE BY CUSTOMERS.
///
/// <H2><CENTER>&COPY; COPYRIGHT MINDMOTION </CENTER></H2>
////////////////////////////////////////////////////////////////////////////////

#ifndef __HT1621_H
#define __HT1621_H

#include <stdint.h>
#include "mm32_device.h"
#include "hal_conf.h"

/*============================================================================
 * GPIO Assignments (3-wire bit-bang, all on GPIOB)
 *
 * New pin mapping (hardware re-wired to avoid PB4/PB5 short):
 *   PB3 = DATA   PB5 = WR (unchanged)   PB7 = CS
 * BTN_UP moved to PB9 (was PB7, now freed for CS).
 * VDD: hard-wired to +3.3V on PCB — no MCU control needed.
 *============================================================================*/
#define HT1621_CS_PORT      GPIOB
#define HT1621_CS_PIN       GPIO_Pin_6  /* PB6: chip select (active low) */
#define HT1621_WR_PORT      GPIOB
#define HT1621_WR_PIN       GPIO_Pin_5  /* PB5: write clock */
#define HT1621_DATA_PORT    GPIOB
#define HT1621_DATA_PIN     GPIO_Pin_4  /* PB4: serial data */

/*============================================================================
 * dot_mask bit positions for ht1621_show_text()
 * Note: decimal point after digit 4 is NOT present in the LCD hardware.
 *============================================================================*/
#define HT1621_DP_NONE      0x00u
#define HT1621_DP1          0x01u  /* decimal point after digit 1 */
#define HT1621_DP2          0x02u  /* decimal point after digit 2 */
#define HT1621_DP3          0x04u  /* decimal point after digit 3 */

/*============================================================================
 * Command codes (exposed for step-by-step diagnostic in Thread_Display)
 *============================================================================*/
#define HT1621_CMD_SYS_DIS   0x00u
#define HT1621_CMD_SYS_EN    0x01u
#define HT1621_CMD_LCD_OFF   0x02u
#define HT1621_CMD_LCD_ON    0x03u
#define HT1621_CMD_RC_256K   0x18u
#define HT1621_CMD_BIAS      0x29u  /* 1/3 bias, 1/4 duty (4 commons) */

/*============================================================================
 * Public API
 *============================================================================*/

/** @brief  Configure PB3 (DATA), PB5 (WR), PB7 (CS) as Output PP. */
void ht1621_init(void);

/** @brief  Send init commands (SYS_EN, RC_256K, BIAS, LCD_ON). No RAM clear. */
void ht1621_power_on(void);

/** @brief  Send a single HT1621 command byte. For diagnostic use. */
void send_cmd_pub(uint8_t cmd);

/** @brief  Write 0x0F to all 8 RAM nibbles (all segments ON). Bypasses font mapping. */
void ht1621_all_on(void);

/** @brief  Clear all RAM (all segments OFF). */
void ht1621_clear(void);

/** @brief  Display a 4-character string with optional decimal points. */
void ht1621_show_text(const char *text, uint8_t dot_mask);

/**
 * @brief  Turn off all LCD segments (clear display).
 */
void ht1621_clear(void);

/**
 * @brief  Power-on self-test sequence (busy-wait, no RTOS).
 *         Call from main() after ht1621_init() and before osKernelStart().
 *         Sequence: 8888 → 0000 → 1234 → 5678 → dp2 test → dp3 test → dp4 test
 *                   → C101 → End.1 → C201 → End.2 → L1.
 *         Each frame shown for 800–1000 ms (visual inspection required).
 */
void ht1621_selftest(void);

#endif /* __HT1621_H */
