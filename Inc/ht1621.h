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
 *============================================================================*/
#define HT1621_VDD_PORT     GPIOB
#define HT1621_VDD_PIN      GPIO_Pin_3  /* PB3: controls HT1621 Vdd power switch */
#define HT1621_CS_PORT      GPIOB
#define HT1621_CS_PIN       GPIO_Pin_6  /* PB6: chip select (active low) */
#define HT1621_WR_PORT      GPIOB
#define HT1621_WR_PIN       GPIO_Pin_5  /* PB5: write clock */
#define HT1621_DATA_PORT    GPIOB
#define HT1621_DATA_PIN     GPIO_Pin_4  /* PB4: serial data */

/*============================================================================
 * dot_mask bit positions for ht1621_show_text()
 * Note: decimal point after digit 1 is NOT present in the LCD hardware.
 *============================================================================*/
#define HT1621_DP_NONE      0x00u
#define HT1621_DP2          0x02u  /* decimal point after digit 2 */
#define HT1621_DP3          0x04u  /* decimal point after digit 3 */
#define HT1621_DP4          0x08u  /* decimal point after digit 4 */

/*============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief  Power on Vdd (PB3 high), configure GPIO, send HT1621 init commands.
 *         Busy-waits ~60 ms for power-on stabilisation — call before RTOS.
 */
void ht1621_init(void);

/**
 * @brief  Display a 4-character string with optional decimal points.
 * @param  text     Pointer to exactly 4 characters. Supported chars:
 *                  '0'-'9', 'A', 'C', 'E', 'L', 'n', 'd', ' ', '-'.
 *                  Unknown chars render as blank.
 * @param  dot_mask Bitmask: HT1621_DP2 | HT1621_DP3 | HT1621_DP4.
 */
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
