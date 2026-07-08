////////////////////////////////////////////////////////////////////////////////
/// @file    buttons.h
/// @author  PDU Project
/// @brief   Button debounce module for BTN_UP (PB7) and BTN_DOWN (PB8).
///          GPIO mode: input with internal pull-up; active-low (pressed = 0).
///          Call btn_tick() every 50 ms from Thread_Display.
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

#ifndef __BUTTONS_H
#define __BUTTONS_H

#include <stdint.h>
#include "mm32_device.h"
#include "hal_conf.h"

/*============================================================================
 * Button Indices
 *============================================================================*/
#define BTN_UP    0u  /* PA10 */
#define BTN_DOWN  1u  /* PA9 */

/*============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief  Configure PA10 (BTN_UP) and PA9 (BTN_DOWN) as input with pull-up.
 *         Call once during peripheral initialisation.
 */
void buttons_init(void);

/**
 * @brief  Update debounce state machines — must be called every 50 ms.
 *         Requires BTN_DEBOUNCE_CNT (5) consecutive samples at the same
 *         level before the debounced state changes (= 250 ms total).
 */
void btn_tick(void);

/**
 * @brief  Returns 1 if the button transitioned to pressed since last call.
 *         Auto-clears on read (single-shot).
 * @param  idx  BTN_UP or BTN_DOWN
 * @return 1 if just pressed, 0 otherwise.
 */
uint8_t btn_just_pressed(uint8_t idx);

/**
 * @brief  Returns current debounced state of button.
 * @param  idx  BTN_UP or BTN_DOWN
 * @return 1 if held down, 0 if released.
 */
uint8_t btn_is_held(uint8_t idx);

#endif /* __BUTTONS_H */
