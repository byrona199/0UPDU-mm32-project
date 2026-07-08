////////////////////////////////////////////////////////////////////////////////
/// @file    buttons.c
/// @author  PDU Project
/// @brief   Button debounce for BTN_UP (PB7) and BTN_DOWN (PB8).
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

#define _BUTTONS_C_

#include "buttons.h"

/*============================================================================
 * Constants
 *============================================================================*/
#define BTN_COUNT        2u
#define BTN_DEBOUNCE_CNT 5u  /* 5 × 50 ms tick = 250 ms debounce window */

/*============================================================================
 * State (all module-private)
 *============================================================================*/
static const uint16_t s_pin[BTN_COUNT]  = { GPIO_Pin_10, GPIO_Pin_9 };

static uint8_t s_cnt_pressed[BTN_COUNT];   /* consecutive pressed samples   */
static uint8_t s_cnt_released[BTN_COUNT];  /* consecutive released samples  */
static uint8_t s_state[BTN_COUNT];         /* 0=released, 1=pressed         */
static uint8_t s_just_pressed[BTN_COUNT];  /* single-shot press event flag  */

/*============================================================================
 * buttons_init
 *============================================================================*/
void buttons_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  /* internal pull-up */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*============================================================================
 * btn_tick: call every 50 ms
 *============================================================================*/
void btn_tick(void)
{
    uint8_t i;

    for (i = 0u; i < BTN_COUNT; i++) {
        /* active-low: GPIO reads Bit_RESET (0) when button is pressed */
        uint8_t raw = (GPIO_ReadInputDataBit(GPIOA, s_pin[i]) == Bit_RESET) ? 1u : 0u;

        if (raw) {
            /* Button physically pressed */
            if (s_cnt_pressed[i] < BTN_DEBOUNCE_CNT) {
                s_cnt_pressed[i]++;
            }
            s_cnt_released[i] = 0u;
            if ((s_cnt_pressed[i] >= BTN_DEBOUNCE_CNT) && !s_state[i]) {
                s_state[i]        = 1u;
                s_just_pressed[i] = 1u;
            }
        } else {
            /* Button physically released */
            if (s_cnt_released[i] < BTN_DEBOUNCE_CNT) {
                s_cnt_released[i]++;
            }
            s_cnt_pressed[i] = 0u;
            if (s_cnt_released[i] >= BTN_DEBOUNCE_CNT) {
                s_state[i] = 0u;
            }
        }
    }
}

/*============================================================================
 * btn_just_pressed: single-shot press event (auto-clears on read)
 *============================================================================*/
uint8_t btn_just_pressed(uint8_t idx)
{
    uint8_t v;
    if (idx >= BTN_COUNT) return 0u;
    v = s_just_pressed[idx];
    s_just_pressed[idx] = 0u;
    return v;
}

/*============================================================================
 * btn_is_held: current debounced state
 *============================================================================*/
uint8_t btn_is_held(uint8_t idx)
{
    if (idx >= BTN_COUNT) return 0u;
    return s_state[idx];
}
