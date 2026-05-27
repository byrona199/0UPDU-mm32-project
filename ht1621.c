////////////////////////////////////////////////////////////////////////////////
/// @file    ht1621.c
/// @author  PDU Project
/// @brief   HT1621 4-digit 7-segment LCD driver (3-wire bit-bang interface).
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

#define _HT1621_C_

#include "ht1621.h"
#include "cmsis_os.h"   /* osDelay() */
#include <stdarg.h>

/* Temporary debug log — shares UART2 output with Thread.c */
#include "dbg_log.h"

/*============================================================================
 * HT1621 Command Codes
 *
 * Format: after "100" mode prefix, send 8-bit command then 1 don't-care bit.
 * Bit values chosen so write_bits(cmd, 8) + write_bits(0, 1) produces the
 * correct 9-bit HT1621 command word (see datasheet Table 3).
 *============================================================================*/
#define HT1621_CMD_SYS_DIS   0x00u  /* oscillator off                */
#define HT1621_CMD_SYS_EN    0x01u  /* oscillator on                 */
#define HT1621_CMD_LCD_OFF   0x02u  /* LCD bias off                  */
#define HT1621_CMD_LCD_ON    0x03u  /* LCD bias on                   */
#define HT1621_CMD_RC_256K   0x18u  /* internal RC osc, 256 kHz      */
/* BIAS cmd byte:  0 0 1 0 d1 d0 X b
 *   d1d0 = duty (00=2COM, 01=3COM, 10=4COM)
 *   b    = bias (0=1/2, 1=1/3)
 *   1/3 bias + 4 COM = 0010 1001 = 0x29                                */
#define HT1621_CMD_BIAS      0x29u  /* 1/3 bias, 1/4 duty (4 commons) */

/*============================================================================
 * 7-Segment Font Encoding
 *
 * One byte per character:
 *   bit7 = A  (top horizontal)      bit6 = B  (top-right vertical)
 *   bit5 = C  (bottom-right vertical) bit4 = D  (bottom horizontal)
 *   bit3 = E  (bottom-left vertical) bit2 = F  (top-left vertical)
 *   bit1 = G  (middle horizontal)   bit0 = unused (decimal point via dot_mask)
 *============================================================================*/
static const uint8_t s_digit_font[10] = {
    0xFCu,  /* '0'  A+B+C+D+E+F     */
    0x60u,  /* '1'  B+C             */
    0xDAu,  /* '2'  A+B+D+E+G       */
    0xF2u,  /* '3'  A+B+C+D+G       */
    0x66u,  /* '4'  B+C+F+G         */
    0xB6u,  /* '5'  A+C+D+F+G       */
    0xBEu,  /* '6'  A+C+D+E+F+G     */
    0xE0u,  /* '7'  A+B+C           */
    0xFEu,  /* '8'  A+B+C+D+E+F+G   */
    0xF6u,  /* '9'  A+B+C+D+F+G     */
};

/**
 * @brief Map an ASCII character to its 7-segment font byte.
 * @param c ASCII character ('0'-'9', 'A','C','E','L','n','d','-',' ').
 * @return Font byte (bit7=A, bit6=B, bit5=C, bit4=D, bit3=E, bit2=F, bit1=G).
 */
static uint8_t char_to_font(char c)
{
    if (c >= '0' && c <= '9') return s_digit_font[(uint8_t)(c - '0')];
    switch (c) {
        case 'A': return 0xEEu;  /* A+B+C+E+F+G  */
        case 'C': return 0x9Cu;  /* A+D+E+F      */
        case 'E': return 0x9Eu;  /* A+D+E+F+G    */
        case 'L': return 0x1Cu;  /* D+E+F        */
        case 'n': return 0x2Au;  /* C+E+G        */
        case 'd': return 0x7Au;  /* B+C+D+E+G    */
        case '-': return 0x02u;  /* G            */
        default:  return 0x00u;  /* blank        */
    }
}

/*============================================================================
 * Internal: compute HT1621 nibbles from font byte
 *
 * Hardware SEG→COM mapping (verified 2026-05-25):
 *   addr 0 (SEG0, digit1 even): bit3=1A  bit2=1F  bit1=1E  bit0=4H(dp4 wrap)
 *   addr 1 (SEG1, digit1 odd):  bit3=1B  bit2=1G  bit1=1C  bit0=1D
 *   addr 2 (SEG2, digit2 even): bit3=2A  bit2=2F  bit1=2E  bit0=1H(dp1)
 *   addr 3 (SEG3, digit2 odd):  bit3=2B  bit2=2G  bit1=2C  bit0=2D
 *   addr 4 (SEG4, digit3 even): bit3=3A  bit2=3F  bit1=3E  bit0=2H(dp2)
 *   addr 5 (SEG5, digit3 odd):  bit3=3B  bit2=3G  bit1=3C  bit0=3D
 *   addr 6 (SEG6, digit4 even): bit3=4A  bit2=4F  bit1=4E  bit0=3H(dp3)
 *   addr 7 (SEG7, digit4 odd):  bit3=4B  bit2=4G  bit1=4C  bit0=4D
 *
 * Note: dp bits are rotated one position right vs the segment digit they
 * sit on the silicon. dp4 wraps back to addr 0 bit0.
 *============================================================================*/

/**
 * @brief Pack A/F/E segments and the decimal-point bit into an even-address nibble.
 * @param f   Font byte (bit7=A, bit2=F, bit3=E).
 * @param dp  Decimal-point bit (1 = on, 0 = off).
 * @return 4-bit nibble [A=b3, F=b2, E=b1, dp=b0].
 */
static uint8_t make_even_nibble(uint8_t f, uint8_t dp)
{
    return (uint8_t)(
        (((f >> 7u) & 1u) << 3u) |  /* A = font bit7 */
        (((f >> 2u) & 1u) << 2u) |  /* F = font bit2 */
        (((f >> 3u) & 1u) << 1u) |  /* E = font bit3 */
        ((dp        & 1u) << 0u)    /* decimal point  */
    );
}

/**
 * @brief Pack B/G/C/D segments into an odd-address nibble.
 * @param f  Font byte (bit6=B, bit1=G, bit5=C, bit4=D).
 * @return 4-bit nibble [B=b3, G=b2, C=b1, D=b0].
 */
static uint8_t make_odd_nibble(uint8_t f)
{
    return (uint8_t)(
        (((f >> 6u) & 1u) << 3u) |  /* B = font bit6 */
        (((f >> 1u) & 1u) << 2u) |  /* G = font bit1 */
        (((f >> 5u) & 1u) << 1u) |  /* C = font bit5 */
        (((f >> 4u) & 1u) << 0u)    /* D = font bit4 */
    );
}

/*============================================================================
 * GPIO convenience macros
 *============================================================================*/
#define CS_HIGH()    GPIO_SetBits  (HT1621_CS_PORT,   HT1621_CS_PIN)
#define CS_LOW()     GPIO_ResetBits(HT1621_CS_PORT,   HT1621_CS_PIN)
#define WR_HIGH()    GPIO_SetBits  (HT1621_WR_PORT,   HT1621_WR_PIN)
#define WR_LOW()     GPIO_ResetBits(HT1621_WR_PORT,   HT1621_WR_PIN)
#define DATA_HIGH()  GPIO_SetBits  (HT1621_DATA_PORT, HT1621_DATA_PIN)
#define DATA_LOW()   GPIO_ResetBits(HT1621_DATA_PORT, HT1621_DATA_PIN)

/**
 * @brief Busy-wait delay (~1 µs per iteration @ 72 MHz).
 * @param us Microseconds to wait.
 */
static void ht1621_delay_us(uint32_t us)
{
    volatile uint32_t n;
    while (us--) {
        for (n = 18u; n > 0u; n--) { }
    }
}

/**
 * @brief Clock one bit into HT1621 on the WR rising edge.
 *
 * HT1621B samples DATA on the WR rising edge.
 * Sequence: WR_LOW → set DATA → WR_HIGH (IC samples here).
 * @param bit Value to transmit (0 or 1).
 */
static void write_bit(uint8_t bit)
{
    WR_LOW();
    if (bit) DATA_HIGH(); else DATA_LOW();
    ht1621_delay_us(5u);   /* t_WL: HT1621 spec ≥400 ns */
    WR_HIGH();             /* rising edge — IC samples DATA here */
    ht1621_delay_us(5u);   /* t_WH: HT1621 spec ≥400 ns */
}

/**
 * @brief Clock N bits MSB-first into HT1621.
 *
 * Used for the 3-bit mode prefix and 6-bit RAM address.
 * @param data  Data word; the top @p nbits bits are transmitted first.
 * @param nbits Number of bits to send.
 */
static void write_bits(uint32_t data, uint8_t nbits)
{
    int8_t i;
    for (i = (int8_t)(nbits - 1u); i >= 0; i--) {
        write_bit((uint8_t)((data >> (uint8_t)i) & 1u));
    }
}

/**
 * @brief Clock N bits LSB-first into HT1621.
 *
 * Used for 4-bit RAM nibbles; HT1621B datasheet specifies D0 is transmitted
 * before D3.
 * @param data  Data word; bit 0 is transmitted first.
 * @param nbits Number of bits to send.
 */
static void write_bits_lsb(uint32_t data, uint8_t nbits)
{
    uint8_t i;
    for (i = 0u; i < nbits; i++) {
        write_bit((uint8_t)((data >> i) & 1u));
    }
}

/**
 * @brief Transmit one command frame to HT1621.
 *
 * Frame: CS_LOW → "100" (3 b, MSB-first) + cmd (8 b, MSB-first)
 *        + don't-care (1 b) → CS_HIGH.
 * @param cmd 8-bit HT1621 command code (e.g. HT1621_CMD_SYS_EN).
 */
static void send_cmd(uint8_t cmd)
{
    CS_LOW();
    write_bits(0x4u, 3u);   /* "100" = command mode  (MSB-first) */
    write_bits(cmd, 8u);    /* 8-bit command code    (MSB-first, D7 first per datasheet) */
    write_bits(0x0u, 1u);   /* 1 don't-care bit                   */
    CS_HIGH();
    dbg_log("[HT1621] send_cmd(0x%02X)\r\n", (unsigned)cmd);
}

/** @brief Public wrapper around send_cmd() for step-by-step diagnostic use.
 * @param cmd 8-bit HT1621 command code.
 */
void send_cmd_pub(uint8_t cmd) { send_cmd(cmd); }

/**
 * @brief Write consecutive nibbles to HT1621 display RAM.
 *
 * Frame: CS_LOW → "101" (3 b, MSB-first) + addr (6 b, MSB-first)
 *        → nibble[0..count-1] (4 b each, LSB-first) → CS_HIGH.
 * @param addr    Starting HT1621 RAM address (0–31).
 * @param nibbles Array of 4-bit nibble values (only low 4 bits are used).
 * @param count   Number of nibbles to write.
 */
static void write_ram(uint8_t addr, const uint8_t *nibbles, uint8_t count)
{
    uint8_t i;
    CS_LOW();
    write_bits(0x5u, 3u);           /* "101" = write data mode (MSB-first) */
    write_bits(addr, 6u);           /* 6-bit start address     (MSB-first) */
    for (i = 0u; i < count; i++) {
        write_bits_lsb(nibbles[i], 4u); /* 4-bit nibble (LSB-first per datasheet) */
    }
    CS_HIGH();
}

/**
 * @brief Configure GPIO pins for the HT1621 3-wire interface.
 *
 * Sets PB3 (DATA), PB5 (WR), PB7 (CS) as output push-pull at idle levels.
 * Does NOT send any HT1621 commands. VDD is hard-wired to 3.3V on PCB.
 * @note Call from main() before osKernelStart().
 *       Use ht1621_power_on() to initialise the IC after the RTOS starts.
 */
void ht1621_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock (safe to call even if already enabled) */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    /* PB3=DATA, PB5=WR, PB7=CS — all Output Push-Pull.
     * VDD is hard-wired to +3.3V on PCB; no MCU control required.         */
    GPIO_InitStructure.GPIO_Pin   = HT1621_CS_PIN   |
                                    HT1621_WR_PIN   |
                                    HT1621_DATA_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Idle levels: CS high (deselect), WR high, DATA low */
    CS_HIGH();
    WR_HIGH();
    DATA_LOW();
}

/**
 * @brief Send the HT1621B initialisation command sequence.
 *
 * Sequence: SYS_EN → RC_256K → BIAS (1/3 bias, 4 COM) → LCD_ON.
 * RAM is not cleared; call ht1621_clear() or ht1621_show_text() afterwards.
 * @note Uses osDelay(). Must be called from a thread context, NOT from
 *       main() before osKernelStart().
 */
void ht1621_power_on(void)
{
    /* Short settle delay (HT1621B needs ≥50 μs after VDD stable before SYS_EN).   *
     * VDD is always on (hard-wired 3.3V), so just ensure idle line levels.          */
    CS_HIGH();
    WR_HIGH();
    DATA_LOW();
    osDelay(5u);   /* settle: ≥50 μs after VDD stable */

    /* Initialisation command sequence (HT1621, not B variant)
     * Order: SYS_EN first, then clock source, bias, LCD on */
    send_cmd(HT1621_CMD_SYS_EN);    /* enable system oscillator      */
    send_cmd(HT1621_CMD_RC_256K);   /* select internal RC oscillator */
    send_cmd(HT1621_CMD_BIAS);      /* 1/3 bias, 4 commons           */
    send_cmd(HT1621_CMD_LCD_ON);    /* turn on LCD bias              */
    /* NOTE: ht1621_clear() deliberately omitted here for diagnostics.
     * Caller is responsible for clearing or writing RAM as needed.  */
}

/** @brief Blank the display by writing zero to all 8 HT1621 RAM nibbles. */
void ht1621_clear(void)
{
    static const uint8_t zeros[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    write_ram(0u, zeros, 8u);
}

/**
 * @brief Light every segment by writing 0x0F to all 8 HT1621 RAM nibbles.
 *
 * Used for visual IC and wiring verification; bypasses the font mapping.
 * Expected result: all 4 digits show full segments including decimal points.
 */
void ht1621_all_on(void)
{
    static const uint8_t ones[8] = {0x0Fu, 0x0Fu, 0x0Fu, 0x0Fu,
                                    0x0Fu, 0x0Fu, 0x0Fu, 0x0Fu};
    write_ram(0u, ones, 8u);
    dbg_log("[HT1621] all_on written (0x0F x8)\r\n");
}

/**
 * @brief Render a 4-character string on the LCD with optional decimal points.
 *
 * @param text     4-character ASCII string. Supported characters: '0'-'9',
 *                 'A','C','E','L','n','d','-',' '. Unknowns display as blank.
 * @param dot_mask Bitfield for decimal points: HT1621_DP1 | HT1621_DP2 |
 *                 HT1621_DP3, or HT1621_DP_NONE. No dp4 on this LCD hardware.
 */
void ht1621_show_text(const char *text, uint8_t dot_mask)
{
    uint8_t font[4];
    uint8_t nibbles[8];
    /* dp slots are rotated one nibble to the right relative to their digit:
     *   dp1 lives in addr2 bit0, dp2 in addr4 bit0, dp3 in addr6 bit0.
     *   addr0 bit0 has no physical dp (no dp4 on this LCD). */
    uint8_t dp1 = (dot_mask & HT1621_DP1) ? 1u : 0u;  /* addr2 bit0 = 1H */
    uint8_t dp2 = (dot_mask & HT1621_DP2) ? 1u : 0u;  /* addr4 bit0 = 2H */
    uint8_t dp3 = (dot_mask & HT1621_DP3) ? 1u : 0u;  /* addr6 bit0 = 3H */

    font[0] = char_to_font(text[0]);
    font[1] = char_to_font(text[1]);
    font[2] = char_to_font(text[2]);
    font[3] = char_to_font(text[3]);

    /* Digit 1: addr0 (SEG0, even, no dp) + addr1 (SEG1, odd) */
    nibbles[0] = make_even_nibble(font[0], 0u);
    nibbles[1] = make_odd_nibble(font[0]);

    /* Digit 2: addr2 (SEG2, even, dp1) + addr3 (SEG3, odd) */
    nibbles[2] = make_even_nibble(font[1], dp1);
    nibbles[3] = make_odd_nibble(font[1]);

    /* Digit 3: addr4 (SEG4, even, dp2) + addr5 (SEG5, odd) */
    nibbles[4] = make_even_nibble(font[2], dp2);
    nibbles[5] = make_odd_nibble(font[2]);

    /* Digit 4: addr6 (SEG6, even, dp3) + addr7 (SEG7, odd) */
    nibbles[6] = make_even_nibble(font[3], dp3);
    nibbles[7] = make_odd_nibble(font[3]);

    write_ram(0u, nibbles, 8u);
    dbg_log("[HT1621] show_text('%c%c%c%c') nibbles:"
            " %X %X %X %X %X %X %X %X\r\n",
            text[0], text[1], text[2], text[3],
            (unsigned)nibbles[0], (unsigned)nibbles[1],
            (unsigned)nibbles[2], (unsigned)nibbles[3],
            (unsigned)nibbles[4], (unsigned)nibbles[5],
            (unsigned)nibbles[6], (unsigned)nibbles[7]);
}

/**
 * @brief Visual self-test sequence for factory / bring-up verification.
 *
 * Cycles through 14 test frames, each held for 5 s:
 *   [1]  "8888"       — all segments on (full segment check)
 *   [2]  "    "       — all segments off (stuck-segment check)
 *   [3]  "0000"       — digit '0' shape (A,B,C,D,E,F, no G)
 *   [4]  "1234"       — digits 1–4 segment code
 *   [5]  "5678"       — digits 5–8 segment code
 *   [6]  "1.234" (dp1) — decimal point after digit 1
 *   [7]  " 1.23" (dp2) — decimal point after digit 2
 *   [8]  "12.3 " (dp3) — decimal point after digit 3
 *   [9]  "C101"       — letter C, digits 1/0
 *   [10] "End.1" (dp3) — letters E,n,d with dp3
 *   [11] "C201"
 *   [12] "End.2" (dp3)
 *   [13] "L1  "       — letter L shape
 *   [14] "----"       — G-segment only (standby indicator)
 *
 * @note Uses osDelay(); must be called from a thread context.
 */
void ht1621_selftest(void)
{
    /* [1] All segments on */
    ht1621_show_text("8888", HT1621_DP_NONE);
    osDelay(5000u);

    /* [2] All segments off */
    ht1621_show_text("    ", HT1621_DP_NONE);
    osDelay(5000u);

    /* [3] 0000 */
    ht1621_show_text("0000", HT1621_DP_NONE);
    osDelay(5000u);

    /* [4] 1234 */
    ht1621_show_text("1234", HT1621_DP_NONE);
    osDelay(5000u);

    /* [5] 5678 */
    ht1621_show_text("5678", HT1621_DP_NONE);
    osDelay(5000u);

    /* [6] decimal point after digit 1: shows "1.234" */
    ht1621_show_text("1234", HT1621_DP1);
    osDelay(5000u);

    /* [7] decimal point after digit 2: shows " 1.23" */
    ht1621_show_text(" 123", HT1621_DP2);
    osDelay(5000u);

    /* [8] decimal point after digit 3: shows "12.3 " */
    ht1621_show_text("123 ", HT1621_DP3);
    osDelay(5000u);

    /* [9] C101 — letter C */
    ht1621_show_text("C101", HT1621_DP_NONE);
    osDelay(5000u);

    /* [10] End.1 = E,n,d,1 with dp after digit 3 */
    ht1621_show_text("End1", HT1621_DP3);
    osDelay(5000u);

    /* [11] C201 */
    ht1621_show_text("C201", HT1621_DP_NONE);
    osDelay(5000u);

    /* [12] End.2 */
    ht1621_show_text("End2", HT1621_DP3);
    osDelay(5000u);

    /* [13] L1 phase label */
    ht1621_show_text("L1  ", HT1621_DP_NONE);
    osDelay(5000u);

    /* [14] Standby — waiting for commissioning */
    ht1621_show_text("----", HT1621_DP_NONE);
    osDelay(5000u);
}
