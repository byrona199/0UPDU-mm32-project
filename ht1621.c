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

/*============================================================================
 * HT1621 Command Codes
 *
 * Format: after "100" mode prefix, send 8-bit command then 1 don't-care bit.
 * Bit values chosen so write_bits(cmd, 8) + write_bits(0, 1) produces the
 * correct 9-bit HT1621 command word (see datasheet Table 3).
 *============================================================================*/
#define HT1621_CMD_SYS_DIS   0x00u  /* oscillator off                */
#define HT1621_CMD_SYS_EN    0x02u  /* oscillator on                 */
#define HT1621_CMD_LCD_OFF   0x04u  /* LCD bias off                  */
#define HT1621_CMD_LCD_ON    0x06u  /* LCD bias on                   */
#define HT1621_CMD_RC_256K   0x30u  /* internal RC osc, 256 kHz      */
#define HT1621_CMD_BIAS      0x52u  /* 1/3 bias, 4 commons           */

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

/*============================================================================
 * Internal: map character to font byte
 *============================================================================*/
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
 * Even address (addr = 2N):   bit3=A  bit2=F  bit1=E  bit0=D
 * Odd  address (addr = 2N+1): bit3=B  bit2=G  bit1=C  bit0=dp_next_digit
 *============================================================================*/
static uint8_t make_even_nibble(uint8_t f)
{
    return (uint8_t)(
        (((f >> 7u) & 1u) << 3u) |  /* A */
        (((f >> 2u) & 1u) << 2u) |  /* F */
        (((f >> 3u) & 1u) << 1u) |  /* E */
        (((f >> 4u) & 1u) << 0u)    /* D */
    );
}

static uint8_t make_odd_nibble(uint8_t f, uint8_t dp_next)
{
    return (uint8_t)(
        (((f >> 6u) & 1u) << 3u) |  /* B          */
        (((f >> 1u) & 1u) << 2u) |  /* G          */
        (((f >> 5u) & 1u) << 1u) |  /* C          */
        ((dp_next   & 1u) << 0u)    /* next dp    */
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

/*============================================================================
 * Internal: busy-wait delay (no RTOS — calibrated for 72 MHz system clock)
 *============================================================================*/
static void ht1621_delay_ms(uint32_t ms)
{
    volatile uint32_t n;
    while (ms--) {
        for (n = 18000u; n > 0u; n--) { /* ~1 ms @ 72 MHz, ~4 cycles/iter */ }
    }
}

/*============================================================================
 * Internal: clock one bit into HT1621 (sampled on WR rising edge)
 *============================================================================*/
static void write_bit(uint8_t bit)
{
    WR_LOW();
    if (bit) DATA_HIGH(); else DATA_LOW();
    __NOP(); __NOP(); __NOP(); __NOP();
    WR_HIGH();
    __NOP(); __NOP(); __NOP(); __NOP();
}

/*============================================================================
 * Internal: clock N bits MSB-first into HT1621
 *============================================================================*/
static void write_bits(uint32_t data, uint8_t nbits)
{
    int8_t i;
    for (i = (int8_t)(nbits - 1u); i >= 0; i--) {
        write_bit((uint8_t)((data >> (uint8_t)i) & 1u));
    }
}

/*============================================================================
 * Internal: send HT1621 command
 *   Frame: CS_LOW → "100"(3 bits) + cmd(8 bits) + X(1 don't-care) → CS_HIGH
 *============================================================================*/
static void send_cmd(uint8_t cmd)
{
    CS_LOW();
    write_bits(0x4u, 3u);   /* "100" = command mode  */
    write_bits(cmd,  8u);   /* 8-bit command code    */
    write_bits(0x0u, 1u);   /* 1 don't-care bit      */
    CS_HIGH();
}

/*============================================================================
 * Internal: write up to 8 nibbles starting at HT1621 address `addr`
 *   Frame: CS_LOW → "101"(3) + addr(6) + nibble0(4) … nibbleN(4) → CS_HIGH
 *============================================================================*/
static void write_ram(uint8_t addr, const uint8_t *nibbles, uint8_t count)
{
    uint8_t i;
    CS_LOW();
    write_bits(0x5u, 3u);       /* "101" = write data mode  */
    write_bits(addr, 6u);       /* 6-bit start address      */
    for (i = 0u; i < count; i++) {
        write_bits(nibbles[i], 4u); /* 4-bit nibble, MSB first */
    }
    CS_HIGH();
}

/*============================================================================
 * ht1621_init
 *============================================================================*/
void ht1621_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock (safe to call even if already enabled) */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin   = HT1621_VDD_PIN  |
                                    HT1621_CS_PIN   |
                                    HT1621_WR_PIN   |
                                    HT1621_DATA_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* Idle levels before powering Vdd */
    CS_HIGH();
    WR_HIGH();
    DATA_LOW();

    /* Power on HT1621 Vdd via PB3 and wait for internal oscillator */
    GPIO_SetBits(HT1621_VDD_PORT, HT1621_VDD_PIN);
    ht1621_delay_ms(60u);   /* ≥50 ms required */

    /* Initialisation command sequence */
    send_cmd(HT1621_CMD_RC_256K);   /* select internal RC oscillator */
    send_cmd(HT1621_CMD_SYS_EN);    /* enable system oscillator      */
    send_cmd(HT1621_CMD_BIAS);      /* 1/3 bias, 4 commons           */
    send_cmd(HT1621_CMD_LCD_ON);    /* turn on LCD bias              */

    ht1621_clear();
}

/*============================================================================
 * ht1621_clear
 *============================================================================*/
void ht1621_clear(void)
{
    static const uint8_t zeros[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    write_ram(0u, zeros, 8u);
}

/*============================================================================
 * ht1621_show_text
 *============================================================================*/
void ht1621_show_text(const char *text, uint8_t dot_mask)
{
    uint8_t font[4];
    uint8_t nibbles[8];
    uint8_t dp2 = (dot_mask & HT1621_DP2) ? 1u : 0u;  /* addr1 bit0 = digit2 dp */
    uint8_t dp3 = (dot_mask & HT1621_DP3) ? 1u : 0u;  /* addr3 bit0 = digit3 dp */
    uint8_t dp4 = (dot_mask & HT1621_DP4) ? 1u : 0u;  /* addr5 bit0 = digit4 dp */

    font[0] = char_to_font(text[0]);
    font[1] = char_to_font(text[1]);
    font[2] = char_to_font(text[2]);
    font[3] = char_to_font(text[3]);

    /* Digit 1: addr 0 (even) + addr 1 (odd, carries dp2) */
    nibbles[0] = make_even_nibble(font[0]);
    nibbles[1] = make_odd_nibble(font[0], dp2);

    /* Digit 2: addr 2 (even) + addr 3 (odd, carries dp3) */
    nibbles[2] = make_even_nibble(font[1]);
    nibbles[3] = make_odd_nibble(font[1], dp3);

    /* Digit 3: addr 4 (even) + addr 5 (odd, carries dp4) */
    nibbles[4] = make_even_nibble(font[2]);
    nibbles[5] = make_odd_nibble(font[2], dp4);

    /* Digit 4: addr 6 (even) + addr 7 (odd, no following digit dp) */
    nibbles[6] = make_even_nibble(font[3]);
    nibbles[7] = make_odd_nibble(font[3], 0u);

    write_ram(0u, nibbles, 8u);
}

/*============================================================================
 * ht1621_selftest: visual power-on verification (busy-wait, no RTOS)
 *
 * 目視驗證項目：
 *   [1] "8888" — 全段點亮，確認每個 segment 均正常
 *   [2] "    " — 全段熄滅，確認沒有卡段
 *   [3] "0000" — 驗證 0 字形（A,B,C,D,E,F，無中橫）
 *   [4] "1234" — 驗證數字段碼
 *   [5] "5678" — 驗證數字段碼
 *   [6] " 1.23" (dp2) — 驗證 digit2 後的小數點（dot after D2）
 *   [7] "12.3 " (dp3) — 驗證 digit3 後的小數點（dot after D3）
 *   [8] "123." (dp4)  — 驗證 digit4 後的小數點（dot after D4）
 *   [9]  "C101" — 驗證 C, 1, 0 字母/數字段碼
 *   [10] "End.1" (dp3) — 驗證 E, n, d 字母，以及 digit3 小數點
 *   [11] "C201"
 *   [12] "End.2" (dp3)
 *   [13] "L1  " — 驗證 L 字形
 *   [14] "----" — 最終畫面（僅中橫 G 段），提示等待使用者操作
 *============================================================================*/
void ht1621_selftest(void)
{
    /* [1] All segments on */
    ht1621_show_text("8888", HT1621_DP_NONE);
    ht1621_delay_ms(800u);

    /* [2] All segments off */
    ht1621_show_text("    ", HT1621_DP_NONE);
    ht1621_delay_ms(400u);

    /* [3] 0000 */
    ht1621_show_text("0000", HT1621_DP_NONE);
    ht1621_delay_ms(800u);

    /* [4] 1234 */
    ht1621_show_text("1234", HT1621_DP_NONE);
    ht1621_delay_ms(800u);

    /* [5] 5678 */
    ht1621_show_text("5678", HT1621_DP_NONE);
    ht1621_delay_ms(800u);

    /* [6] decimal point after digit 2: shows " 1.23" */
    ht1621_show_text(" 123", HT1621_DP2);
    ht1621_delay_ms(800u);

    /* [7] decimal point after digit 3: shows "12.3 " */
    ht1621_show_text("123 ", HT1621_DP3);
    ht1621_delay_ms(800u);

    /* [8] decimal point after digit 4: shows "1234." */
    ht1621_show_text("1234", HT1621_DP4);
    ht1621_delay_ms(800u);

    /* [9] C101 — letter C */
    ht1621_show_text("C101", HT1621_DP_NONE);
    ht1621_delay_ms(1000u);

    /* [10] End.1 = E,n,d,1 with dp after digit 3 */
    ht1621_show_text("End1", HT1621_DP3);
    ht1621_delay_ms(1000u);

    /* [11] C201 */
    ht1621_show_text("C201", HT1621_DP_NONE);
    ht1621_delay_ms(1000u);

    /* [12] End.2 */
    ht1621_show_text("End2", HT1621_DP3);
    ht1621_delay_ms(1000u);

    /* [13] L1 phase label */
    ht1621_show_text("L1  ", HT1621_DP_NONE);
    ht1621_delay_ms(800u);

    /* [14] Standby — waiting for commissioning */
    ht1621_show_text("----", HT1621_DP_NONE);
    ht1621_delay_ms(1000u);
}
