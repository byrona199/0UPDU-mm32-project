////////////////////////////////////////////////////////////////////////////////
/// @file    pdu_role.c
/// @author  PDU Project
/// @brief   PDU node role persistence in the last Flash page of MM32F0131C4P.
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

#define _PDU_ROLE_C_

#include "pdu_role.h"
#include "mm32_device.h"
#include "hal_conf.h"           /* includes hal_flash.h */

/*============================================================================
 * Constants
 *============================================================================*/
#define PDU_ROLE_MAGIC       0x50445500UL   /* "PDU\0"                       */
#define PDU_ROLE_FLASH_ADDR  0x08003C00UL   /* last 1 KB page of 16 KB Flash */
#define PDU_ROLE_MIN         1u
#define PDU_ROLE_MAX         40u

/*============================================================================
 * Internal: compute CRC word (XOR of magic and role)
 *============================================================================*/
static uint32_t role_crc(uint32_t role)
{
    return PDU_ROLE_MAGIC ^ role;
}

/*============================================================================
 * pdu_role_load
 *============================================================================*/
uint8_t pdu_role_load(void)
{
    const volatile uint32_t *p = (const volatile uint32_t *)PDU_ROLE_FLASH_ADDR;
    uint32_t magic = p[0];
    uint32_t role  = p[1];
    uint32_t crc   = p[2];

    if (magic != PDU_ROLE_MAGIC)           return 0u;
    if (crc   != role_crc(role))           return 0u;
    if (role  <  PDU_ROLE_MIN ||
        role  >  PDU_ROLE_MAX)             return 0u;

    return (uint8_t)role;
}

/*============================================================================
 * pdu_role_save
 *============================================================================*/
int pdu_role_save(uint8_t role)
{
    FLASH_Status st;

    if (role < PDU_ROLE_MIN || role > PDU_ROLE_MAX) return -1;

    __disable_irq();

    FLASH_Unlock();

    /* Erase the target page (all bytes → 0xFF) */
    st = FLASH_ErasePage(PDU_ROLE_FLASH_ADDR);
    if (st != FLASH_COMPLETE) {
        FLASH_Lock();
        __enable_irq();
        return -2;
    }

    /* Program magic */
    st = FLASH_ProgramWord(PDU_ROLE_FLASH_ADDR + 0UL, PDU_ROLE_MAGIC);
    if (st != FLASH_COMPLETE) { FLASH_Lock(); __enable_irq(); return -2; }

    /* Program role */
    st = FLASH_ProgramWord(PDU_ROLE_FLASH_ADDR + 4UL, (uint32_t)role);
    if (st != FLASH_COMPLETE) { FLASH_Lock(); __enable_irq(); return -2; }

    /* Program CRC */
    st = FLASH_ProgramWord(PDU_ROLE_FLASH_ADDR + 8UL, role_crc((uint32_t)role));
    if (st != FLASH_COMPLETE) { FLASH_Lock(); __enable_irq(); return -2; }

    FLASH_Lock();
    __enable_irq();

    return 0;
}
