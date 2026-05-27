////////////////////////////////////////////////////////////////////////////////
/// @file    pdu_role.h
/// @author  PDU Project
/// @brief   PDU node role persistence in the last Flash page of MM32F0131C4P.
///
///   Role value 1..40 maps to a CAN node:
///     bus_id    = (role - 1) / 20 + 1       (1 or 2)
///     node_id   = (role - 1) % 20 + 1       (1..20)
///   Role value 0 means "not configured" — CAN must NOT be started.
///
///   Flash page used: 0x0800FC00 (last 1 KB of 64 KB Flash)
///   Layout: magic(4B) + role(4B) + crc(4B)
///   CRC:    magic XOR role
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

#ifndef __PDU_ROLE_H
#define __PDU_ROLE_H

#include <stdint.h>

/*============================================================================
 * Public API
 *============================================================================*/

/**
 * @brief  Load PDU role from Flash.
 * @return Saved role (1..40) on success; 0 if Flash is blank or corrupt.
 */
uint8_t pdu_role_load(void);

/**
 * @brief  Save PDU role to Flash (erase last page, program 3 words).
 *         Interrupts are disabled for the duration of Flash programming.
 *         Caller should invoke NVIC_SystemReset() after a successful save
 *         so that CAN is re-initialised with the new node ID.
 * @param  role  Value to store (1..40).
 * @return  0  success
 *         -1  role out of range
 *         -2  Flash erase or program error
 */
int pdu_role_save(uint8_t role);

#endif /* __PDU_ROLE_H */
