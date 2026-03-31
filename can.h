////////////////////////////////////////////////////////////////////////////////
/// @file    can.h
/// @author  AE TEAM
/// @brief   CAN Bus driver header for MM32 PDU node.
///          Uses the shared can_protocol.h for protocol definitions.
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

#ifndef __CAN_H
#define __CAN_H

#include <string.h>
#include "cmsis_os.h"
#include "mm32_device.h"
#include "hal_conf.h"

/*============================================================================
 * Shared protocol definitions (local copy — keep in sync with meta-pdu's
 * recipes-pdu-model/pdu-model/files/include/pdu/can_protocol.h)
 *============================================================================*/
#define __KEIL__
#include "can_protocol.h"

/*============================================================================
 * Node Configuration
 *
 * NODE_ID must be unique per CAN bus (1-20).
 * Set via compile-time define or GPIO strapping.
 * TODO: Read MY_NODE_ID from DIP-switch / GPIO strap pins at boot.
 *       Replace compile-time default once hardware is ready.
 *============================================================================*/
#ifndef MY_NODE_ID
#define MY_NODE_ID  1   /* Override with -DMY_NODE_ID=N in Keil project */
#endif

/*============================================================================
 * CAN Frame Type Constants (MM32 HAL)
 *============================================================================*/
#define CAN_ID_STD              0
#define CAN_ID_EXT              1
#define CAN_DATA_FRAME          0
#define CAN_REMOTE_FRAME        1

/*============================================================================
 * CAN Filter Mode
 *============================================================================*/
typedef enum {
    StandardFrame_SingleFilter = 0,
    ExtendedFrame_SingleFilter = 1,
    StandardFrame_DoubleFilter = 2,
    ExtendedFrame_DoubleFilter = 3,
} CAN_Mode;

/*============================================================================
 * TX/RX Message Structures
 *============================================================================*/
typedef struct {
    u32 CANID;
    u32 CANIDtype;
    u8  RTR;
    u8  DLC;
    u8  Data[8];
} CanTxMsg;

typedef struct {
    u32 ID;
    u8  FF;
    u8  RTR;
    u8  DLC;
    u8  Data[8];
} gCanPeliRxMsgType;

/*============================================================================
 * Global Variables
 *============================================================================*/
#ifdef _CAN_C_
#define CAN_GLOBAL
CAN_GLOBAL u8 flag = 0;
#else
#define CAN_GLOBAL extern
CAN_GLOBAL u8 flag;
#endif

CAN_GLOBAL CanPeliRxMsg gCanPeliRxMsgBuff;
CAN_GLOBAL CanTxMsg     gCanTxMsgBuff;

#undef CAN_GLOBAL

/*============================================================================
 * Function Prototypes
 *============================================================================*/

/** @brief Initialize CAN peripheral, GPIO, filter, and NVIC */
void CAN_NVIC_Init(void);

/** @brief Send a single CAN frame */
void Send_CANFrame(CanTxMsg *TxMessage);

/** @brief Send a multi-frame transport message
 *  @param msg_type  CAN_MSG_* message type constant
 *  @param data      Pointer to the payload buffer
 *  @param len       Payload length in bytes
 */
void CAN_SendMultiFrame(uint8_t msg_type, const uint8_t *data, uint16_t len);

/** @brief Send a single-frame transport message (≤7 bytes payload)
 *  @param msg_type  CAN_MSG_* message type constant
 *  @param data      Pointer to the payload buffer
 *  @param len       Payload length (must be ≤ 7)
 */
void CAN_SendSingleFrame(uint8_t msg_type, const uint8_t *data, uint8_t len);

#endif /* __CAN_H */
