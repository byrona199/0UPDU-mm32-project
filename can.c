////////////////////////////////////////////////////////////////////////////////
/// @file    can.c
/// @author  AE TEAM
/// @brief   CAN Bus driver for MM32 PDU node.
///          Handles CAN GPIO, filter, TX (single + multi-frame), and RX IRQ.
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

#define _CAN_C_

#include "stdio.h"
#include "hal_can.h"
#include "can.h"

CanPeliRxMsg gCanPeliRxMsgBuff;
CanTxMsg     gCanTxMsgBuff;

/*============================================================================
 * Internal: CAN GPIO Setup (PA11=RX, PA12=TX)
 *============================================================================*/
static void CAN_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1ENR_CAN, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* CAN RX - PA11 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* CAN TX - PA12 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_4);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_4);

    NVIC_InitStructure.NVIC_IRQChannel         = CAN_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority  = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd       = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*============================================================================
 * CAN_Config: baud rate + acceptance filter
 *============================================================================*/
void CAN_Config(u32 CAN_Pre, CAN_Mode ID, u32 idCode1, u32 idCode2, u32 mask1, u32 mask2)
{
    CAN_Peli_InitTypeDef       CAN_Peli_InitStructure;
    RCC_ClocksTypeDef          RCC_Clocks;
    u32 idCodeTemp1, idMaskTemp1;
    u32 idCodeTemp2, idMaskTemp2;
    CAN_Peli_FilterInitTypeDef CAN_Peli_FilterInitStructure;

    CAN_ResetMode_Cmd(CAN1, ENABLE);
    CAN_Mode_Cmd(CAN1, CAN_PELIMode);

    RCC_GetClocksFreq(&RCC_Clocks);

    CAN_Peli_StructInit(&CAN_Peli_InitStructure);
    CAN_Peli_FilterStructInit(&CAN_Peli_FilterInitStructure);
    CAN_AutoCfg_BaudParam(&CAN_Peli_InitStructure, RCC_Clocks.PCLK1_Frequency, CAN_Pre);

    CAN_Peli_InitStructure.SAM = RESET;
    CAN_Peli_InitStructure.STM = DISABLE;
    CAN_Peli_InitStructure.LOM = DISABLE;
    CAN_Peli_Init(&CAN_Peli_InitStructure);

    switch (ID) {
        case StandardFrame_SingleFilter:
            idCodeTemp1 = idCode1 << (3 + 18);
            idMaskTemp1 = mask1   << (3 + 18);
            CAN_Peli_FilterInitStructure.AFM = CAN_FilterMode_Singal;
            CAN_Peli_FilterInitStructure.CAN_FilterId0     = (idCodeTemp1 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId1     = (idCodeTemp1 >> 16) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId2     = (idCodeTemp1 >>  8) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId3     = (idCodeTemp1 >>  0) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId0 = (idMaskTemp1 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId1 = ((idMaskTemp1 >> 16) & 0xff) | 0x1f;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId2 = ((idMaskTemp1 >>  8) & 0xff) | 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId3 = ((idMaskTemp1 >>  0) & 0xff) | 0xff;
            break;
        case ExtendedFrame_SingleFilter:
            idCodeTemp1 = idCode1 << 3;
            idMaskTemp1 = mask1   << 3;
            CAN_Peli_FilterInitStructure.AFM = CAN_FilterMode_Singal;
            CAN_Peli_FilterInitStructure.CAN_FilterId0     = (idCodeTemp1 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId1     = (idCodeTemp1 >> 16) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId2     = (idCodeTemp1 >>  8) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId3     = (idCodeTemp1 >>  0) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId0 = (idMaskTemp1 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId1 = (idMaskTemp1 >> 16) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId2 = (idMaskTemp1 >>  8) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId3 = (idMaskTemp1 >>  0) & 0xff;
            break;
        case StandardFrame_DoubleFilter:
            idCodeTemp1 = idCode1 << (3 + 18);
            idMaskTemp1 = mask1   << (3 + 18);
            idCodeTemp2 = idCode2 << (3 + 18);
            idMaskTemp2 = mask2   << (3 + 18);
            CAN_Peli_FilterInitStructure.AFM = CAN_FilterMode_Double;
            CAN_Peli_FilterInitStructure.CAN_FilterId0     = (idCodeTemp1 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId1     = (idCodeTemp1 >> 16) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId2     = (idCodeTemp2 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId3     = (idCodeTemp2 >> 16) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId0 = (idMaskTemp1 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId1 = ((idMaskTemp1 >> 16) & 0xff) | 0x1f;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId2 = (idMaskTemp2 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId3 = ((idMaskTemp2 >> 16) & 0xff) | 0x1f;
            break;
        case ExtendedFrame_DoubleFilter:
            idCodeTemp1 = idCode1 << 3;
            idMaskTemp1 = mask1   << 3;
            idCodeTemp2 = idCode2 << 3;
            idMaskTemp2 = mask2   << 3;
            CAN_Peli_FilterInitStructure.AFM = CAN_FilterMode_Double;
            CAN_Peli_FilterInitStructure.CAN_FilterId0     = (idCodeTemp1 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId1     = (idCodeTemp1 >> 16) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId2     = (idCodeTemp2 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterId3     = (idCodeTemp2 >> 16) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId0 = (idMaskTemp1 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId1 = (idMaskTemp1 >> 16) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId2 = (idMaskTemp2 >> 24) & 0xff;
            CAN_Peli_FilterInitStructure.CAN_FilterMaskId3 = (idMaskTemp2 >> 16) & 0xff;
            break;
        default:
            break;
    }
    CAN_Peli_FilterInit(&CAN_Peli_FilterInitStructure);
    CAN_Peli_ITConfig(CAN_IT_RI, ENABLE);
    CAN_ResetMode_Cmd(CAN1, DISABLE);
}

/*============================================================================
 * Send_CANFrame: transmit a single CAN frame (no artificial delay)
 *============================================================================*/
void Send_CANFrame(CanTxMsg *TxMessage)
{
    CanPeliTxMsg CanPeliTxMsgStructure;
    u32 ID = 0;
    u32 i;

    if (TxMessage->CANIDtype) {
        /* Extended frame */
        ID = TxMessage->CANID << 3;
        CanPeliTxMsgStructure.FF   = 0x01;
        CanPeliTxMsgStructure.IDLL = (ID >>  0) & 0xff;
        CanPeliTxMsgStructure.IDLH = (ID >>  8) & 0xff;
        CanPeliTxMsgStructure.IDHL = (ID >> 16) & 0xff;
        CanPeliTxMsgStructure.IDHH = (ID >> 24) & 0xff;
    } else {
        /* Standard frame */
        ID = TxMessage->CANID << 21;
        CanPeliTxMsgStructure.FF   = 0x00;
        CanPeliTxMsgStructure.IDHL = (ID >> 16) & 0xff;
        CanPeliTxMsgStructure.IDHH = (ID >> 24) & 0xff;
    }

    CanPeliTxMsgStructure.DLC = TxMessage->DLC;
    CanPeliTxMsgStructure.RTR = TxMessage->RTR;

    for (i = 0; i < 8; i++) {
        CanPeliTxMsgStructure.Data[i] = TxMessage->Data[i];
    }

    CAN_Peli_Transmit(&CanPeliTxMsgStructure);
    /* No delay — CAN hardware handles arbitration and TX timing */
}

/*============================================================================
 * CAN_SendSingleFrame: send ≤7 bytes payload with transport header
 *============================================================================*/
void CAN_SendSingleFrame(uint8_t msg_type, const uint8_t *data, uint8_t len)
{
    CanTxMsg tx;
    uint8_t i;

    if (len > CAN_TRANSPORT_PAYLOAD) len = CAN_TRANSPORT_PAYLOAD;

    tx.CANID     = CAN_MAKE_ID(msg_type, MY_NODE_ID);
    tx.CANIDtype = CAN_ID_STD;
    tx.RTR       = CAN_DATA_FRAME;
    tx.DLC       = len + 1;  /* +1 for transport header byte */

    tx.Data[0] = CAN_MAKE_HEADER(CAN_FRAME_SINGLE, 0);
    for (i = 0; i < len; i++) {
        tx.Data[1 + i] = data[i];
    }
    /* Zero-pad remaining bytes */
    for (i = len + 1; i < 8; i++) {
        tx.Data[i] = 0;
    }

    Send_CANFrame(&tx);
}

/*============================================================================
 * CAN_SendMultiFrame: fragment payload into First/Continuation/Last frames
 *============================================================================*/
void CAN_SendMultiFrame(uint8_t msg_type, const uint8_t *data, uint16_t len)
{
    CanTxMsg tx;
    uint16_t offset = 0;
    uint8_t  seq    = 0;
    uint8_t  frame_type;
    uint8_t  chunk;
    uint8_t  i;

    /* Single frame if it fits */
    if (len <= CAN_TRANSPORT_PAYLOAD) {
        CAN_SendSingleFrame(msg_type, data, (uint8_t)len);
        return;
    }

    tx.CANID     = CAN_MAKE_ID(msg_type, MY_NODE_ID);
    tx.CANIDtype = CAN_ID_STD;
    tx.RTR       = CAN_DATA_FRAME;
    tx.DLC       = 8;

    while (offset < len) {
        uint16_t remain = len - offset;

        if (offset == 0) {
            frame_type = CAN_FRAME_FIRST;
        } else if (remain <= CAN_TRANSPORT_PAYLOAD) {
            frame_type = CAN_FRAME_LAST;
        } else {
            frame_type = CAN_FRAME_CONTINUATION;
        }

        chunk = (remain > CAN_TRANSPORT_PAYLOAD) ? CAN_TRANSPORT_PAYLOAD : (uint8_t)remain;

        tx.Data[0] = CAN_MAKE_HEADER(frame_type, seq);
        for (i = 0; i < chunk; i++) {
            tx.Data[1 + i] = data[offset + i];
        }
        /* Zero-pad last frame if needed */
        for (i = chunk + 1; i < 8; i++) {
            tx.Data[i] = 0;
        }
        if (frame_type == CAN_FRAME_LAST) {
            tx.DLC = chunk + 1;
        }

        Send_CANFrame(&tx);

        /* Inter-frame delay: give Linux SocketCAN time to drain the receive FIFO.
         * Without delay, FlexCAN hardware FIFO (6 slots) overflows when
         * OUTLET_METRICS sends 76 back-to-back frames, causing seq gaps. */
        osDelay(4);  /* 4ms: 76 frames → +304ms, total burst ~342ms (<500ms timeout)
                      * Increased from 2ms to reduce FlexCAN FIFO overflow / seq gaps */

        offset += chunk;
        seq = (seq + 1) & CAN_FRAME_SEQ_MASK;
    }
}

/*============================================================================
 * CAN IRQ Handler: receive message and set flag
 *============================================================================*/
void CAN_IRQHandler(void)
{
    u32 CAN_IR_STA;
    u32 CAN_SR_STA;

    CAN_IR_STA = CAN1_PELI->IR;
    CAN_SR_STA = CAN1_PELI->SR;

    if (CAN_IR_STA & CAN_IT_RI) {
        CAN_Peli_Receive(&gCanPeliRxMsgBuff);
        flag = 1;
    }

    /* Bus error — ignore for now (protocol handled by master) */
    if (CAN_IR_STA & CAN_IT_BEI) {
    }

    /* Data overrun — release receive buffer */
    if (CAN_SR_STA & CAN_STATUS_DOS) {
        CAN1_PELI->CMR |= 0x01 << 5;
        CAN1_PELI->CMR |= 0x08;
    }

    /* Arbitration lost — normal in multi-node bus */
    if (CAN_IR_STA & CAN_IT_ALI) {
    }
}

/*============================================================================
 * CAN_NVIC_Init: entry point called from main()
 *
 * Filter setup: Accept all Standard frames addressed to MY_NODE_ID or
 * broadcast (NODE_ID=0). Uses mask filter on NODE_ID bits [4:0].
 *
 * 11-bit CAN ID layout: [MSG_TYPE:10:5][NODE_ID:4:0]
 *
 * Filter mask convention: 1 = don't care, 0 = must match (SJA1000).
 *   mask = 0x7E0 → bits[10:5] (MSG_TYPE) = don't care (accept all types)
 *                   bits[4:0]  (NODE_ID)  = must match exactly
 *
 * NOTE: Uses double filter to accept both MY_NODE_ID and broadcast (0).
 *============================================================================*/
void CAN_NVIC_Init(void)
{
    CAN_GPIO_Config();
    /* 250 kbps, Standard Double Filter:
     * Filter 1: MY_NODE_ID, mask 0x7E0 → accept any MSG_TYPE to MY_NODE_ID
     * Filter 2: broadcast 0x00, mask 0x7E0 → accept any MSG_TYPE to NODE_ID=0
     */
    CAN_Config(CAN_BITRATE,
               StandardFrame_DoubleFilter,
               MY_NODE_ID,     /* idCode1: our node ID */
               0x00,           /* idCode2: broadcast address */
               0x7E0,          /* mask1: MSG_TYPE=don't care, NODE_ID=exact match */
               0x7E0);         /* mask2: MSG_TYPE=don't care, NODE_ID=0 exact match */
}
