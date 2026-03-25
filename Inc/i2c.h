////////////////////////////////////////////////////////////////////////////////
/// @file    i2c.h
/// @author  AE TEAM
/// @brief   THIS FILE PROVIDES ALL THE SYSTEM FIRMWARE FUNCTIONS.
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


// Define to prevent recursive inclusion
#ifndef __I2C_H
#define __I2C_H

// Files includes
#include "cmsis_os.h"                                           // CMSIS RTOS header file
#include "mm32_device.h"
#include "hal_device.h"
#include "hal_conf.h"
#include "stdio.h"
#include "string.h"


////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Example_Layer
/// @brief MM32 Example Layer
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_RESOURCE
/// @brief MM32 Examples resource modules
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Constants
/// @{

//The size of each EEPROM page
//#define PAGESIZE 16
//Device address of EEPROM
//#define EEPROM_ADDR 0xA8
//#define   EMINIBOARD
#define   MM32F0131C4Q
#if defined(MINIBOARD)
#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SCL_PIN                     GPIO_Pin_8
#define I2C1_SCL_PORT                    GPIOB
#define I2C1_SCL_AFSOURCE                GPIO_PinSource8
#define I2C1_SCL_AFMODE                  GPIO_AF_1

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SDA_PIN                     GPIO_Pin_9
#define I2C1_SDA_PORT                    GPIOB
#define I2C1_SDA_AFSOURCE                GPIO_PinSource9
#define I2C1_SDA_AFMODE                  GPIO_AF_1
#endif
#if defined(EMINIBOARD)
#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SCL_PIN                     GPIO_Pin_6
#define I2C1_SCL_PORT                    GPIOB
#define I2C1_SCL_AFSOURCE                GPIO_PinSource6
#define I2C1_SCL_AFMODE                  GPIO_AF_1

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOB
#define I2C1_SDA_PIN                     GPIO_Pin_7
#define I2C1_SDA_PORT                    GPIOB
#define I2C1_SDA_AFSOURCE                GPIO_PinSource7
#define I2C1_SDA_AFMODE                  GPIO_AF_1
#endif
#if defined(MM32F0131C4Q)
#define I2C1_SCL_BUSCLK                  RCC_AHBPeriph_GPIOA
#define I2C1_SCL_PIN                     GPIO_Pin_9
#define I2C1_SCL_PORT                    GPIOA
#define I2C1_SCL_AFSOURCE                GPIO_PinSource9
#define I2C1_SCL_AFMODE                  GPIO_AF_4

#define I2C1_SDA_BUSCLK                  RCC_AHBPeriph_GPIOA
#define I2C1_SDA_PIN                     GPIO_Pin_10
#define I2C1_SDA_PORT                    GPIOA
#define I2C1_SDA_AFSOURCE                GPIO_PinSource10
#define I2C1_SDA_AFMODE                  GPIO_AF_4
#endif

#define VOLTA       11
#define VOLTB       12
#define VOLTC       13
#define VOLTD       14
#define CURRA       21
#define CURRB       22
#define CURRC       23
#define CURRD       24
#define WATTA       31
#define WATTB       32
#define WATTC       33
#define WATTD       34
#define PFA         41
#define PFB         42
#define PFC         43
#define PFD         44
#define KWHA        51
#define KWHB        52
#define KWHC        53
#define KWHD        54
#define HZ          6
#define TRIPA       7
#define TRIPB       71
#define TRIPC       72
#define RELAYTEST   8


// Voltage
#define VRMS_ADDRH          0x30
#define VRMS_ADDRL          0x78
// Current
#define IRMSA_ADDRH         0x30
#define IRMSA_ADDRL         0x84
#define IRMSB_ADDRH         0x30
#define IRMSB_ADDRL         0x8A
#define IRMSC_ADDRH         0x30
#define IRMSC_ADDRL         0xAE
// Watt
#define ACTIVEPOWERA_ADDRH  0x30
#define ACTIVEPOWERA_ADDRL  0x90
#define ACTIVEPOWERB_ADDRH  0x30
#define ACTIVEPOWERB_ADDRL  0x96
#define ACTIVEPOWERC_ADDRH  0x30
#define ACTIVEPOWERC_ADDRL  0xB4
// KWH
#define CF_COUNTA_ADDRH     0x31
#define CF_COUNTA_ADDRL     0x2C
#define CF_COUNTB_ADDRH     0x31
#define CF_COUNTB_ADDRL     0x32
#define CF_COUNTC_ADDRH     0x31
#define CF_COUNTC_ADDRL     0x50
// Hz
// ZccCnt
#define ZCCCNT_ADDRH        0x30
#define ZCCCNT_ADDRL        0x18
// ZccStart
#define ZCCSTART_ADDRH      0x30
#define ZCCSTART_ADDRL      0x1E
// ZccStop
#define ZCCSTOP_ADDRH       0x30
#define ZCCSTOP_ADDRL       0x24
// SampleCnt
#define SAMPLECNT_ADDRH     0x38
#define SAMPLECNT_ADDRL     0x09
// Relay
#define RELAY_ADDRH         0x38
#define RELAY_ADDRL         0x03
// TRIP
#define TRIP_ADDRH          0x39
#define TRIP_ADDRL          0x16

/// @}

////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Enumeration
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @brief XXXX enumerate definition.
/// @anchor XXXX
////////////////////////////////////////////////////////////////////////////////


/// @}

////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Variables
/// @{
#ifdef _I2C_C_
#define GLOBAL

#else
#define GLOBAL extern

#endif
#undef GLOBAL

/// @}


////////////////////////////////////////////////////////////////////////////////
/// @defgroup MM32_Exported_Functions
/// @{
#define I2CBUFFLEN 16
extern u8 I2CTxBuff[I2CBUFFLEN];
extern u8 I2CRxBuff[I2CBUFFLEN];

static void I2C1_GPIO_Config(void);
static void I2C1_NVIC_Init(void);
static u8 ReadByte(I2C_TypeDef* I2Cx);
//static void EEPROM_WriteByte(I2C_TypeDef* I2Cx, u8 data);
u8 SendByte(u8 dat);
u8 SendCommand(u8 len, u8 CmdH, u8 CmdL);
//static void EEPROM_ReadBuff(I2C_TypeDef* I2Cx, u16 mem_byte_addr, u16 rx_count, u8* rx_data );
//static u8 EEPROM_WriteBuff(u8 sub, u8* ptr, u16 cnt);
void I2C_MasterModeInit(I2C_TypeDef* I2Cx, u32 uiI2C_speed);
void I2C_SetDeviceAddr(I2C_TypeDef* I2Cx, u8 deviceaddr);
void I2C_Meter_Init(void);
void I2C_MeterData(void);
void I2C_EmptyTest(void);
void SwitchRelay(u8 *data);
extern u8 relaycheck;

//extern float currentA;
extern float voltageVA;
extern float voltageVB;
extern float voltageVC;
extern float voltageVD;
extern float frequencyHz;
extern float wattW;
extern float pf; // 0~1
extern float kwh;
extern float o_current[12];
extern float o_kw[12];
extern float o_pf[12];
extern float o_kwh[12];
extern u8 outlets_state[12];
extern u8 curr_state[12];
//extern u8 trip[9];
//extern u8 trip_flag;

// test
//#define BUFFLEN 5
//extern volatile u8 gTxBuff[BUFFLEN];
//extern volatile u8 gRxBuff[BUFFLEN];

/// @}


/// @}

/// @}


////////////////////////////////////////////////////////////////////////////////
#endif
////////////////////////////////////////////////////////////////////////////////
