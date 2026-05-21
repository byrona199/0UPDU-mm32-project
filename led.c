////////////////////////////////////////////////////////////////////////////////
/// @file     led.c
/// @author   AE TEAM
/// @brief    THIS FILE PROVIDES ALL THE SYSTEM FUNCTIONS.
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
#define _LED_C_

// Files includes
#include "mm32_device.h"
#include "hal_conf.h"

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup MM32_Example_Layer
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @addtogroup LED
/// @{
#define LEDGREEN_ON()     GPIOB->BRR = GPIO_BRR_BR8// PA8
#define LEDGREEN_OFF()    GPIOB->BSRR = GPIO_BSRR_BS8// PA8


////////////////////////////////////////////////////////////////////////////////
/// @addtogroup LED_Exported_Constants
/// @{

////////////////////////////////////////////////////////////////////////////////
/// @brief    LED initialization
/// @param    None
/// @retval   None
////////////////////////////////////////////////////////////////////////////////
void LED_Init(void)
{
    /* PB8 (LED1) is now used as BTN_DOWN — hardware reworked.
     * GPIO configuration is handled by buttons_init().
     * LED_Init() is intentionally a no-op to preserve the call-site interface. */
}

/// @}


/// @}

/// @}

