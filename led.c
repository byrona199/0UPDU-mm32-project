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
/*
    // PA8
    //RCC->AHBENR |= RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB;                         //enable GPIOA,B clock
    RCC->AHBENR |= RCC_AHBENR_GPIOA;                         //enable GPIOA clock

    GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_8_Pos);
    GPIOA->CRH |= GPIO_CNF_MODE_OUT_PP << GPIO_CRH_CNF_MODE_8_Pos;
    GPIOA->ODR |= GPIO_ODR_ODR8;                                               //PA8  output high
*/

    // PB8 LED1
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    
    /*//set PB3,PB4,PB5 as push-pull output
    GPIOB->CRL &= ~((GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_3_Pos) | \
                    (GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_4_Pos) | \
                    (GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_5_Pos) );
    GPIOB->CRL |=  ((GPIO_CNF_MODE_OUT_PP << GPIO_CRL_CNF_MODE_3_Pos) | \
                    (GPIO_CNF_MODE_OUT_PP << GPIO_CRL_CNF_MODE_4_Pos) | \
                    (GPIO_CNF_MODE_OUT_PP << GPIO_CRL_CNF_MODE_5_Pos) );

    GPIOB->ODR |= GPIO_ODR_ODR3 | GPIO_ODR_ODR4 | GPIO_ODR_ODR5;                //PB3,PB4,PB5 output high*/

    LEDGREEN_OFF();
    //LED2_OFF();
    //LED3_OFF();
    //LED4_OFF();
}

/// @}


/// @}

/// @}

