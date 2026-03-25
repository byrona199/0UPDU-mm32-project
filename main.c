/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "osObjects.h"                      // RTOS object definitions
#include "uart_txrx_interrupt.h"
#include "can.h"
#include "iwdg.h"
#include  <stdio.h>

extern int Init_Thread (void);
void Peripheral_gpio_init(void);

/*
 * main: initialize and start the system
 */
int main (void) {
    // initialize peripherals here
    Peripheral_gpio_init();

    // initialize CMSIS-RTOS
    osKernelInitialize();

    // Initial uart
    UART2_NVIC_Init(115200);
	//UART1_NVIC_Init(115200);

    // Initial CAN BUS
    CAN_NVIC_Init();

    // IWDG reset and start
    IWDG_ResetTest();

    // create 'thread' functions that start executing,
    Init_Thread ();

    // start thread execution
    osKernelStart();
    
    while(1)
    {
        osDelay(1000);
        //printf("Reset dog!\r\n");
        //feed dog,After shielding, reset causes led flicker.
        Write_Iwdg_RL();
    }
}

void Peripheral_gpio_init()
{
    // Meter A  PA1
    // Meter B  PA0
    // Meter C  PB4
    // Meter D  PB5

    // Relay 1  PB1
    // Relay 2  PB0
    // Relay 3  PA7
    // Relay 4  PA6
    // Relay 5  PA5
    // Relay 6  PA4
    // Relay 7  PB2
    // Relay 8  PA8
    // Relay 9  PA11
    // Relay 10 PA12
    // Relay 11 PA15
    // Relay 12 PB3

    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
