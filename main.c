/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/

#define osObjectsPublic                     // define objects in main module
#include "osObjects.h"                      // RTOS object definitions
#include "uart_txrx_interrupt.h"
#include "can.h"
#include "iwdg.h"
#include "modbus_rtu.h"
#include "ht1621.h"
#include "buttons.h"
#include "pdu_role.h"
#include  <stdio.h>

extern int Init_Thread (void);

/*
 * main: initialize and start the system
 */
int main (void) {

    // initialize CMSIS-RTOS
    osKernelInitialize();

    // HT1621 LCD: configure GPIO (PB3=DATA, PB5=WR, PB7=CS) before RTOS starts
    ht1621_init();

    // Initial debug uart (UART1, PA1/PA0, 115200)
    //UART1_NVIC_Init(115200);

    // Initial Modbus RTU on UART2 (RS-485, 9600 baud)
    modbus_init();

    // Load PDU role from Flash; set node/bus globals used by all threads
    {
        uint8_t role = pdu_role_load();
        g_my_role = role;
        if (role != 0u) {
            g_my_bus_id  = (uint8_t)((role - 1u) / 20u + 1u);   /* 1..2 */
            g_my_node_id = (uint8_t)((role - 1u) % 20u + 1u);   /* 1..20 */
            // Initial CAN BUS (only when a valid role is configured)
            CAN_NVIC_Init();
        }
    }

    // IWDG: temporarily disabled while debugging boot path
    //IWDG_ResetTest();

    // create 'thread' functions that start executing,
    Init_Thread ();

    // start thread execution
    osKernelStart();

    while (1) {
        osDelay(1000);
        //Write_Iwdg_RL();   /* IWDG disabled */
    }
}

