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
extern int Thread_Health_OK(void);

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

    // create 'thread' functions that start executing,
    Init_Thread ();

    // start thread execution
    osKernelStart();

    // IWDG: 啟用硬體 watchdog；移到 osKernelStart() 之後才呼叫，因為
    // IWDG_ResetTest() 內部會呼叫 osDelay()，必須在 RTOS 排程器啟動、
    // main() 已經是以執行緒身分繼續執行的情況下才能安全使用。
    // 餵狗邏輯見下方 while(1) idle loop，綁定 Thread_Health_OK()，只有
    // CAN_RX/CAN_Connect/UART 三個關鍵執行緒都存活才餵狗，避免「idle loop
    // 自己沒卡住就一直餵」的假保護。
    IWDG_ResetTest();

    while (1) {
        osDelay(300);   /* 需 <= Thread.c 的 HEALTH_CHECK_INTERVAL_MS(300ms) */
        if (Thread_Health_OK()) {
            Write_Iwdg_RL();
        }
        /* 不健康：不餵狗，讓 IWDG 於 ~3.3s 後自動重置 MCU */
    }
}

