#include "uart_txrx_interrupt.h"
#include "can.h"

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/

#define MAX_OUTLET 12

// Thread fuction Definition
void Thread_UART_outlet_ctrl(void const *argument);
void Thread_UART_outlet_stat(void const *argument);
void Thread_CAN_outlet_ctrl(void const *argument);
void Thread_CAN_outlet_stat(void const *argument);

// Thread ID Definition
osThreadId tid_UART_outlet_ctrl;
osThreadId tid_UART_outlet_stat;
osThreadId tid_CAN_outlet_ctrl;
osThreadId tid_CAN_outlet_stat;

// Thread Priority, instances, stacksz Definition
osThreadDef(Thread_UART_outlet_ctrl, osPriorityNormal, 1, 1024);
osThreadDef(Thread_UART_outlet_stat, osPriorityNormal, 1, 1280);
osThreadDef(Thread_CAN_outlet_ctrl, osPriorityNormal, 1, 1024);
osThreadDef(Thread_CAN_outlet_stat, osPriorityNormal, 1, 1280);

// Semaphore
osSemaphoreId uart_sem;
osSemaphoreDef(uart_sem);

// Mutex
osMutexId uart_mutex;
osMutexDef(uart_mutex);
osMutexId can_mutex;
osMutexDef(can_mutex);

extern u8 sSendBuf[SENDBUFLENGTH];
extern u8 sRecvBuf[RECVBUFLENGTH];

extern u16 RecvLen;
extern u16 SendLen;

//float currentA = 0;
float voltageVA = 0;
float voltageVB = 0;
float voltageVC = 0;
float voltageVD = 0;
float frequencyHz = 0;
float wattW = 0;
float pf = 0; // 0~1
float kwh = 0;
float o_current[12] = {0};
float o_kw[12] = {0};
float o_pf[12] = {0};
float o_kwh[12] = {0};
u8 outlets_state[12] = {0};                                     // recv main board outlet change state
u8 curr_state[12] = {0};                                        // mm32 record outlet status
u8 relaycheck = 0;

void floatouint8(float val, u8 data[])
{
    memcpy(data, &val, sizeof(float));
}

int Init_Thread(void)
{
    uart_sem = osSemaphoreCreate(osSemaphore(uart_sem), 1);
    uart_mutex = osMutexCreate(osMutex(uart_mutex));
    can_mutex = osMutexCreate(osMutex(can_mutex));
    
    tid_UART_outlet_stat = osThreadCreate(osThread(Thread_UART_outlet_stat), NULL);
    if(!tid_UART_outlet_stat) return -1;
    
    tid_UART_outlet_ctrl = osThreadCreate(osThread(Thread_UART_outlet_ctrl), NULL);
    if(!tid_UART_outlet_ctrl) return -1;

    tid_CAN_outlet_stat = osThreadCreate(osThread(Thread_CAN_outlet_stat), NULL);
    if(!tid_CAN_outlet_stat) return -1;
    
    tid_CAN_outlet_ctrl = osThreadCreate(osThread(Thread_CAN_outlet_ctrl), NULL);
    if(!tid_CAN_outlet_ctrl) return -1;

    return(0);
}

void Thread_UART_outlet_ctrl(void const *argument)
{

}

void Thread_UART_outlet_stat(void const *argument)
{

}

void Thread_CAN_outlet_ctrl(void const *argument)
{

}
void Thread_CAN_outlet_stat(void const *argument)
{

}
