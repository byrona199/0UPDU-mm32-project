/* host 單元測試（plan Task 2）：
 *   cd mm32-firmware
 *   gcc -Wall -Wextra -Itests/host/stubs -IInc \
 *       -o tests/host/test_single_meter \
 *       tests/host/test_single_meter.c modbus_meter.c
 *   ./tests/host/test_single_meter
 */
#include <assert.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include "can_protocol.h"
#include "modbus_meter.h"

#ifdef NDEBUG
#error "This test verifies via assert(); it must not be compiled with NDEBUG."
#endif

/* modbus_meter.c 參照 RTU 層；本測試只走純函式路徑，給樁即可連結。 */
int modbus_read_regs(uint8_t slave_id, uint8_t fc, uint16_t start_reg,
                     uint16_t num_regs, uint16_t *out_regs)
{
    (void)slave_id; (void)fc; (void)start_reg; (void)num_regs; (void)out_regs;
    return MB_ERR_TIMEOUT;
}
int modbus_write_single_reg(uint8_t slave_id, uint16_t reg_addr, uint16_t value)
{
    (void)slave_id; (void)reg_addr; (void)value;
    return MB_ERR_TIMEOUT;
}

int main(void)
{
    single_meter_data_t sm;
    single_meter_data_t sm_hilo;
    can_metrics_t       out;
    uint16_t            regs[SINGLE_METER_REG_COUNT];

    /* --- 案例 1：機種判定用正面判準，未知型號不得被猜成任何一種 --- */
    assert(meter_type_from_model(0x0333, 0x0000) == METER_TYPE_THREE);
    assert(meter_type_from_model(0x9000, 0xB131) == METER_TYPE_SINGLE);
    assert(meter_type_from_model(0x9000, 0x0000) == METER_TYPE_SINGLE);  /* 型號2 只是輔助 */
    assert(meter_type_from_model(0x1234, 0x5678) == METER_TYPE_UNKNOWN);
    assert(meter_type_from_model(0x0000, 0x0000) == METER_TYPE_UNKNOWN); /* 全 0 = 讀不到 */

    /* --- 案例 2：單相解析。regs[] 索引 = 位址 - 0x0048 --- */
    memset(regs, 0, sizeof(regs));
    regs[0] = 22000;   /* 0x0048 電壓 220.00 V (/100) */
    regs[1] = 15500;   /* 0x0049 電流 15.500 A (/1000) */
    regs[2] = 3300;    /* 0x004A 有功功率 3300 W */
    regs[3] = 0;       /* 0x004B 電能高位 */
    regs[4] = 32000;   /* 0x004C 電能低位 → 32000/3200 = 10.00 kWh */
    regs[5] = 970;     /* 0x004D PF 0.970 (/1000) */
    regs[9] = 6000;    /* 0x0051 頻率 60.00 Hz (/100) */

    single_meter_parse(regs, &sm);
    assert(sm.voltage   == 22000);
    assert(sm.current   == 15500);
    assert(sm.power     == 3300);
    assert(sm.energy    == 32000u);
    assert(sm.pf        == 970);
    assert(sm.frequency == 6000);

    /* --- 案例 2b：U32 高低字組的組合方向 ---
     * 只測 EP_HI == 0 無法揭露高低字寫反：那會讓電能差 65536 倍而測試全綠。
     * regs[3] 是高位、regs[4] 是低位（與三相 total_meter_read() 的 U32 慣例相同）。
     * 用獨立變數 sm_hilo，避免覆寫案例 2 留給案例 3 使用的 sm（regs 為共用陣列，
     * single_meter_parse() 會清空輸出結構，直接沿用 sm 會讓案例 3 的電壓/電流/
     * 功率/PF 斷言全部改看這裡填的 0）。 */
    memset(regs, 0, sizeof(regs));
    regs[SM_OFF_EP_HI] = 1;      /* 高位 1 */
    regs[SM_OFF_EP_LO] = 5;      /* 低位 5 */
    single_meter_parse(regs, &sm_hilo);
    assert(sm_hilo.energy == 65541u);  /* (1 << 16) | 5；寫反會得到 327681 */

    /* --- 案例 3：換算到 CAN 定點 ---
     * 單相刻度與三相**不同**，四個換算各自不同，錯一個都不會有錯誤訊息： */
    single_meter_to_can(&sm, &out);
    assert(out.voltage == 2200);   /* 220.00V: /100 → CAN ×0.1V  = raw/10  */
    assert(out.current == 1550);   /* 15.500A: /1000 → CAN ×0.01A = raw/10  */
    assert(out.power   == 3300);   /* 3300W:   W    → CAN ×1W     = raw     */
    assert(out.pf      == 97);     /* 0.970:   /1000 → CAN ×0.01  = raw/10  */
    assert(out.energy  == 1000u);  /* 10.00kWh:/3200 → CAN ×0.01  = raw/32  */

    /* --- 案例 4：/3200 的邊界。整數除法無條件捨去，誤差必須 < 1 個 CAN 刻度 --- */
    memset(&sm, 0, sizeof(sm));
    sm.energy = 3199u;             /* 0.999... kWh */
    single_meter_to_can(&sm, &out);
    assert(out.energy == 99u);     /* 3199/32 = 99（0.99 kWh），捨去 < 0.01 kWh */

    /* --- 案例 5：全 0 輸入不得產生非零輸出（離線時的安全性）--- */
    memset(&sm, 0, sizeof(sm));
    single_meter_to_can(&sm, &out);
    assert(out.voltage == 0 && out.current == 0 && out.power == 0);
    assert(out.pf == 0 && out.energy == 0u);

    printf("test_single_meter: ALL PASS\n");
    return 0;
}
