/* host 單元測試（qt-ui 改版 spec §6、AC-F1／F2）：
 *   cd mm32-firmware
 *   gcc -Wall -Wextra -Itests/host/stubs -IInc \
 *       -o tests/host/test_outlet_phase \
 *       tests/host/test_outlet_phase.c modbus_meter.c
 *   ./tests/host/test_outlet_phase
 */
#include <assert.h>
#include <stdio.h>
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
    /* 三相：outlet 1–16／17–32／33–48（0-based idx 0–15／16–31／32–47）→ L1／L2／L3 */
    assert(outlet_phase_for(0,  METER_TYPE_THREE) == 0);
    assert(outlet_phase_for(15, METER_TYPE_THREE) == 0);
    assert(outlet_phase_for(16, METER_TYPE_THREE) == 1);
    assert(outlet_phase_for(31, METER_TYPE_THREE) == 1);
    assert(outlet_phase_for(32, METER_TYPE_THREE) == 2);
    assert(outlet_phase_for(47, METER_TYPE_THREE) == 2);
    assert(outlet_phase_for(48, METER_TYPE_THREE) == 0);   /* 越界：保守回 L1 */

    /* 單相與未知：全部 L1（單相分到 L2/L3 會讓 pdu-core 產生 3 筆重複共通告警） */
    for (uint8_t i = 0; i < CAN_MAX_OUTLETS; i++) {
        assert(outlet_phase_for(i, METER_TYPE_SINGLE) == 0);
        assert(outlet_phase_for(i, METER_TYPE_UNKNOWN) == 0);
    }

    printf("test_outlet_phase: ALL PASS\n");
    return 0;
}
