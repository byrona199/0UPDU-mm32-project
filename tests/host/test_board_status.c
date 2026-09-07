/* host 單元測試（spec §5.2）：
 *   gcc -Wall -Wextra -Itests/host/stubs -IInc \
 *       -o tests/host/test_board_status \
 *       tests/host/test_board_status.c modbus_meter.c
 *   ./tests/host/test_board_status
 */
#include <assert.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include "can_protocol.h"
#include "modbus_meter.h"

/* modbus_meter.c 會參照 RTU 層；本測試的路徑不會呼叫到，給樁即可連結。 */
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
    /* --- 案例 1：線上佈局（與 pdu-model 的 test_can_protocol.c 同一組契約）--- */
    assert(sizeof(can_board_status_t) == 3);
    assert(CAN_BOARD_STATUS_DATA_SIZE == 36);
    assert(CAN_BOARD_STATUS_FRAME_COUNT == 6);
    assert(offsetof(can_board_status_t, frequency) == 0);
    assert(offsetof(can_board_status_t, status)    == 2);

    /* --- 案例 2：板數一致。board_idx = outlet_idx / 4 的換算完全依賴這條 --- */
    assert(CAN_MAX_METER_BOARDS * METER_CHANNELS_PER_BOARD == CAN_MAX_OUTLETS);
    assert(CAN_MAX_METER_BOARDS == METER_BOARD_COUNT);

    can_board_status_t bs;
    meter_board_t      board;

    memset(&board, 0, sizeof(board));

    /* --- 案例 3：讀取成功 → 頻率 1:1 搬值（不換算），online 立起 --- */
    memset(&bs, 0, sizeof(bs));
    board.frequency = 6000u;                  /* 60.00 Hz（×0.01Hz，與 CAN 同刻度）*/
    meter_board_status_update(&bs, &board, MB_OK);
    assert(bs.frequency == 6000u);
    assert(bs.status == CAN_BOARD_ONLINE);    /* reserved bits 必須為 0 */

    board.frequency = 5000u;                  /* 50.00 Hz */
    meter_board_status_update(&bs, &board, MB_OK);
    assert(bs.frequency == 5000u);
    assert(bs.status == CAN_BOARD_ONLINE);

    /* --- 案例 4：讀取失敗 → 只清 online，frequency 保留最後已知值 --- */
    /*     spec §2.3 不變式：清零會讓 host 端該板 4 埠的既有告警看起來像已恢復。 */
    meter_board_status_update(&bs, &board, MB_ERR_TIMEOUT);
    assert(bs.frequency == 5000u);            /* 保留，不是 0 */
    assert((bs.status & CAN_BOARD_ONLINE) == 0);

    /* --- 案例 5：不同失敗碼行為一致 --- */
    meter_board_status_update(&bs, &board, MB_ERR_CRC);
    assert(bs.frequency == 5000u);
    assert((bs.status & CAN_BOARD_ONLINE) == 0);

    /* --- 案例 6：恢復 → 再次成功即回到 online 並更新頻率 --- */
    board.frequency = 5998u;
    meter_board_status_update(&bs, &board, MB_OK);
    assert(bs.frequency == 5998u);
    assert(bs.status == CAN_BOARD_ONLINE);

    /* --- 案例 7：開機初值（static 全 0）即「離線且無資料」，不需額外初始化 --- */
    memset(&bs, 0, sizeof(bs));
    assert((bs.status & CAN_BOARD_ONLINE) == 0);
    assert(bs.frequency == 0u);

    /* --- 案例 8：reserved bits 必須被清乾淨（成功路徑用指派而非 |=）--- */
    memset(&bs, 0xFF, sizeof(bs));            /* 先髒化 */
    meter_board_status_update(&bs, &board, MB_OK);
    assert(bs.status == CAN_BOARD_ONLINE);    /* 高位元被清掉 */

    /* --- 案例 9：12 板陣列連續無 padding：memcpy 到 burst 緩衝區的正確性前提 --- */
    {
        can_board_status_t arr[CAN_MAX_METER_BOARDS];
        assert(sizeof(arr) == CAN_BOARD_STATUS_DATA_SIZE);
        assert((size_t)((const unsigned char *)&arr[11] - (const unsigned char *)arr)
               == 11u * 3u);
    }

    printf("test_board_status: all assertions passed\n");
    return 0;
}
