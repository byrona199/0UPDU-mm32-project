/**
 * @file modbus_meter.h
 * @brief 1084 metering board abstraction layer.
 *
 * Each board has 4 outlet channels.
 * Up to 12 boards (slave ID 2-13) = 48 outlets max.
 *
 * Register map (FC04, address 0x0000-0x0025):
 *   0x0000  Device model
 *   0x0001  Hardware version    (×0.01)
 *   0x0002  Software version    (×0.01)
 *   0x0003  Sub-model
 *   0x0004  RS485 address
 *   0x0005  Target state        (bit0-3 = ch1-4, 1=ON/開啟, 0=OFF/關閉)
 *   0x0006  Relay state         (bit0-3 = ch1-4, 1=ON/開啟, 0=OFF/關閉)
 *   0x0007  Feedback state      (bit0-3 = ch1-4, 1=ON/開啟, 0=OFF/關閉)
 *   0x0008  Fault state         (bit0-3 = ch1-4, 1=fault)
 *   0x0009  Frequency           (×0.01Hz)
 *   0x000A  Ch1 voltage         (×0.01V)
 *   0x000B  Ch1 current         (×0.001A)
 *   0x000C  Ch1 power           (×0.1W)
 *   0x000D  Ch1 PF              (×0.0001)
 *   0x000E-0x000F  Ch1 energy   (U32, ×0.0001kWh)
 *   0x0010-0x0015  Ch2 (same layout)
 *   0x0016-0x001B  Ch3 (same layout)
 *   0x001C-0x0021  Ch4 (same layout)
 *   0x0022  Ch1 delay time      (1s, 0-6)
 *   0x0023  Ch2 delay time
 *   0x0024  Ch3 delay time
 *   0x0025  Ch4 delay time
 *
 * Relay control (FC06):
 *   Address 1: Ch1 relay  (0xFF00=ON, 0x0000=OFF)
 *   Address 2: Ch2 relay
 *   Address 3: Ch3 relay
 *   Address 4: Ch4 relay
 */

#ifndef __MODBUS_METER_H
#define __MODBUS_METER_H

#include "modbus_rtu.h"
#include "can_protocol.h"
#include <stdint.h>

/*============================================================================
 * Board Configuration
 *============================================================================*/

#define METER_BOARD_COUNT       12      /* Max boards on RS-485 bus */
#define METER_CHANNELS_PER_BOARD 4     /* Outlets per board */
#define METER_SLAVE_ID_BASE     2      /* First board slave ID */

#define METER_TOTAL_OUTLETS     (METER_BOARD_COUNT * METER_CHANNELS_PER_BOARD) /* 48 */

/*============================================================================
 * FC04 Register Addresses
 *============================================================================*/

#define METER_REG_DEVICE_MODEL  0x0000
#define METER_REG_HW_VERSION    0x0001
#define METER_REG_SW_VERSION    0x0002
#define METER_REG_SUB_MODEL     0x0003
#define METER_REG_RS485_ADDR    0x0004
#define METER_REG_TARGET_STATE  0x0005
#define METER_REG_RELAY_STATE   0x0006
#define METER_REG_FEEDBACK_STATE 0x0007
#define METER_REG_FAULT_STATE   0x0008
#define METER_REG_FREQUENCY     0x0009

/* Per-channel register offsets (6 registers per channel, but energy is U32=2 regs) */
#define METER_REG_CH1_VOLTAGE   0x000A
#define METER_REG_CH1_CURRENT   0x000B
#define METER_REG_CH1_POWER     0x000C
#define METER_REG_CH1_PF        0x000D
#define METER_REG_CH1_ENERGY    0x000E  /* U32: 0x000E (high) + 0x000F (low) */

#define METER_REG_CH2_VOLTAGE   0x0010
#define METER_REG_CH3_VOLTAGE   0x0016
#define METER_REG_CH4_VOLTAGE   0x001C

#define METER_REG_CH1_DELAY     0x0022
#define METER_REG_CH2_DELAY     0x0023
#define METER_REG_CH3_DELAY     0x0024
#define METER_REG_CH4_DELAY     0x0025

/* Total registers to read in one FC04 request */
#define METER_REG_READ_COUNT    38      /* 0x0000 to 0x0025 = 38 registers */

/*============================================================================
 * FC06 Relay Control Addresses
 *============================================================================*/

#define METER_RELAY_CH1_ADDR    0x0001
#define METER_RELAY_CH2_ADDR    0x0002
#define METER_RELAY_CH3_ADDR    0x0003
#define METER_RELAY_CH4_ADDR    0x0004

#define METER_RELAY_ON          0xFF00
#define METER_RELAY_OFF         0x0000

/*============================================================================
 * Per-Channel Data (after parsing)
 *============================================================================*/

typedef struct {
    uint16_t voltage;       /* raw ×0.01V from 1084 */
    uint16_t current;       /* raw ×0.001A from 1084 */
    uint16_t power;         /* raw ×0.1W from 1084 */
    uint16_t pf;            /* raw ×0.0001 from 1084 */
    uint32_t energy;        /* raw ×0.0001kWh from 1084 */
} meter_channel_t;

/*============================================================================
 * Per-Board Data (all 4 channels + board info)
 *============================================================================*/

typedef struct {
    uint16_t device_model;
    uint16_t hw_version;
    uint16_t sw_version;
    uint16_t rs485_addr;
    uint16_t target_state;   /* bit0-3 = ch1-4 relay target */
    uint16_t relay_state;    /* bit0-3 = ch1-4 relay actual */
    uint16_t feedback_state; /* bit0-3 = ch1-4 feedback */
    uint16_t fault_state;    /* bit0-3 = ch1-4 fault */
    uint16_t frequency;      /* ×0.01Hz */
    meter_channel_t channels[METER_CHANNELS_PER_BOARD];
    uint16_t delay[METER_CHANNELS_PER_BOARD]; /* seconds, 0-6 */
} meter_board_t;

/*============================================================================
 * Outlet ↔ Board/Channel Mapping
 *============================================================================*/

typedef struct {
    uint8_t slave_id;  /* Modbus slave address */
    uint8_t channel;   /* 0-3 */
} meter_outlet_map_t;

/*============================================================================
 * Total Power Meter (Slave ID 1, FC03)
 *
 * 3-phase input power meter at node level.
 * Register map (FC03, address 0x0100-0x0121, 34 registers):
 *
 *   Offset  Addr    Description              Unit / Scale
 *   ------  ------  -----------------------  ----------------
 *   [0]     0x0100  A phase voltage           ÷100 V
 *   [1]     0x0101  B phase voltage           ÷100 V
 *   [2]     0x0102  C phase voltage           ÷100 V
 *   [3]     0x0103  A phase current            ÷100 A
 *   [4]     0x0104  B phase current            ÷100 A
 *   [5]     0x0105  C phase current            ÷100 A
 *   [6]     0x0106  A phase active power       W
 *   [7]     0x0107  B phase active power       W
 *   [8]     0x0108  C phase active power       W
 *   [9-10]  0x0109  Total active power (U32)   W
 *   [11]    0x010B  A reactive power (skip)
 *   [12]    0x010C  B reactive power (skip)
 *   [13]    0x010D  C reactive power (skip)
 *   [14-15] 0x010E  Total reactive (skip, U32)
 *   [16]    0x0110  A apparent power (skip)
 *   [17]    0x0111  B apparent power (skip)
 *   [18]    0x0112  C apparent power (skip)
 *   [19-20] 0x0113  Total apparent (skip, U32)
 *   [21]    0x0115  Frequency                 ÷100 Hz
 *   [22]    0x0116  A phase PF                ÷1000
 *   [23]    0x0117  B phase PF                ÷1000
 *   [24]    0x0118  C phase PF                ÷1000
 *   [25]    0x0119  Total PF                  ÷1000
 *   [26-27] 0x011A  A active energy (U32)     ÷100 kWh
 *   [28-29] 0x011C  B active energy (U32)     ÷100 kWh
 *   [30-31] 0x011E  C active energy (U32)     ÷100 kWh
 *   [32-33] 0x0120  Total active energy (U32) ÷100 kWh
 *============================================================================*/

#define TOTAL_METER_SLAVE_ID    1
#define TOTAL_METER_START_REG   0x0100
#define TOTAL_METER_REG_COUNT   34      /* 0x0100 to 0x0121 */

/* Register offsets (index into FC03 response array, base = 0x0100) */
#define TM_OFF_VA       0       /* A voltage */
#define TM_OFF_VB       1       /* B voltage */
#define TM_OFF_VC       2       /* C voltage */
#define TM_OFF_IA       3       /* A current */
#define TM_OFF_IB       4       /* B current */
#define TM_OFF_IC       5       /* C current */
#define TM_OFF_PA       6       /* A active power */
#define TM_OFF_PB       7       /* B active power */
#define TM_OFF_PC       8       /* C active power */
#define TM_OFF_PT_HI    9       /* Total active power (U32 high) */
#define TM_OFF_PT_LO    10      /* Total active power (U32 low) */
#define TM_OFF_FREQ     21      /* Frequency */
#define TM_OFF_PFA      22      /* A PF */
#define TM_OFF_PFB      23      /* B PF */
#define TM_OFF_PFC      24      /* C PF */
#define TM_OFF_PFT      25      /* Total PF */
#define TM_OFF_EA_HI    26      /* A active energy (U32 high) */
#define TM_OFF_EA_LO    27      /* A active energy (U32 low) */
#define TM_OFF_EB_HI    28      /* B active energy */
#define TM_OFF_EB_LO    29
#define TM_OFF_EC_HI    30      /* C active energy */
#define TM_OFF_EC_LO    31
#define TM_OFF_ET_HI    32      /* Total active energy */
#define TM_OFF_ET_LO    33

/*============================================================================
 * Total Meter Data (after parsing)
 *============================================================================*/

typedef struct {
    uint16_t voltage[3];     /* A/B/C, raw ÷100V */
    uint16_t current[3];     /* A/B/C, raw ÷100A */
    uint16_t power[3];       /* A/B/C, raw W */
    uint32_t total_power;    /* raw W (U32) */
    uint16_t frequency;      /* raw ÷100Hz */
    uint16_t pf[3];          /* A/B/C, raw ÷1000 */
    uint16_t total_pf;       /* raw ÷1000 */
    uint32_t energy[3];      /* A/B/C, raw ÷100kWh */
    uint32_t total_energy;   /* raw ÷100kWh */
} total_meter_data_t;

/*============================================================================
 * API
 *============================================================================*/

/**
 * @brief Read all registers from one metering board (FC04, 38 regs).
 * @param slave_id  Modbus slave address (2-13)
 * @param out       Parsed board data
 * @return MB_OK on success, negative on failure
 */
int meter_read_all(uint8_t slave_id, meter_board_t *out);

/**
 * @brief Control a relay on a metering board (FC06).
 * @param slave_id  Modbus slave address
 * @param channel   Channel index 0-3
 * @param on        1=ON (close relay), 0=OFF (open relay)
 * @return MB_OK on success, negative on failure
 */
int meter_set_relay(uint8_t slave_id, uint8_t channel, uint8_t on);

/**
 * @brief Map outlet ID (0-47) to board slave_id and channel.
 * @param outlet_id  Global outlet index
 * @param out        Output mapping
 */
void meter_outlet_to_board(uint8_t outlet_id, meter_outlet_map_t *out);

/**
 * @brief Convert 1084 channel data to CAN metrics format.
 *
 * Unit conversions:
 *   voltage: 1084 ×0.01V → CAN ×0.1V   (÷10)
 *   current: 1084 ×0.001A → CAN ×0.01A  (÷10)
 *   power:   1084 ×0.1W  → CAN ×1W      (÷10)
 *   pf:      1084 ×0.0001 → CAN ×0.01   (÷100)
 *   energy:  1084 ×0.0001kWh → CAN ×0.01kWh (÷100)
 */
void meter_channel_to_can(const meter_channel_t *ch, can_metrics_t *out);

/**
 * @brief 由一次 meter_read_all() 的結果更新該板的 CAN 板級狀態
 *
 * 成功：frequency 由 1084 暫存器 0x0009 直接搬值（同為 ×0.01Hz，零算術），
 *       status 設為 CAN_BOARD_ONLINE（reserved bits 一併清零）。
 * 失敗：**只清 online bit，frequency 保留最後已知值**。
 *       清零會讓該板 4 埠的既有告警在 host 端看起來像「已恢復」——
 *       離線是「失去觀測能力」而非「確知正常」。
 *
 * @param bs     輸出，3 B packed；呼叫端需跨輪保留（不可每輪重新歸零）
 * @param board  本輪讀到的板資料；rc != MB_OK 時其內容不被使用
 * @param rc     meter_read_all() 的回傳值
 */
void meter_board_status_update(can_board_status_t *bs,
                               const meter_board_t *board,
                               int rc);

/**
 * @brief Read total power meter (FC03, 34 regs, slave ID 1).
 * @param out  Parsed total meter data
 * @return MB_OK on success, negative on failure
 */
int total_meter_read(total_meter_data_t *out);

/**
 * @brief Convert total meter data to CAN phase_metrics format.
 *
 * Unit conversions:
 *   voltage: meter ÷100V  → CAN ×0.1V    (÷10)
 *   current: meter ÷100A  → CAN ×0.01A   (same scale)
 *   power:   meter W      → CAN ×1W      (same, U16 clamp)
 *   pf:      meter ÷1000  → CAN ×0.01    (÷10)
 *   energy:  meter ÷100kWh → CAN ×0.01kWh (same scale)
 *
 * @param tm         Total meter data
 * @param out_total  CAN metrics for total (phase_metrics[0])
 * @param out_phases CAN metrics for L1/L2/L3 (phase_metrics[1..3])
 */
void total_meter_to_can(const total_meter_data_t *tm,
                        can_metrics_t *out_total,
                        can_metrics_t out_phases[3]);

/*============================================================================
 * 單相總電錶 JSY-MK-109（JSY1009）
 *
 * 出貨時單相／三相**只會安裝一種**，兩者 slave id 都是 1，靠型號暫存器分辨。
 * 暫存器與刻度與三相完全不同，不可套用 TM_OFF_* 或 total_meter_to_can()。
 *============================================================================*/

#define SINGLE_METER_START_REG  0x0048
#define SINGLE_METER_REG_COUNT  10      /* 0x0048 .. 0x0051 */

/* Register offsets (index into FC03 response array, base = 0x0048) */
#define SM_OFF_VOLTAGE  0       /* 0x0048, /100 V    */
#define SM_OFF_CURRENT  1       /* 0x0049, /1000 A   */
#define SM_OFF_POWER    2       /* 0x004A, W         */
#define SM_OFF_EP_HI    3       /* 0x004B, U32 high  */
#define SM_OFF_EP_LO    4       /* 0x004C, U32 low   */
#define SM_OFF_PF       5       /* 0x004D, /1000     */
#define SM_OFF_FREQ     9       /* 0x0051, /100 Hz   */

typedef struct {
    uint16_t voltage;        /* raw /100 V    */
    uint16_t current;        /* raw /1000 A   */
    uint16_t power;          /* raw W         */
    uint32_t energy;         /* raw /3200 kWh ⚠ 非 10 的冪次，見 single_meter_to_can() */
    uint16_t pf;             /* raw /1000     */
    uint16_t frequency;      /* raw /100 Hz   */
} single_meter_data_t;

/*============================================================================
 * 電錶機種
 *============================================================================*/

/* 兩款電錶都在 0x0000/0x0001 有型號暫存器、都用 FC03，位址重疊而值不同。 */
#define METER_MODEL_REG         0x0000
#define METER_MODEL_REG_COUNT   2
#define METER_MODEL1_THREE      0x0333  /* JSY-MK-333 三相 */
#define METER_MODEL1_SINGLE     0x9000  /* JSY-MK-109 單相 */
#define METER_MODEL2_SINGLE     0xB131  /* JSY-MK-109 型號2（輔助確認）*/

typedef enum {
    METER_TYPE_UNKNOWN = 0,   /* 尚未偵測到／讀取失敗 → 必須重試 */
    METER_TYPE_SINGLE  = 1,
    METER_TYPE_THREE   = 2
} meter_type_t;

/**
 * @brief 由型號暫存器值判定機種（純函式，可 host 測試）
 *
 * ⚠ 用**兩個正面判準**，不是「不是三相就是單相」：接上第三種電錶時必須明確
 *   回 UNKNOWN 並持續重試，而不是被誤判成某一種後讀到全 0——那會長得跟
 *   「電錶離線」一模一樣，且沒有任何錯誤訊息。
 *
 * @param model1 暫存器 0x0000 的值
 * @param model2 暫存器 0x0001 的值（目前僅供日後擴充，單相為 0xB131）
 */
meter_type_t meter_type_from_model(uint16_t model1, uint16_t model2);

/**
 * @brief 讀型號暫存器並判定機種（FC03, slave 1, 0x0000, 2 regs）
 * @return METER_TYPE_UNKNOWN 表示讀取失敗或型號不認得，呼叫端必須重試
 */
meter_type_t meter_detect_type(void);

/**
 * @brief 解析單相電錶的 FC03 回應（純函式）
 * @param regs SINGLE_METER_REG_COUNT 個暫存器，索引 = 位址 - 0x0048
 * @param out  輸出，先被清零
 */
void single_meter_parse(const uint16_t *regs, single_meter_data_t *out);

/**
 * @brief 讀取單相總電錶（FC03, slave 1, 0x0048, 10 regs）
 * @return MB_OK on success, negative on failure
 */
int single_meter_read(single_meter_data_t *out);

/**
 * @brief 單相電錶 → CAN 定點（純函式）
 *
 * 刻度與三相**完全不同**，四者各異：
 *   voltage: /100 V    → CAN ×0.1V     (raw/10)
 *   current: /1000 A   → CAN ×0.01A    (raw/10)   ← 三相是 raw
 *   power:   W         → CAN ×1W       (raw)
 *   pf:      /1000     → CAN ×0.01     (raw/10)
 *   energy:  /3200 kWh → CAN ×0.01kWh  (raw/32)   ← 三相是 raw
 *
 * ⚠ energy 的 /3200 取自手冊，尚未實機驗證（見 spec AC9）。誤用 /100 會差 32 倍
 *   且無任何錯誤訊息。
 */
void single_meter_to_can(const single_meter_data_t *sm, can_metrics_t *out_total);

#endif /* __MODBUS_METER_H */
