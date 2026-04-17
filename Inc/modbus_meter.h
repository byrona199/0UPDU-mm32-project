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

#endif /* __MODBUS_METER_H */
