/**
 * @file modbus_meter.c
 * @brief 1084 metering board read/write implementation.
 *
 * Reads all 38 registers from a board via FC04, parses into meter_board_t.
 * Controls relays via FC06.
 * Converts 1084 fixed-point units to CAN protocol fixed-point units.
 */

#include "modbus_meter.h"
#include <string.h>

/*============================================================================
 * Debug log (defined in Thread.c)
 *============================================================================*/
extern void dbg_log(const char *fmt, ...);

/*============================================================================
 * Register offset table for each channel's first register (voltage)
 * Ch1=0x000A, Ch2=0x0010, Ch3=0x0016, Ch4=0x001C
 * Each channel occupies 6 register slots (V, I, P, PF, EP_hi, EP_lo)
 *============================================================================*/
static const uint16_t ch_voltage_reg[METER_CHANNELS_PER_BOARD] = {
    METER_REG_CH1_VOLTAGE,  /* 0x000A = index 10 in FC04 response */
    METER_REG_CH2_VOLTAGE,  /* 0x0010 = index 16 */
    METER_REG_CH3_VOLTAGE,  /* 0x0016 = index 22 */
    METER_REG_CH4_VOLTAGE,  /* 0x001C = index 28 */
};

/*============================================================================
 * meter_outlet_to_board
 *============================================================================*/

void meter_outlet_to_board(uint8_t outlet_id, meter_outlet_map_t *out)
{
    uint8_t board_idx = outlet_id / METER_CHANNELS_PER_BOARD;
    out->slave_id = board_idx + METER_SLAVE_ID_BASE;
    out->channel  = outlet_id % METER_CHANNELS_PER_BOARD;
}

/*============================================================================
 * meter_channel_to_can — unit conversion from 1084 to CAN format
 *============================================================================*/

void meter_channel_to_can(const meter_channel_t *ch, can_metrics_t *out)
{
    /* voltage: 1084 ×0.01V → CAN ×0.1V  => divide by 10 */
    out->voltage = ch->voltage / 10;

    /* current: 1084 ×0.001A → CAN ×0.01A => divide by 10 */
    out->current = ch->current / 10;

    /* power: 1084 ×0.1W → CAN ×1W => divide by 10 */
    out->power = ch->power / 10;

    /* pf: 1084 ×0.0001 → CAN ×0.01 => divide by 100 */
    out->pf = (uint8_t)(ch->pf / 100);

    /* energy: 1084 ×0.0001kWh → CAN ×0.01kWh => divide by 100 */
    out->energy = (uint32_t)(ch->energy / 100);
}

/*============================================================================
 * Parse FC04 register array into meter_board_t
 *
 * regs[0..37] correspond to register addresses 0x0000..0x0025
 *============================================================================*/

static void parse_board_data(const uint16_t *regs, meter_board_t *out)
{
    uint8_t ch;
    uint16_t base_idx;

    memset(out, 0, sizeof(meter_board_t));

    /* Header registers */
    out->device_model  = regs[0];
    out->hw_version    = regs[1];
    out->sw_version    = regs[2];
    out->rs485_addr    = regs[4];
    out->target_state  = regs[5];
    out->relay_state   = regs[6];
    out->feedback_state = regs[7];
    out->fault_state   = regs[8];
    out->frequency     = regs[9];

    /* Per-channel data */
    for (ch = 0; ch < METER_CHANNELS_PER_BOARD; ch++) {
        base_idx = ch_voltage_reg[ch];  /* This is also the array index since start_reg=0 */

        out->channels[ch].voltage = regs[base_idx];
        out->channels[ch].current = regs[base_idx + 1];
        out->channels[ch].power   = regs[base_idx + 2];
        out->channels[ch].pf      = regs[base_idx + 3];
        /* Energy is U32: high word at base_idx+4, low word at base_idx+5 */
        out->channels[ch].energy  = ((uint32_t)regs[base_idx + 4] << 16)
                                  | (uint32_t)regs[base_idx + 5];
    }

    /* Delay times */
    out->delay[0] = regs[0x0022];  /* index 34 */
    out->delay[1] = regs[0x0023];  /* index 35 */
    out->delay[2] = regs[0x0024];  /* index 36 */
    out->delay[3] = regs[0x0025];  /* index 37 */
}

/*============================================================================
 * meter_read_all
 *============================================================================*/

int meter_read_all(uint8_t slave_id, meter_board_t *out)
{
    uint16_t regs[METER_REG_READ_COUNT];
    int rc;

    rc = modbus_read_input_regs(slave_id, 0x0000, METER_REG_READ_COUNT, regs);
    if (rc != MB_OK) {
        dbg_log("[METER] read_all failed: slave=%u rc=%d\r\n", slave_id, rc);
        return rc;
    }

    parse_board_data(regs, out);

    dbg_log("[METER] slave=%u freq=%u relay=0x%04X fault=0x%04X\r\n",
            slave_id, out->frequency, out->relay_state, out->fault_state);
    dbg_log("[METER]   ch1: V=%u I=%u P=%u PF=%u E=%lu\r\n",
            out->channels[0].voltage, out->channels[0].current,
            out->channels[0].power, out->channels[0].pf,
            (unsigned long)out->channels[0].energy);

    return MB_OK;
}

/*============================================================================
 * meter_set_relay
 *============================================================================*/

int meter_set_relay(uint8_t slave_id, uint8_t channel, uint8_t on)
{
    uint16_t reg_addr;
    uint16_t value;
    int rc;

    if (channel >= METER_CHANNELS_PER_BOARD) return MB_ERR_BAD_FC;

    /* FC06 relay address: channel 0→addr 1, channel 1→addr 2, etc. */
    reg_addr = (uint16_t)(channel + 1);
    value = on ? METER_RELAY_ON : METER_RELAY_OFF;

    dbg_log("[METER] set_relay: slave=%u ch=%u %s (addr=%u val=0x%04X)\r\n",
            slave_id, channel, on ? "ON" : "OFF", reg_addr, value);

    rc = modbus_write_single_reg(slave_id, reg_addr, value);
    if (rc != MB_OK) {
        dbg_log("[METER] set_relay failed: rc=%d\r\n", rc);
    }
    return rc;
}
