/**
 * @file can_protocol.h
 * @brief CAN Bus binary protocol definitions shared between MM32 firmware and can-meter.
 *
 * This file defines the custom lightweight binary protocol used for communication
 * between the Linux master (can-meter) and up to 20 MM32 nodes per CAN bus.
 *
 * Protocol overview:
 * - 11-bit Standard CAN ID: MSG_TYPE[10:5] | NODE_ID[4:0]
 * - Transport layer: single-frame and multi-frame with Byte 0 header
 * - Fixed-point encoding for metrics (no float on CAN)
 * - Full-burst polling: one POLL_REQ triggers POWER + OUTLET_STATE + OUTLET_METRICS
 */

#ifndef PDU_CAN_PROTOCOL_H
#define PDU_CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Allow inclusion in both Linux (stdint.h) and Keil (cstdint or u8/u16) environments */
#ifdef __KEIL__
#include <stdint.h>
#else
#include <stdint.h>
#include <stdbool.h>
#endif

/*============================================================================
 * CAN Bus Parameters
 *============================================================================*/

#define CAN_BITRATE             250000  /* 250 kbps */
#define CAN_MAX_DATA_LEN        8       /* Standard CAN frame data length */
#define CAN_TRANSPORT_PAYLOAD   7       /* Payload per frame (Byte 0 = header) */

/*============================================================================
 * Node Configuration
 *============================================================================*/

#define CAN_NODE_ID_BROADCAST   0       /* Broadcast address */
#define CAN_NODE_ID_MIN         1
#define CAN_NODE_ID_MAX         20
#define CAN_NODES_PER_BUS       20
#define CAN_NUM_BUSES           2
#define CAN_TOTAL_NODES         (CAN_NODES_PER_BUS * CAN_NUM_BUSES) /* 40 */

#define CAN_MAX_OUTLETS         48
#define CAN_MAX_PHASES          3       /* L1, L2, L3 */
#define CAN_TOTAL_PHASES        4       /* total + L1 + L2 + L3 */

/*============================================================================
 * CAN ID Layout: MSG_TYPE[10:5] | NODE_ID[4:0]
 *
 * Lower CAN ID = higher bus priority (CAN arbitration).
 * MSG_TYPE 0x00 (RELAY_CMD) has the highest priority.
 *============================================================================*/

#define CAN_ID_NODE_BITS        5
#define CAN_ID_NODE_MASK        0x1F    /* 5 bits */
#define CAN_ID_MSG_SHIFT        5

/** @brief Build an 11-bit CAN ID from message type and node ID */
#define CAN_MAKE_ID(msg_type, node_id) \
    ((uint16_t)(((msg_type) << CAN_ID_MSG_SHIFT) | ((node_id) & CAN_ID_NODE_MASK)))

/** @brief Extract MSG_TYPE from a CAN ID */
#define CAN_GET_MSG_TYPE(can_id) \
    ((uint8_t)(((can_id) >> CAN_ID_MSG_SHIFT) & 0x3F))

/** @brief Extract NODE_ID from a CAN ID */
#define CAN_GET_NODE_ID(can_id) \
    ((uint8_t)((can_id) & CAN_ID_NODE_MASK))

/*============================================================================
 * MSG_TYPE Definitions (ordered by priority, lower = higher priority)
 *============================================================================*/

#define CAN_MSG_RELAY_CMD           0x00    /* Master→Node: relay control */
#define CAN_MSG_RELAY_ACK           0x01    /* Node→Master: relay ack */
#define CAN_MSG_HEARTBEAT           0x02    /* Bidirectional: keepalive */
#define CAN_MSG_POLL_REQ            0x03    /* Master→Node: poll request */
#define CAN_MSG_POWER_METRICS       0x04    /* Node→Master: power data (multi-frame) */
#define CAN_MSG_OUTLET_STATE        0x05    /* Node→Master: outlet states (multi-frame) */
#define CAN_MSG_OUTLET_METRICS      0x06    /* Node→Master: per-outlet data (multi-frame) */
#define CAN_MSG_CONFIG_WRITE        0x07    /* Master→Node: config write */
#define CAN_MSG_CONFIG_ACK          0x08    /* Node→Master: config ack */

/*============================================================================
 * Transport Header (Byte 0 of every CAN frame)
 *
 * Bits [7:6] = Frame Type
 * Bits [5:0] = Sequence Number (0-63)
 *============================================================================*/

#define CAN_FRAME_SINGLE        0x00    /* 00: complete message in one frame */
#define CAN_FRAME_FIRST         0x01    /* 01: first frame of multi-frame */
#define CAN_FRAME_CONTINUATION  0x02    /* 10: middle frame */
#define CAN_FRAME_LAST          0x03    /* 11: last frame of multi-frame */

#define CAN_FRAME_TYPE_SHIFT    6
#define CAN_FRAME_TYPE_MASK     0xC0
#define CAN_FRAME_SEQ_MASK      0x3F

/** @brief Build transport header byte */
#define CAN_MAKE_HEADER(frame_type, seq) \
    ((uint8_t)(((frame_type) << CAN_FRAME_TYPE_SHIFT) | ((seq) & CAN_FRAME_SEQ_MASK)))

/** @brief Extract frame type from header byte */
#define CAN_GET_FRAME_TYPE(header) \
    ((uint8_t)(((header) & CAN_FRAME_TYPE_MASK) >> CAN_FRAME_TYPE_SHIFT))

/** @brief Extract sequence number from header byte */
#define CAN_GET_SEQ(header) \
    ((uint8_t)((header) & CAN_FRAME_SEQ_MASK))

/*============================================================================
 * Fixed-Point Metrics Encoding
 *
 * Used for CAN transmission instead of float. MM32 has no FPU.
 * Conversion: real_value = raw_value * scale_factor
 *============================================================================*/

#define CAN_SCALE_CURRENT       0.01f   /* uint16 × 0.01 = Amps */
#define CAN_SCALE_VOLTAGE       0.1f    /* uint16 × 0.1  = Volts */
#define CAN_SCALE_POWER         1.0f    /* uint16 × 1    = Watts */
#define CAN_SCALE_PF            0.01f   /* uint8  × 0.01 = Power Factor */
#define CAN_SCALE_ENERGY        0.01f   /* uint32 × 0.01 = kWh */
#define CAN_SCALE_FREQUENCY     0.01f   /* uint16 × 0.01 = Hz */
#define CAN_SCALE_DELAY         100     /* uint16 × 100  = ms */

/** @brief Per-phase electrical metrics, fixed-point, 11 bytes packed */
typedef struct {
    uint16_t current;       /* ×0.01A,  range 0-655.35A */
    uint16_t voltage;       /* ×0.1V,   range 0-6553.5V */
    uint16_t power;         /* ×1W,     range 0-65535W */
    uint8_t  pf;            /* ×0.01,   range 0-1.00 */
    uint32_t energy;        /* ×0.01kWh, range 0-42949672 kWh */
} __attribute__((packed)) can_metrics_t;

#define CAN_METRICS_SIZE        11      /* sizeof(can_metrics_t) */

/*============================================================================
 * POWER_METRICS Payload Layout (MSG_TYPE 0x04)
 *
 * Total data: freq(2) + type(1) + 4×metrics(44) = 47 bytes
 * Frames: ceil(47/7) = 7 frames
 *
 * Byte order within the reassembled buffer:
 *   [0-1]   frequency   (uint16_t, ×0.01Hz)
 *   [2]     phase_type  (uint8_t, 0=single, 1=three-phase)
 *   [3-13]  total       (can_metrics_t)
 *   [14-24] phase L1    (can_metrics_t)
 *   [25-35] phase L2    (can_metrics_t)
 *   [36-46] phase L3    (can_metrics_t)
 *============================================================================*/

#define CAN_POWER_DATA_SIZE     47      /* 2 + 1 + 4×11 */
#define CAN_POWER_FRAME_COUNT   7       /* ceil(47/7) */

typedef struct {
    uint16_t      frequency;            /* ×0.01Hz */
    uint8_t       phase_type;           /* 0=single, 1=three-phase */
    can_metrics_t total;
    can_metrics_t phases[CAN_MAX_PHASES]; /* L1, L2, L3 */
} __attribute__((packed)) can_power_payload_t;

/*============================================================================
 * OUTLET_STATE Payload Layout (MSG_TYPE 0x05)
 *
 * Per outlet: state(1 bit) + phase_mapping(2 bits) = 3 bits
 * 48 outlets × 3 bits = 144 bits = 18 bytes
 * Frames: ceil(18/7) = 3 frames
 *
 * Bit packing (LSB first within each byte):
 *   Outlet 0: byte[0] bits [2:0]  → state=[0], phase=[2:1]
 *   Outlet 1: byte[0] bits [5:3]  → state=[3], phase=[5:4]
 *   Outlet 2: byte[0] bits [7:6] + byte[1] bit [0]
 *   ...
 *============================================================================*/

#define CAN_OUTLET_STATE_DATA_SIZE   18  /* ceil(48*3/8) */
#define CAN_OUTLET_STATE_FRAME_COUNT 3   /* ceil(18/7) */

/** @brief Pack one outlet's state+phase into 3 bits */
#define CAN_OUTLET_PACK(state, phase) \
    ((uint8_t)(((state) & 0x01) | (((phase) & 0x03) << 1)))

/** @brief Extract state from 3-bit packed value */
#define CAN_OUTLET_GET_STATE(packed)   ((packed) & 0x01)

/** @brief Extract phase from 3-bit packed value */
#define CAN_OUTLET_GET_PHASE(packed)   (((packed) >> 1) & 0x03)

/*============================================================================
 * OUTLET_METRICS Payload Layout (MSG_TYPE 0x06)
 *
 * 48 outlets × 11 bytes = 528 bytes
 * Frames: ceil(528/7) = 76 frames
 *============================================================================*/

#define CAN_OUTLET_METRICS_DATA_SIZE    (CAN_MAX_OUTLETS * CAN_METRICS_SIZE)  /* 528 */
#define CAN_OUTLET_METRICS_FRAME_COUNT  76  /* ceil(528/7) */

/*============================================================================
 * Total frames for a full burst response to POLL_REQ(ALL)
 *============================================================================*/

#define CAN_BURST_TOTAL_FRAMES  \
    (CAN_POWER_FRAME_COUNT + CAN_OUTLET_STATE_FRAME_COUNT + CAN_OUTLET_METRICS_FRAME_COUNT)
/* 7 + 3 + 76 = 86 response frames */

/*============================================================================
 * HEARTBEAT Payload (MSG_TYPE 0x02) — Single frame, 8 bytes
 *============================================================================*/

typedef struct {
    uint8_t  node_status;       /* 0=OK, 1=Warning, 2=Error */
    uint8_t  relay_fault_flags; /* bitfield: bit N = relay group N fault */
    uint8_t  reserved[6];
} __attribute__((packed)) can_heartbeat_t;

#define CAN_NODE_STATUS_OK      0
#define CAN_NODE_STATUS_WARNING 1
#define CAN_NODE_STATUS_ERROR   2

/*============================================================================
 * RELAY_CMD Payload (MSG_TYPE 0x00) — Single frame, 8 bytes
 *============================================================================*/

typedef struct {
    uint8_t  command;           /* see CAN_RELAY_CMD_* */
    uint8_t  outlet_id;         /* 0x00-0x2F for single outlet */
    uint16_t delay;             /* unit: 100ms, max 6553.5 sec */
    uint8_t  reserved[4];
} __attribute__((packed)) can_relay_cmd_t;

#define CAN_RELAY_CMD_OFF       0x00
#define CAN_RELAY_CMD_ON        0x01
#define CAN_RELAY_CMD_CYCLE     0x02    /* OFF then ON after delay */
#define CAN_RELAY_CMD_ALL_OFF   0x03
#define CAN_RELAY_CMD_ALL_ON    0x04

/*============================================================================
 * RELAY_ACK Payload (MSG_TYPE 0x01) — Single frame, 8 bytes
 *============================================================================*/

typedef struct {
    uint8_t  command;           /* echoed command */
    uint8_t  outlet_id;         /* echoed outlet */
    uint8_t  result;            /* 0=success, 1=fail */
    uint8_t  reserved[5];
} __attribute__((packed)) can_relay_ack_t;

#define CAN_RELAY_ACK_OK        0
#define CAN_RELAY_ACK_FAIL      1

/*============================================================================
 * POLL_REQ Payload (MSG_TYPE 0x03) — Single frame, 8 bytes
 *============================================================================*/

typedef struct {
    uint8_t  request_type;      /* 0x00 = ALL */
    uint8_t  reserved[6];       /* payload total must fit 7 bytes */
} __attribute__((packed)) can_poll_req_t;

#define CAN_POLL_ALL            0x00

/*============================================================================
 * CONFIG_WRITE Payload (MSG_TYPE 0x07) — Single frame, 8 bytes
 *============================================================================*/

typedef struct {
    uint8_t  config_id;         /* which config parameter */
    uint8_t  reserved_hdr;
    uint8_t  data[6];           /* config-specific data */
} __attribute__((packed)) can_config_write_t;

/*============================================================================
 * CONFIG_ACK Payload (MSG_TYPE 0x08) — Single frame, 8 bytes
 *============================================================================*/

typedef struct {
    uint8_t  config_id;         /* echoed config_id */
    uint8_t  result;            /* 0=success, 1=fail */
    uint8_t  reserved[6];
} __attribute__((packed)) can_config_ack_t;

/*============================================================================
 * Node State (used by can-meter on the master side)
 *============================================================================*/

typedef enum {
    CAN_NODE_OFFLINE = 0,       /* Never connected */
    CAN_NODE_ONLINE,            /* Normal communication */
    CAN_NODE_TIMEOUT,           /* 3 missed heartbeats (15s) */
    CAN_NODE_BUS_OFF,           /* 6 missed heartbeats (30s) */
    CAN_NODE_RECOVERING         /* Heartbeat received after timeout */
} can_node_state_t;

/*============================================================================
 * Timing Constants
 *============================================================================*/

#define CAN_HEARTBEAT_INTERVAL_S    5       /* Heartbeat every 5 seconds */
#define CAN_POLL_INTERVAL_S         5       /* Full poll cycle every 5 seconds */
#define CAN_POLL_TIMEOUT_MS         500     /* Timeout waiting for node response: must be > burst time (~190ms) */
#define CAN_REASSEMBLY_TIMEOUT_MS   500     /* Timeout for multi-frame reassembly */
#define CAN_HEARTBEAT_MISS_TIMEOUT  3       /* Missed HBs before NODE_TIMEOUT */
#define CAN_HEARTBEAT_MISS_BUSOFF   6       /* Missed HBs before NODE_BUS_OFF */
#define CAN_WATCHDOG_TIMEOUT_S      10      /* MM32 WDT timeout */

/*============================================================================
 * Reassembly Buffer Sizes
 *============================================================================*/

#define CAN_REASSEMBLY_POWER_SIZE   64      /* > 47 bytes */
#define CAN_REASSEMBLY_STATE_SIZE   32      /* > 18 bytes */
#define CAN_REASSEMBLY_METRICS_SIZE 544     /* > 528 bytes */

/*============================================================================
 * Helper: Convert between float and fixed-point
 *============================================================================*/

/** @brief Float current (A) → uint16 fixed-point */
#define CAN_ENCODE_CURRENT(f)   ((uint16_t)((f) / CAN_SCALE_CURRENT + 0.5f))
/** @brief uint16 fixed-point → float current (A) */
#define CAN_DECODE_CURRENT(u)   ((float)(u) * CAN_SCALE_CURRENT)

#define CAN_ENCODE_VOLTAGE(f)   ((uint16_t)((f) / CAN_SCALE_VOLTAGE + 0.5f))
#define CAN_DECODE_VOLTAGE(u)   ((float)(u) * CAN_SCALE_VOLTAGE)

#define CAN_ENCODE_POWER(f)     ((uint16_t)((f) / CAN_SCALE_POWER + 0.5f))
#define CAN_DECODE_POWER(u)     ((float)(u) * CAN_SCALE_POWER)

#define CAN_ENCODE_PF(f)        ((uint8_t)((f) / CAN_SCALE_PF + 0.5f))
#define CAN_DECODE_PF(u)        ((float)(u) * CAN_SCALE_PF)

#define CAN_ENCODE_ENERGY(f)    ((uint32_t)((f) / CAN_SCALE_ENERGY + 0.5f))
#define CAN_DECODE_ENERGY(u)    ((float)(u) * CAN_SCALE_ENERGY)

#define CAN_ENCODE_FREQUENCY(f) ((uint16_t)((f) / CAN_SCALE_FREQUENCY + 0.5f))
#define CAN_DECODE_FREQUENCY(u) ((float)(u) * CAN_SCALE_FREQUENCY)

#ifdef __cplusplus
}
#endif

#endif /* PDU_CAN_PROTOCOL_H */
