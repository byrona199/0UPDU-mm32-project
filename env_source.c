/**
 * @file env_source.c
 * @brief 溫濕度來源實作。
 *
 * 預設（stub）：無感測器，回報無效。
 * ENV_TEST_DATA：定點三角波假資料，供 Linux 端 52-byte 真值解碼與
 * 告警鏈路聯調（範圍刻意跨越 pdu-core 預設門檻 45.0°C / 90.0%RH）。
 */
#include "can_protocol.h"
#include "env_source.h"

#ifdef ENV_TEST_DATA

#define ENV_TEST_TEMP_INIT   250   /* 25.0°C */
#define ENV_TEST_TEMP_MIN    200   /* 20.0°C */
#define ENV_TEST_TEMP_MAX    500   /* 50.0°C */
#define ENV_TEST_HUMI_INIT   600   /* 60.0%RH */
#define ENV_TEST_HUMI_MIN    400   /* 40.0%RH */
#define ENV_TEST_HUMI_MAX    950   /* 95.0%RH */

uint8_t env_source_read(int16_t *temp_x10, uint16_t *humi_x10)
{
    static int16_t  t     = ENV_TEST_TEMP_INIT;
    static int16_t  t_dir = 1;
    static uint16_t h     = ENV_TEST_HUMI_INIT;
    static int16_t  h_dir = 1;

    t += t_dir;
    if (t >= ENV_TEST_TEMP_MAX) { t = ENV_TEST_TEMP_MAX; t_dir = -1; }
    else if (t <= ENV_TEST_TEMP_MIN) { t = ENV_TEST_TEMP_MIN; t_dir = 1; }

    h = (uint16_t)(h + h_dir);
    if (h >= ENV_TEST_HUMI_MAX) { h = ENV_TEST_HUMI_MAX; h_dir = -1; }
    else if (h <= ENV_TEST_HUMI_MIN) { h = ENV_TEST_HUMI_MIN; h_dir = 1; }

    *temp_x10 = t;
    *humi_x10 = h;
    return CAN_ENV_VALID_TEMP | CAN_ENV_VALID_HUMI;
}

#else /* stub：感測器來源未定 */

uint8_t env_source_read(int16_t *temp_x10, uint16_t *humi_x10)
{
    (void)temp_x10;
    (void)humi_x10;
    return 0;
}

#endif /* ENV_TEST_DATA */
