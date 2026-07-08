/**
 * @file env_source.h
 * @brief 節點溫濕度資料來源抽象層。
 *
 * 感測器來源尚未定案（候選：I2C 溫濕度 sensor、1084 板/JSY 電表暫存器）。
 * 定案後只需改寫 env_source.c，介面與呼叫端不變。
 */
#ifndef __ENV_SOURCE_H
#define __ENV_SOURCE_H

#include <stdint.h>

/**
 * @brief 讀取節點溫濕度（每次呼叫更新一步）。
 * @param temp_x10 輸出：溫度 ×0.1°C（有號定點）
 * @param humi_x10 輸出：濕度 ×0.1%RH
 * @return CAN_ENV_VALID_TEMP / CAN_ENV_VALID_HUMI 位元組合；0 = 無有效資料（輸出參數不變）
 */
uint8_t env_source_read(int16_t *temp_x10, uint16_t *humi_x10);

#endif /* __ENV_SOURCE_H */
