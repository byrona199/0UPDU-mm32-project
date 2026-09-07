/* host 単元測試用的空樁：modbus_rtu.h 需要 hal_conf.h（MM32 HAL），
 * 但被測的轉換函式不觸碰硬體。
 * 以 -Itests/host/stubs 置於 -IInc 之前，即可讓此檔優先被選中。 */
#ifndef __HAL_CONF_H_STUB
#define __HAL_CONF_H_STUB
#endif
