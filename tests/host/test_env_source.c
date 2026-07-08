/* host 單元測試：gcc -I../../Inc -DENV_TEST_DATA -o test_env test_env_source.c ../../env_source.c */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "can_protocol.h"
#include "env_source.h"

int main(void)
{
    int16_t  t = 0;
    uint16_t h = 0;

#ifdef ENV_TEST_DATA
    /* 假資料模式：valid 兩 bit 皆立、初值附近、三角波在範圍內折返 */
    uint8_t v = env_source_read(&t, &h);
    assert(v == (CAN_ENV_VALID_TEMP | CAN_ENV_VALID_HUMI));
    assert(t >= 200 && t <= 500);   /* 20.0~50.0°C */
    assert(h >= 400 && h <= 950);   /* 40.0~95.0%RH */

    /* 走 10000 步：永不越界、且曾觸頂/觸底（證明有折返而非卡死） */
    int hit_tmax = 0, hit_tmin = 0, hit_hmax = 0, hit_hmin = 0;
    for (int i = 0; i < 10000; i++) {
        v = env_source_read(&t, &h);
        assert(v == (CAN_ENV_VALID_TEMP | CAN_ENV_VALID_HUMI));
        assert(t >= 200 && t <= 500);
        assert(h >= 400 && h <= 950);
        if (t == 500) hit_tmax = 1;
        if (t == 200) hit_tmin = 1;
        if (h == 950) hit_hmax = 1;
        if (h == 400) hit_hmin = 1;
    }
    assert(hit_tmax && hit_tmin && hit_hmax && hit_hmin);
    printf("test_env_source (ENV_TEST_DATA): all assertions passed\n");
#else
    /* stub 模式：回 0 且不碰輸出參數 */
    t = 123; h = 456;
    uint8_t v = env_source_read(&t, &h);
    assert(v == 0);
    assert(t == 123 && h == 456);
    printf("test_env_source (stub): all assertions passed\n");
#endif
    return 0;
}
