#include "limit_debug.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t now;
static char log_text[8192];
static unsigned used, sends;
uint32_t HAL_GetTick(void) { return now; }
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *uart, uint8_t *data,
                                  uint16_t size, uint32_t timeout)
{
    assert(uart != NULL && timeout <= 20U);
    assert(used + size < sizeof(log_text));
    memcpy(log_text + used, data, size); used += size; log_text[used] = '\0';
    ++sends;
    return HAL_OK;
}
static void sample(limit_switch_t *limit, uint8_t raw, unsigned count)
{
    while (count--) {
        ++now; LimitSwitch_Sample(limit, raw); LimitDebug_Sample(0U, raw, limit);
    }
}
int main(void)
{
    UART_HandleTypeDef uart = {0};
    limit_switch_t limit;
    LimitSwitch_Init(&limit); LimitDebug_Init(&uart);
    sample(&limit, 0U, 30U);
    assert(sends == 0U); /* ISR never transmits. */
    LimitDebug_Task(); LimitDebug_Task();
    assert(strstr(log_text, "M1 DI0 INIT raw=0 active=1 count=0"));
    assert(strstr(log_text, "M1 DI0 CLEAR raw=0 active=0 count=0"));
    unsigned before = sends;
    sample(&limit, 0U, 100U); LimitDebug_Task();
    assert(sends == before); /* No periodic spam. */
    sample(&limit, 1U, 1U); sample(&limit, 0U, 30U);
    LimitDebug_Task(); LimitDebug_Task();
    assert(strstr(log_text, "M1 DI0 TRIGGER raw=1 active=1 count=1"));
    assert(strstr(log_text, "M1 DI0 CLEAR raw=0 active=0 count=1"));
    sample(&limit, 1U, 1U); sample(&limit, 0U, 30U);
    LimitDebug_Task(); LimitDebug_Task();
    assert(strstr(log_text, "TRIGGER raw=1 active=1 count=2"));
    LimitDebug_Sample(1U, 0U, &limit); LimitDebug_Task();
    assert(strstr(log_text, "M2 DI1 INIT"));
    for (unsigned i = 0; i < 20U; ++i) {
        sample(&limit, 1U, 1U); sample(&limit, 0U, 30U);
    }
    LimitDebug_Task();
    assert(strstr(log_text, "WARNING dropped="));
    puts("limit debug queue tests passed");
    return 0;
}
