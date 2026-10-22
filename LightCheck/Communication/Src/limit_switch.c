#include "limit_switch.h"

void LimitSwitch_Init(limit_switch_t *limit)
{
    limit->trips = 1U;
    limit->active = 1U;
    limit->safe_samples = 0U;
}

void LimitSwitch_Sample(limit_switch_t *limit, uint8_t unsafe)
{
    if (unsafe != 0U) {
        if (limit->active == 0U) ++limit->trips;
        limit->active = 1U;
        limit->safe_samples = 0U;
    } else if (limit->active != 0U) {
        if (++limit->safe_samples >= 30U) {
            limit->active = 0U;
            limit->safe_samples = 0U;
        }
    }
}
