#include "axis_guard.h"
#include <string.h>
#include <limits.h>

void GuardInputs_Init(guard_inputs_t *inputs)
{
    memset(inputs, 0, sizeof(*inputs));
    for (unsigned i = 0; i < GUARD_INPUT_COUNT; ++i) {
        LimitSwitch_Init(&inputs->high[i]); LimitSwitch_Init(&inputs->low[i]);
    }
}
void GuardInputs_Sample(guard_inputs_t *inputs, uint8_t raw)
{
    inputs->raw = raw & 0x3FU;
    for (unsigned i = 0; i < GUARD_INPUT_COUNT; ++i) {
        uint8_t high = (raw >> i) & 1U;
        LimitSwitch_Sample(&inputs->high[i], high);
        LimitSwitch_Sample(&inputs->low[i], (uint8_t)!high);
    }
}
void AxisGuard_Init(axis_guard_t *guard)
{
    memset(guard, 0, sizeof(*guard));
    guard->negative = guard->positive = guard->home = GUARD_UNUSED;
    guard->active_high = 1U;
    guard->angle_sign = 1;
    guard->dual_channel[0]=2;guard->dual_channel[1]=0;
    guard->minimum = -90000; guard->maximum = 90000;
}
uint8_t AxisGuard_Active(const guard_inputs_t *inputs, const axis_guard_t *guard, uint8_t channel)
{
    if (channel >= GUARD_INPUT_COUNT) return 0U;
    return guard->active_high ? inputs->high[channel].active : inputs->low[channel].active;
}
uint32_t AxisGuard_Trips(const guard_inputs_t *inputs, const axis_guard_t *guard, uint8_t channel)
{
    if (channel >= GUARD_INPUT_COUNT) return 0U;
    return guard->active_high ? inputs->high[channel].trips : inputs->low[channel].trips;
}
int AxisGuard_Angle(const axis_guard_t *guard, int32_t raw, uint16_t mode, uint16_t ppr,
                   uint16_t ratio_num, uint16_t ratio_den, int32_t *angle)
{
    if (!guard->zero_valid || mode != 3U || !ppr || ppr != guard->ppr || !ratio_num || !ratio_den)
        return 0;
    int64_t delta = (int64_t)raw - guard->origin;
    /* Do not guess a coordinate across a signed counter discontinuity. */
    if (delta < INT32_MIN || delta > INT32_MAX) return 0;
    int64_t scaled = delta * 360000LL * (guard->angle_sign < 0 ? -1 : 1);
    if (scaled > INT64_MAX / ratio_den || scaled < INT64_MIN / ratio_den) return 0;
    int64_t value = scaled * ratio_den / ((int64_t)ppr * ratio_num) + guard->reference_angle;
    if (value < INT32_MIN || value > INT32_MAX) return 0;
    *angle = (int32_t)value;
    return 1;
}
int AxisGuard_TargetAllowed(const axis_guard_t *guard, int32_t current, int32_t delta)
{
    if (!guard->soft_on) return 1;
    int64_t target = (int64_t)current + delta;
    int32_t margin=guard->dual_enabled ? GUARD_DUAL_FEEDBACK_MARGIN : 0;
    return guard->zero_valid && (int64_t)current >= (int64_t)guard->minimum-margin &&
           (int64_t)current <= (int64_t)guard->maximum+margin &&
           target >= guard->minimum && target <= guard->maximum;
}
