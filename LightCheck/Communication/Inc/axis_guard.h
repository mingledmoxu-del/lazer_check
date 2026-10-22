#ifndef AXIS_GUARD_H
#define AXIS_GUARD_H
#include "limit_switch.h"
#include <stdint.h>

#define GUARD_INPUT_COUNT 6U
#define GUARD_UNUSED 255U
/* Stopped encoder jitter at a measured end pose; command targets stay strict. */
#define GUARD_DUAL_FEEDBACK_MARGIN 50
enum { HOME_IDLE, HOME_SEEK, HOME_CONTACT_STOP, HOME_BACKOFF,
       HOME_BACKOFF_STOP, HOME_LATCH, HOME_FINAL_STOP, HOME_DONE, HOME_FAULT };
enum { GUARD_OK, GUARD_SENSOR, GUARD_RANGE, GUARD_FEEDBACK, GUARD_TIMEOUT,
       GUARD_COMM, GUARD_ALARM, GUARD_CANCELLED, GUARD_EXECUTION };
typedef struct {
    volatile uint8_t raw;
    limit_switch_t high[GUARD_INPUT_COUNT], low[GUARD_INPUT_COUNT];
} guard_inputs_t;
typedef struct {
    uint8_t negative, positive, home, active_high;
    uint8_t hardware_on, soft_on, confirmed, home_at_end;
    int8_t home_direction;
    int32_t minimum, maximum; /* output millidegrees */
    uint8_t zero_valid, phase, fault, stop_pending, zero_samples;
    int32_t origin, home_start;
    int32_t reference_angle; /* pose at captured reference, millidegrees */
    uint8_t fixed_range; /* Fixture policy, independent of reference_angle==0. */
    uint16_t ppr;
    int16_t home_rpm;
    uint32_t home_budget, home_deadline, poll_at, retry_at;
    uint32_t backoff_budget;
    uint32_t progress_at;
    int32_t stage_start, home_last;
    uint64_t travel_counts;
    uint32_t seen[3];
    /* Axis1 two measured poses: horizontal-side, raised-side. RAM only. */
    uint8_t dual_enabled, dual_channel[2], dual_valid, dual_last_hit, dual_pending, dual_home;
    uint8_t dual_home_target; /* 0: either sensor; 2: raised sensor required. */
    int8_t angle_sign; /* encoder-positive -> physical-positive, +1 or -1 */
    int32_t dual_angle[2];
    uint32_t dual_seen[2];
} axis_guard_t;
void GuardInputs_Init(guard_inputs_t *inputs);
void GuardInputs_Sample(guard_inputs_t *inputs, uint8_t raw);
void AxisGuard_Init(axis_guard_t *guard);
uint8_t AxisGuard_Active(const guard_inputs_t *inputs, const axis_guard_t *guard, uint8_t channel);
uint32_t AxisGuard_Trips(const guard_inputs_t *inputs, const axis_guard_t *guard, uint8_t channel);
int AxisGuard_Angle(const axis_guard_t *guard, int32_t raw, uint16_t mode, uint16_t ppr,
                   uint16_t ratio_num, uint16_t ratio_den, int32_t *angle);
int AxisGuard_TargetAllowed(const axis_guard_t *guard, int32_t current, int32_t delta);
#endif
