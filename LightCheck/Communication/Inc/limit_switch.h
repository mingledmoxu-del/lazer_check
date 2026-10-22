#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H
#include <stdint.h>

/* Light-ON sensors: GPIO high means blocked beam, broken wire or no power.
   Assertion is immediate; release requires 30 consecutive safe samples. */
typedef struct {
    volatile uint32_t trips;
    volatile uint8_t active;
    uint8_t safe_samples;
} limit_switch_t;
void LimitSwitch_Init(limit_switch_t *limit);
void LimitSwitch_Sample(limit_switch_t *limit, uint8_t unsafe);
#endif
