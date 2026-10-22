#ifndef LIMIT_DEBUG_H
#define LIMIT_DEBUG_H
#include "stm32h7xx_hal.h"
#include "limit_switch.h"

void LimitDebug_Init(UART_HandleTypeDef *uart);
/* Single producer: call in SysTick after sampling, never transmits here. */
void LimitDebug_Sample(uint8_t axis, uint8_t raw, const limit_switch_t *limit);
/* Single consumer: call in main after the stop task, not inside interrupts. */
void LimitDebug_Task(void);
#endif
