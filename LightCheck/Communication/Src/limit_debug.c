#include "limit_debug.h"
#include <stdio.h>
#include <string.h>

#define EVENT_COUNT 16U
typedef struct {
    uint32_t tick, trips;
    uint8_t axis, raw, active, initial;
} limit_event_t;
static volatile limit_event_t events[EVENT_COUNT];
static volatile uint8_t head, tail;
static volatile uint32_t dropped;
static uint32_t reported_dropped, previous_trips[2];
static uint8_t seen[2], previous_active[2];
static UART_HandleTypeDef *debug_uart;

void LimitDebug_Init(UART_HandleTypeDef *uart)
{
    /* Initialize before enabling the SysTick sampling hook. */
    debug_uart = uart;
    head = tail = 0U;
    dropped = reported_dropped = 0U;
    memset(seen, 0, sizeof(seen));
    memset(previous_trips, 0, sizeof(previous_trips));
    memset(previous_active, 0, sizeof(previous_active));
}

void LimitDebug_Sample(uint8_t axis, uint8_t raw, const limit_switch_t *limit)
{
    if (axis >= 2U || limit == NULL) return;
    uint32_t trips = limit->trips;
    uint8_t active = limit->active;
    if (seen[axis] && previous_active[axis] == active && previous_trips[axis] == trips)
        return;
    uint8_t next = (uint8_t)((head + 1U) % EVENT_COUNT);
    if (next == tail) {
        ++dropped;
    } else {
        events[head] = (limit_event_t){ HAL_GetTick(), trips, axis, raw,
                                      active, (uint8_t)!seen[axis] };
        head = next;
    }
    seen[axis] = 1U;
    previous_active[axis] = active;
    previous_trips[axis] = trips;
}

void LimitDebug_Task(void)
{
    if (debug_uart == NULL) return;
    char line[112];
    int size;
    uint32_t lost = dropped;
    if (lost != reported_dropped) {
        size = snprintf(line, sizeof(line), "[LIMIT] WARNING dropped=%lu\r\n",
                        (unsigned long)lost);
        reported_dropped = lost;
    } else {
        if (tail == head) return;
        limit_event_t event = events[tail];
        tail = (uint8_t)((tail + 1U) % EVENT_COUNT);
        size = snprintf(line, sizeof(line),
                        "[LIMIT %lu ms] M%u DI%u %s raw=%u active=%u count=%lu\r\n",
                        (unsigned long)event.tick, (unsigned)(event.axis + 1U),
                        (unsigned)event.axis,
                        event.initial ? "INIT" : event.active ? "TRIGGER" : "CLEAR",
                        (unsigned)event.raw, (unsigned)event.active,
                        (unsigned long)(event.trips - 1U));
    }
    if (size > 0 && (unsigned)size < sizeof(line))
        /* Best effort only. No retries: debug must not hold up the stop task. */
        (void)HAL_UART_Transmit(debug_uart, (uint8_t *)line, (uint16_t)size, 20U);
}
