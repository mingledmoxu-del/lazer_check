#ifndef P100S_BENCH_H
#define P100S_BENCH_H
#include "p100s.h"

/* 调试器写 command，恢复运行后执行一次。先设置 rpm，再写命令。 */
enum { BENCH_IDLE, BENCH_CHECK, BENCH_ENABLE, BENCH_RUN, BENCH_STOP, BENCH_DISABLE };
enum { BENCH_UNKNOWN, BENCH_DISABLED, BENCH_ENABLED, BENCH_RUNNING, BENCH_STOPPING, BENCH_FAULT };
typedef struct {
    volatile uint32_t command;
    volatile int32_t rpm; /* 首次调试限制为 -100..100 rpm */
    uint32_t state;
    uint32_t deadline_start;
    uint16_t failed_address;
    uint16_t actual;
    uint16_t expected;
    uint16_t run_state;
    modbus_result_t result; /* 命令结果；不被周期状态读取覆盖 */
    modbus_result_t stop_result;
    modbus_result_t disable_result;
    modbus_result_t poll_result;
    uint8_t exception_code;
} p100s_bench_t;
void p100s_bench_init(p100s_bench_t *bench);
void p100s_bench_command(p100s_bench_t *bench, p100s_handle_t *motor, uint32_t now);
void p100s_bench_poll(p100s_bench_t *bench, p100s_handle_t *motor, uint32_t now);
#endif
