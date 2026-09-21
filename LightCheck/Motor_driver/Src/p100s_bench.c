#include "p100s_bench.h"
#include <string.h>

static modbus_result_t read_reg(p100s_handle_t *m, uint16_t a, uint16_t *v) {
    return modbus_read_holding(m->bus, m->slave_id, a, 1, v);
}

static modbus_result_t expect(p100s_bench_t *b, p100s_handle_t *m, uint16_t a, uint16_t v) {
    uint16_t actual = 0;
    modbus_result_t r = read_reg(m, a, &actual);
    if (r != MODBUS_OK || actual != v) {
        b->failed_address = a;
        b->actual = actual;
        b->expected = v;
        b->exception_code = r == MODBUS_ERROR_EXCEPTION ? m->bus->exception_code : 0;
        return r == MODBUS_OK ? MODBUS_ERROR_ARGUMENT : r;
    }
    return MODBUS_OK;
}

/* PA 暂存写入后读取普通地址，核对当前生效值。目标固件不支持则拒绝继续。 */
static modbus_result_t write_pa(p100s_bench_t *b, p100s_handle_t *m, uint16_t n, uint16_t v) {
    modbus_result_t r = p100s_write_parameter(m, P100S_GROUP_PA, n, v, P100S_STORAGE_TEMPORARY);
    if (r != MODBUS_OK) {
        b->failed_address = n + 0x80U;
        b->exception_code = m->exception_code;
        return r; /* 写超时可能已生效，不自动重发。 */
    }
    return expect(b, m, n, v);
}

static modbus_result_t check(p100s_bench_t *b, p100s_handle_t *m, uint16_t enabled) {
    /* 严格限定本次无外部 IO 的台架配置；任何不一致均停止在读阶段。 */
    static const uint16_t pairs[][2] = {
        {71,1}, {72,96}, {73,0}, {4,1}, {22,1}, {40,100}, {41,100},
        {0x100,1}, {0x101,2}, {0x102,3}, {0x103,4}, {0x104,0}, {0x105,0},
        {0x10f,0}, {0x110,0}, {0x111,0}, {0x112,0}, {0x113,0},
        {0x11e,0}, /* P3-30: 禁用虚拟输入 */
        {0x1013,0}, {0x1009,1}
    };
    b->failed_address = 0xffffU;
    b->exception_code = 0;
    for (unsigned i = 0; i < sizeof(pairs)/sizeof(pairs[0]); ++i) {
        modbus_result_t r = expect(b, m, pairs[i][0], pairs[i][1]);
        if (r != MODBUS_OK) return r;
    }
    modbus_result_t r = expect(b, m, 53, 0); /* 外部 SON 模式，软件不写 PA-53 */
    if (r != MODBUS_OK) return r;
    r = expect(b, m, 24, 0);
    if (r != MODBUS_OK) return r;
    return expect(b, m, 0x1016, enabled ? 3 : 1);
}

/* 故障/显式解除使能：即使零速写入失败，也独立尝试 PA53=0。
 * 只表示已发送停止请求，不保证断线时电机已经停下。 */
static void release(p100s_bench_t *b, p100s_handle_t *m, uint8_t fault) {
    uint16_t address = b->failed_address, actual = b->actual, expected = b->expected;
    uint8_t exception = b->exception_code;
    b->stop_result = write_pa(b, m, 24, 0);
    b->disable_result = expect(b, m, 0x1016, 1); /* 解除使能由 DB44 SON 完成 */
    b->state = !fault && b->stop_result == MODBUS_OK && b->disable_result == MODBUS_OK
        ? BENCH_DISABLED : BENCH_FAULT;
    if (fault && address != 0xffffU) {
        b->failed_address = address; b->actual = actual; b->expected = expected;
        b->exception_code = exception;
    }
}

void p100s_bench_init(p100s_bench_t *b) {
    memset(b, 0, sizeof(*b));
    b->rpm = 50;
    b->failed_address = 0xffffU;
    b->poll_result = MODBUS_ERROR_ARGUMENT;
}

void p100s_bench_command(p100s_bench_t *b, p100s_handle_t *m, uint32_t now) {
    uint32_t cmd = b->command;
    int32_t rpm = b->rpm;
    b->command = BENCH_IDLE;
    if (cmd == BENCH_IDLE) return;
    b->failed_address = 0xffffU;
    b->exception_code = 0;
    b->result = MODBUS_ERROR_ARGUMENT;
    if (cmd == BENCH_DISABLE) {
        release(b, m, 0);
        b->result = b->stop_result != MODBUS_OK ? b->stop_result : b->disable_result;
        return;
    }
    if (cmd == BENCH_STOP) {
        b->result = write_pa(b, m, 24, 0);
        if (b->result != MODBUS_OK) release(b, m, 1);
        else { b->state = BENCH_STOPPING; b->deadline_start = now; }
        return;
    }
    if (cmd == BENCH_CHECK) {
        /* 运动期间不进行耗时的全参数检查。 */
        if (b->state == BENCH_ENABLED || b->state == BENCH_RUNNING || b->state == BENCH_STOPPING) return;
        b->result = check(b, m, 0);
        b->state = b->result == MODBUS_OK ? BENCH_DISABLED : BENCH_UNKNOWN;
        return;
    }
    if (cmd == BENCH_ENABLE && b->state == BENCH_DISABLED) {
        b->result = check(b, m, 0);
        if (b->result != MODBUS_OK) { b->state = BENCH_UNKNOWN; return; }
        b->result = write_pa(b, m, 24, 0);
        if (b->result == MODBUS_OK) b->result = expect(b, m, 0x1016, 3);
        if (b->result != MODBUS_OK) { b->state = BENCH_DISABLED; }
        else { b->state = BENCH_ENABLED; b->deadline_start = now; }
        return;
    }
    if (cmd == BENCH_RUN && b->state == BENCH_ENABLED && rpm != 0 && rpm >= -100 && rpm <= 100) {
        b->result = check(b, m, 1);
        if (b->result == MODBUS_OK) b->result = write_pa(b, m, 24, (uint16_t)rpm);
        if (b->result != MODBUS_OK) release(b, m, 1);
        else { b->state = BENCH_RUNNING; b->deadline_start = now; }
    }
}

void p100s_bench_poll(p100s_bench_t *b, p100s_handle_t *m, uint32_t now) {
    b->poll_result = p100s_read_status(m);
    if (b->poll_result == MODBUS_OK) b->poll_result = read_reg(m, 0x1016, &b->run_state);
    uint8_t active = b->state == BENCH_ENABLED || b->state == BENCH_RUNNING || b->state == BENCH_STOPPING;
    if (!active) return;
    if (b->poll_result != MODBUS_OK || m->status.alarm_code != 0 || b->run_state != 3) {
        release(b, m, 1);
        return;
    }
    uint32_t elapsed = now - b->deadline_start;
    if (b->state == BENCH_RUNNING && elapsed >= 2000U) {
        b->stop_result = write_pa(b, m, 24, 0);
        if (b->stop_result != MODBUS_OK) release(b, m, 1);
        else { b->state = BENCH_STOPPING; b->deadline_start = now; }
    } else if (b->state == BENCH_STOPPING) {
        if (m->status.speed_raw >= -5 && m->status.speed_raw <= 5) release(b, m, 0);
        else if (elapsed >= 3000U) release(b, m, 1);
    }
}
