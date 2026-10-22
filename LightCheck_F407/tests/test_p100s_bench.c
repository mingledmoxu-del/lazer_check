#include "p100s_bench.h"
#include <assert.h>
#include <string.h>

static uint16_t regs[0x1020];
static unsigned writes;
static uint16_t fail_write;
static uint16_t fail_read;
static uint8_t ignore_write;

static modbus_result_t exchange(void *ctx, const uint8_t *tx, uint16_t size,
    uint8_t *rx, uint16_t capacity, uint16_t *out, uint32_t timeout) {
    (void)ctx; (void)capacity; (void)timeout;
    assert(size == 8);
    uint16_t a = (uint16_t)(tx[2] << 8 | tx[3]);
    uint16_t v = (uint16_t)(tx[4] << 8 | tx[5]);
    rx[0] = tx[0]; rx[1] = tx[1];
    if (tx[1] == 6) {
        ++writes;
        assert(a == 0x98 || a == 0xb5); /* 不允许任何 EEPROM 写入 */
        if (!ignore_write) {
            regs[a - 0x80] = v;
            if (a == 0xb5) regs[0x1016] = v ? 3 : 1;
        }
        if (a == fail_write) return MODBUS_ERROR_TIMEOUT; /* 已执行但回包丢失 */
        memcpy(rx, tx, 6); *out = 8;
    } else {
        assert(tx[1] == 3);
        if (a == fail_read) return MODBUS_ERROR_TIMEOUT;
        rx[2] = (uint8_t)(2*v);
        for (unsigned i=0; i<v; ++i) {
            rx[3+2*i] = (uint8_t)(regs[a+i] >> 8);
            rx[4+2*i] = (uint8_t)regs[a+i];
        }
        *out = (uint16_t)(5+2*v);
    }
    uint16_t crc = modbus_crc16(rx, *out-2);
    rx[*out-2] = (uint8_t)crc; rx[*out-1] = (uint8_t)(crc >> 8);
    return MODBUS_OK;
}

static void setup(p100s_bench_t *b, p100s_handle_t *m, modbus_handle_t *bus) {
    memset(regs,0,sizeof(regs)); writes=0; fail_write=0xffff; fail_read=0xffff; ignore_write=0;
    regs[71]=1; regs[72]=96; regs[4]=1; regs[22]=1; regs[40]=100; regs[41]=100;
    for (unsigned i=0;i<4;++i) regs[0x100+i]=(uint16_t)(i+1);
    regs[0x1009]=1; regs[0x1016]=1;
    assert(modbus_init(bus,0,exchange,400)==MODBUS_OK);
    assert(p100s_init(m,bus,1)==MODBUS_OK);
    p100s_bench_init(b);
}
static void command(p100s_bench_t *b,p100s_handle_t *m,unsigned cmd,unsigned now) {
    b->command=cmd; p100s_bench_command(b,m,now); assert(b->command==0);
}
static void arm(p100s_bench_t *b,p100s_handle_t *m) {
    command(b,m,BENCH_CHECK,0); assert(b->state==BENCH_DISABLED);
    command(b,m,BENCH_ENABLE,0); assert(b->state==BENCH_ENABLED);
}
int main(void) {
    p100s_bench_t b; p100s_handle_t m; modbus_handle_t bus;
    setup(&b,&m,&bus);
    p100s_bench_poll(&b,&m,0); assert(writes==0);
    command(&b,&m,BENCH_RUN,0); assert(writes==0);
    regs[73]=1; command(&b,&m,BENCH_CHECK,0);
    assert(b.result!=MODBUS_OK && b.failed_address==73 && writes==0);
    setup(&b,&m,&bus); regs[0x10f]=1; command(&b,&m,BENCH_CHECK,0);
    assert(b.failed_address==0x10f && writes==0);
    setup(&b,&m,&bus); regs[24]=50; command(&b,&m,BENCH_CHECK,0);
    assert(b.failed_address==24 && writes==0);
    setup(&b,&m,&bus); arm(&b,&m);
    b.rpm=101; unsigned before=writes; command(&b,&m,BENCH_RUN,0); assert(writes==before);
    b.rpm=-50; command(&b,&m,BENCH_RUN,0);
    assert(b.state==BENCH_RUNNING && regs[24]==(uint16_t)-50);
    regs[0x1000]=(uint16_t)-50; p100s_bench_poll(&b,&m,2000);
    assert(b.state==BENCH_STOPPING && regs[24]==0 && regs[53]==1);
    regs[0x1000]=0; p100s_bench_poll(&b,&m,2100);
    assert(b.state==BENCH_DISABLED && regs[53]==0);
    setup(&b,&m,&bus); arm(&b,&m); fail_write=0x98;
    command(&b,&m,BENCH_RUN,0);
    assert(b.state==BENCH_FAULT && regs[53]==0 && b.result==MODBUS_ERROR_TIMEOUT);
    setup(&b,&m,&bus); arm(&b,&m); fail_read=0x1000;
    p100s_bench_poll(&b,&m,100);
    assert(b.state==BENCH_FAULT && regs[53]==0 && b.poll_result==MODBUS_ERROR_TIMEOUT);
    setup(&b,&m,&bus); arm(&b,&m); regs[0x1013]=9;
    p100s_bench_poll(&b,&m,100); assert(b.state==BENCH_FAULT && regs[53]==0);
    setup(&b,&m,&bus); arm(&b,&m); p100s_bench_poll(&b,&m,15000);
    assert(b.state==BENCH_STOPPING);
    setup(&b,&m,&bus); command(&b,&m,BENCH_CHECK,0); ignore_write=1;
    command(&b,&m,BENCH_ENABLE,0); assert(b.state==BENCH_FAULT);
    setup(&b,&m,&bus); arm(&b,&m); fail_write=0xb5;
    command(&b,&m,BENCH_DISABLE,0); assert(b.state==BENCH_FAULT && b.disable_result==MODBUS_ERROR_TIMEOUT);
    setup(&b,&m,&bus); arm(&b,&m); command(&b,&m,BENCH_STOP,0);
    regs[0x1000]=50; p100s_bench_poll(&b,&m,3000);
    assert(b.state==BENCH_FAULT && regs[53]==0);
    return 0;
}
