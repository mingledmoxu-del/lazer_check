#include "p100s_console.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CONSOLE_LINE 128U
static UART_HandleTypeDef *g_uart;
static p100s_bench_t *g_bench;
static uint8_t g_rx;
static volatile uint8_t g_line[CONSOLE_LINE];
static volatile uint16_t g_len;
static volatile uint8_t g_ready;
static volatile uint8_t g_overflow;
static uint32_t g_last_status;

static void reply(uint32_t id, const char *status, const char *tail) {
    char out[192];
    int n = snprintf(out, sizeof(out), "@R %lu %s %s\r\n", (unsigned long)id, status, tail ? tail : "");
    if (n > 0 && g_uart) HAL_UART_Transmit(g_uart, (uint8_t *)out, (uint16_t)n, 100U);
}
static int field_i32(const char *s, const char *key, int32_t *v) {
    const char *p = strstr(s, key); char *end;
    if (!p || (p != s && p[-1] != ' ')) return 0;
    p += strlen(key); if (*p != '=') return 0; *v = strtol(p + 1, &end, 10); return end != p + 1;
}
void p100s_console_init(UART_HandleTypeDef *uart, p100s_bench_t *bench) {
    g_uart=uart; g_bench=bench; g_len=0; g_ready=0; g_overflow=0; g_last_status=0;
    HAL_UART_Receive_IT(g_uart, &g_rx, 1U);
}
void p100s_console_rx_complete(UART_HandleTypeDef *huart) {
    if (huart != g_uart) return;
    uint8_t c=g_rx;
    if (!g_ready) {
        if (c == '\n') { if (g_len) { g_line[g_len]=0; g_ready=1; } }
        else if (c != '\r') { if (g_len < CONSOLE_LINE-1U) g_line[g_len++]=c; else g_overflow=1; }
    }
    HAL_UART_Receive_IT(g_uart, &g_rx, 1U);
}
static void periodic_status(p100s_handle_t *m, uint32_t now) {
    if (now-g_last_status < 500U) return;
    g_last_status=now;
    if (m->status_valid) {
        char out[256]; int n=snprintf(out,sizeof(out),"@S 0 OK valid=1 rpm=%d position=%ld command_position=%ld position_error=%ld inputs=0x%04X outputs=0x%04X alarm=%u drive=%u state=%lu\r\n",(int)m->status.speed_raw,(long)m->status.position,(long)m->status.command_position,(long)m->status.position_error,(unsigned)m->status.input_state,(unsigned)m->status.output_state,(unsigned)m->status.alarm_code,(unsigned)g_bench->run_state,(unsigned long)g_bench->state);
        HAL_UART_Transmit(g_uart,(uint8_t*)out,(uint16_t)n,100U);
    } else HAL_UART_Transmit(g_uart,(uint8_t*)"@S 0 FAIL valid=0 age_ms=999999\r\n",35U,100U);
}
void p100s_console_poll(p100s_handle_t *m, uint32_t now) {
    periodic_status(m, now); if (!g_ready) return;
    char line[CONSOLE_LINE]; __disable_irq(); memcpy(line,(const void*)g_line,g_len+1U); uint16_t len=g_len; g_len=0; g_ready=0; uint8_t ov=g_overflow; g_overflow=0; __enable_irq();
    if (ov || len >= CONSOLE_LINE-1U) { reply(0,"REJECT","code=LINE_TOO_LONG"); return; }
    if (strncmp(line,"@C ",3)!=0) { reply(0,"REJECT","code=BAD_FRAME"); return; }
    char *p=line+3, *end; uint32_t id=strtoul(p,&end,10); if (end==p || *end!=' ') { reply(0,"REJECT","code=BAD_ID"); return; }
    while (*end==' ') ++end;
    char cmd[16]={0};
    if (sscanf(end,"%15s",cmd)!=1) { reply(id,"REJECT","code=BAD_COMMAND"); return; }
    uint32_t c=0; int32_t rpm=0;
    if (!strcmp(cmd,"STATUS")) { p100s_bench_poll(g_bench,m,now); reply(id,"OK","queued=0"); return; }
    if (!strcmp(cmd,"CHECK")) c=BENCH_CHECK; else if (!strcmp(cmd,"ENABLE")) c=BENCH_ENABLE; else if (!strcmp(cmd,"RUN")) { if (!field_i32(end,"rpm",&rpm)||rpm==0||rpm<-100||rpm>100) { reply(id,"REJECT","code=RPM_RANGE"); return; } g_bench->rpm=rpm; c=BENCH_RUN; } else if (!strcmp(cmd,"STOP")) c=BENCH_STOP; else if (!strcmp(cmd,"DISABLE")) c=BENCH_DISABLE; else { reply(id,"REJECT","code=BAD_COMMAND"); return; }
    if (c!=BENCH_STOP && c!=BENCH_DISABLE && g_bench->command!=BENCH_IDLE) { reply(id,"REJECT","code=BUSY"); return; }
    g_bench->command=c; p100s_bench_command(g_bench,m,now);
    char tail[120]; snprintf(tail,sizeof(tail),"result=%d state=%lu addr=0x%04X actual=%u expected=%u ex=%u",(int)g_bench->result,(unsigned long)g_bench->state,(unsigned)g_bench->failed_address,(unsigned)g_bench->actual,(unsigned)g_bench->expected,(unsigned)g_bench->exception_code);
    reply(id,g_bench->result==MODBUS_OK?"OK":"FAIL",tail);
}
