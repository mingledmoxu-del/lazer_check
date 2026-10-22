#ifndef P100S_CONSOLE_H
#define P100S_CONSOLE_H
#include "p100s_bench.h"
#include "usart.h"
void p100s_console_init(UART_HandleTypeDef *uart, p100s_bench_t *bench);
void p100s_console_poll(p100s_handle_t *motor, uint32_t now);
void p100s_console_rx_complete(UART_HandleTypeDef *huart);
#endif
