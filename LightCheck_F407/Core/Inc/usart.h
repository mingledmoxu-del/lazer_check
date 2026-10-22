#ifndef USART_H
#define USART_H
#include "main.h"
extern UART_HandleTypeDef huart1, huart2;
void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
#endif
