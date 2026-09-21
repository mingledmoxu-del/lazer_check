#ifndef PC_COMM_H
#define PC_COMM_H
#include "stm32h7xx_hal.h"
#include "pc_protocol.h"

HAL_StatusTypeDef PC_Comm_Init(UART_HandleTypeDef *uart, pc_protocol_t *protocol);
HAL_StatusTypeDef PC_Comm_MspInit(UART_HandleTypeDef *uart);
void PC_Comm_MspDeInit(UART_HandleTypeDef *uart);
void PC_Comm_DMA_IRQHandler(void);
void PC_Comm_Task(void);
void PC_Comm_RxEvent(UART_HandleTypeDef *uart, uint16_t size);
void PC_Comm_Error(UART_HandleTypeDef *uart);
HAL_StatusTypeDef PC_Comm_SendResponse(uint8_t device_id, uint8_t command, pc_status_t status, uint16_t parameter, int16_t value);
HAL_StatusTypeDef PC_Comm_SendResponseData(uint8_t device_id, uint8_t command, pc_status_t status,
                                           uint16_t parameter, const uint8_t *data, uint8_t data_length);

#endif
