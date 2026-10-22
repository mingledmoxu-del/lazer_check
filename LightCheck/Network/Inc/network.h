#ifndef NETWORK_H
#define NETWORK_H
#include "command_manager.h"
#include "pc_protocol.h"
#include "stm32h7xx_hal.h"
HAL_StatusTypeDef Network_Init(pc_protocol_t *protocol, command_manager_t *manager);
void Network_Task(void);
HAL_StatusTypeDef Network_Send(const uint8_t *data,uint16_t length);
#endif
