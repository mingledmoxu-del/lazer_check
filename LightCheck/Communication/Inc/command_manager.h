#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H
#include "pc_protocol.h"
#include "p100s.h"
#include "stm32h7xx_hal.h"

typedef struct {
    p100s_handle_t *motors[2];
    UART_HandleTypeDef *pc_uart;
    int16_t target_speed[2];
    uint8_t servo_enabled[2];
    uint8_t busy;
} command_manager_t;
void Command_Manager_Init(command_manager_t *manager, p100s_handle_t *motor1, p100s_handle_t *motor2, UART_HandleTypeDef *pc_uart);
void Command_Process(void *context, const pc_command_packet_t *command);

#endif
