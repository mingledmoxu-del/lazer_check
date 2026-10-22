#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H
#include "pc_protocol.h"
#include "p100s.h"
#include "stm32f4xx_hal.h"
#include "limit_switch.h"
#include "motor_config.h"
#include "axis_guard.h"
#include "trajectory.h"

typedef struct command_manager {
    p100s_handle_t *motors[2];
    UART_HandleTypeDef *pc_uart;
    int16_t target_speed[2];
    uint16_t gear_ratio_num[2];
    uint16_t gear_ratio_den[2];
    uint8_t servo_enabled[2];
    uint8_t busy;
    limit_switch_t limits[2];
    uint32_t limit_handled[2];
    uint32_t limit_retry_at[2];
    uint8_t limit_bypass[2];
    uint32_t limit_bypass_until[2];
    uint8_t continuous_active[2];
    uint8_t continuous_stop_pending[2];
    uint32_t continuous_until[2];
    uint32_t continuous_retry_at[2];
    guard_inputs_t guard_inputs;
    axis_guard_t guard[2];
    trajectory_t trajectory;
} command_manager_t;
void Command_Manager_Init(command_manager_t *manager, p100s_handle_t *motor1, p100s_handle_t *motor2, UART_HandleTypeDef *pc_uart);
void Command_Process(void *context, const pc_command_packet_t *command);
/* Sample only in the 1 ms SysTick interrupt; Modbus runs in the main loop. */
void Command_Manager_SampleLimits(command_manager_t *manager, uint8_t limit1, uint8_t limit2);
void Command_Manager_Task(command_manager_t *manager);
void Command_Manager_SampleInputs(command_manager_t *manager, uint8_t raw_mask);
modbus_result_t Command_Manager_PrepareVelocity(command_manager_t *m, uint8_t index);
modbus_result_t Command_Manager_UpdateVelocity(command_manager_t *m, uint8_t index, int16_t rpm);

#endif
