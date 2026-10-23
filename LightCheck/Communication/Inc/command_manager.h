#ifndef COMMAND_MANAGER_H
#define COMMAND_MANAGER_H
#include "pc_protocol.h"
#include "p100s.h"
#include "stm32h7xx_hal.h"
#include "limit_switch.h"
#include "motor_config.h"
#include "axis_guard.h"
/* All sessions share one controller; a transport loss latches stops for both axes. */
#include "trajectory.h"
#define POINT_BANK_SLOTS PC_POINT_BANK_SLOTS
#define POINT_BANK_CAPACITY 16U
typedef struct {
    uint32_t token;
    uint16_t speed,tolerance;
    uint8_t count,loaded,ready;
    int32_t points[POINT_BANK_CAPACITY][2];
    int32_t origin[2],reference[2];
    uint16_t ppr[2],ratio_num[2],ratio_den[2];
    int8_t sign[2];
} point_bank_t;

typedef struct command_manager {
    p100s_handle_t *motors[2];
    UART_HandleTypeDef *pc_uart;
    int16_t target_speed[2];
    uint16_t gear_ratio_num[2];
    uint16_t gear_ratio_den[2];
    uint8_t servo_enabled[2];
    uint8_t busy;
    uint8_t point_range_unsupported[2]; /* Capability only; never caches parameter values. */
    uint8_t point_preflight; /* This dispatch will perform its own fresh idle-axis checks. */
    limit_switch_t limits[2];
    uint32_t limit_handled[2];
    uint32_t limit_retry_at[2];
    uint8_t limit_bypass[2];
    uint32_t limit_bypass_until[2];
    uint8_t continuous_active[2];
    uint8_t continuous_stop_pending[2];
    uint32_t continuous_until[2];
    uint32_t continuous_retry_at[2];
    /* Verify the initial encoder direction of every internal position task. */
    uint8_t position_watch[2];
    int8_t position_direction[2];
    int32_t position_start[2];
    uint16_t position_noise[2];
    uint32_t position_started_at[2], position_poll_at[2];
    uint8_t point_active[2], point_zero_samples[2];
    uint8_t point_settling[2], point_completed[2];
    int32_t point_target[2], point_last[2], point_progress_position[2];
    int16_t point_max_rpm[2], point_sent_rpm[2];
    uint16_t point_ppr[2];
    uint32_t point_tolerance[2];
    uint16_t point_decel_ms[2], point_s_curve_ms[2];
    uint32_t point_sample_at[2], point_units_at[2];
    uint8_t point_sample_valid[2];
    uint32_t point_poll_at[2], point_progress_at[2], point_deadline[2];
    uint32_t point_init_ms[2],point_cycle_started[2],point_cycle_ms[2],point_gap_ms[2],point_max_gap_ms[2];
    guard_inputs_t guard_inputs;
    axis_guard_t guard[2];
    trajectory_t trajectory;
    point_bank_t point_banks[POINT_BANK_SLOTS];
} command_manager_t;
void Command_Manager_Init(command_manager_t *manager, p100s_handle_t *motor1, p100s_handle_t *motor2, UART_HandleTypeDef *pc_uart);
void Command_Process(void *context, const pc_command_packet_t *command);
/* Sample only in the 1 ms SysTick interrupt; Modbus runs in the main loop. */
void Command_Manager_SampleLimits(command_manager_t *manager, uint8_t limit1, uint8_t limit2);
void Command_Manager_Task(command_manager_t *manager);
uint16_t Command_Manager_PointFlags(command_manager_t *m, uint8_t index);
void Command_Manager_SampleInputs(command_manager_t *manager, uint8_t raw_mask);
modbus_result_t Command_Manager_PrepareVelocity(command_manager_t *m, uint8_t index);
modbus_result_t Command_Manager_UpdateVelocity(command_manager_t *m, uint8_t index, int16_t rpm);

void Command_Manager_TransportLost(command_manager_t *manager);
#endif
