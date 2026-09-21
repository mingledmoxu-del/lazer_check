#include "command_manager.h"
#include "pc_comm.h"
#include <limits.h>
#include <string.h>

#define P100S_PA_CONTROL_MODE       4U
#define P100S_PA_PULSES_PER_REV    11U
#define P100S_PA_COMMAND_FORMAT    14U
#define P100S_PA_SPEED_SOURCE      22U
#define P100S_PA_INTERNAL_SPEED_1  24U
#define P100S_PA_FORCE_ENABLE      53U
#define P100S_PA_ALARM_CLEAR       61U
#define P100S_REG_CONTROL_MODE 0x1009U
#define P100S_REG_RUN_STATE     0x1016U
#define P100S_P3_FORCE_2          16U
#define P100S_P3_FORCE_4          18U
#define P100S_P4_MODE              0U
#define P100S_P4_P1_TURNS          2U
#define P100S_P4_P1_PULSES         3U
#define P100S_P4_P1_SPEED          4U
#define P100S_FORCE_CMODE      0x80U
#define P100S_FORCE_POS2       0x40U
#define P100S_FORCE_POS1       0x20U
#define P100S_FORCE_POS0       0x10U
#define P100S_FORCE_HOLD       0x04U
#define P100S_FORCE_CTRG       0x08U
#define P100S_FORCE_POSITION_MASK \
    (P100S_FORCE_POS2 | P100S_FORCE_POS1 | P100S_FORCE_POS0 | P100S_FORCE_CTRG | P100S_FORCE_HOLD)

static pc_status_t map_status(modbus_result_t r)
{
    switch (r) {
    case MODBUS_OK: return PC_STATUS_SUCCESS;
    case MODBUS_ERROR_TIMEOUT: return PC_STATUS_SERVO_NO_RESPONSE;
    case MODBUS_ERROR_CRC: return PC_STATUS_CRC_ERROR;
    case MODBUS_ERROR_BUSY: return PC_STATUS_BUSY;
    case MODBUS_ERROR_ARGUMENT: return PC_STATUS_ILLEGAL_PARAM;
    default: return PC_STATUS_RS485_ERROR;
    }
}

static uint8_t is_supported(uint8_t command)
{
    return (uint8_t)((command >= PC_CMD_WRITE_PA && command <= PC_CMD_EMERGENCY_STOP) ||
                     (command >= PC_CMD_SET_SPEED && command <= PC_CMD_JOG));
}

static int16_t read_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static int32_t read_i32(const uint8_t *data)
{
    uint32_t raw = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) | data[3];
    return (int32_t)raw;
}

static void put_i32(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw >> 24); data[1] = (uint8_t)(raw >> 16);
    data[2] = (uint8_t)(raw >> 8);  data[3] = (uint8_t)raw;
}

static modbus_result_t read_register(p100s_handle_t *motor, uint16_t address, uint16_t *value)
{
    if (motor == NULL || motor->bus == NULL || value == NULL) return MODBUS_ERROR_ARGUMENT;
    return modbus_read_holding(motor->bus, motor->slave_id, address, 1U, value);
}

static modbus_result_t write_pa_temp(p100s_handle_t *motor, uint16_t number, uint16_t value)
{
    return p100s_write_parameter(motor, P100S_GROUP_PA, number, value, P100S_STORAGE_TEMPORARY);
}

static modbus_result_t update_forced_input(p100s_handle_t *motor, uint16_t number, uint16_t mask, uint8_t enabled)
{
    uint16_t value = 0U;
    modbus_result_t result = p100s_read_parameter(motor, P100S_GROUP_P3, number, &value);
    if (result != MODBUS_OK) return result;
    uint16_t updated = enabled != 0U ? (uint16_t)(value | mask) : (uint16_t)(value & (uint16_t)~mask);
    if (updated == value) return MODBUS_OK;
    return p100s_write_parameter(motor, P100S_GROUP_P3, number, updated, P100S_STORAGE_SAVED);
}

static modbus_result_t select_mixed_control_mode(p100s_handle_t *motor, uint8_t speed_mode)
{
    modbus_result_t result = write_pa_temp(motor, P100S_PA_CONTROL_MODE, 3U);
    if (result == MODBUS_OK)
        result = update_forced_input(motor, P100S_P3_FORCE_2, P100S_FORCE_CMODE, speed_mode);
    return result;
}

static modbus_result_t trigger_internal_position_1(p100s_handle_t *motor)
{
    uint16_t value = 0U;
    modbus_result_t result = p100s_read_parameter(motor, P100S_GROUP_P3, P100S_P3_FORCE_4, &value);
    if (result != MODBUS_OK) return result;

    /* Select P1 (POS2..0 = 000), release HOLD and reset CTRG first. */
    uint16_t idle = (uint16_t)(value & (uint16_t)~P100S_FORCE_POSITION_MASK);
    result = p100s_write_parameter(motor, P100S_GROUP_P3, P100S_P3_FORCE_4,
                                   idle, P100S_STORAGE_SAVED);
    /* Keep CTRG high after the rising edge. The next command resets it before
       generating a new edge, avoiding a pulse shorter than the drive scan. */
    if (result == MODBUS_OK)
        result = p100s_write_parameter(motor, P100S_GROUP_P3, P100S_P3_FORCE_4,
                                       (uint16_t)(idle | P100S_FORCE_CTRG), P100S_STORAGE_SAVED);
    return result;
}

static modbus_result_t reset_alarm(p100s_handle_t *motor)
{
    modbus_result_t result = write_pa_temp(motor, P100S_PA_ALARM_CLEAR, 0U);
    if (result == MODBUS_OK) result = write_pa_temp(motor, P100S_PA_ALARM_CLEAR, 1U);
    if (result == MODBUS_OK) result = write_pa_temp(motor, P100S_PA_ALARM_CLEAR, 0U);
    return result;
}

static modbus_result_t stop_motor(p100s_handle_t *motor)
{
    uint16_t mode = 0U;
    modbus_result_t result = read_register(motor, P100S_REG_CONTROL_MODE, &mode);
    if (result != MODBUS_OK) return result;
    if (mode == 1U) return write_pa_temp(motor, P100S_PA_INTERNAL_SPEED_1, 0U);
    return update_forced_input(motor, P100S_P3_FORCE_4, P100S_FORCE_HOLD, 1U);
}

static modbus_result_t start_speed(command_manager_t *manager, uint8_t index, p100s_handle_t *motor)
{
    int16_t speed = manager->target_speed[index];
    if (speed == 0 || speed < -5000 || speed > 5000)
        return MODBUS_ERROR_ARGUMENT;
    /* External SON is the motion authority. 0x1016 is telemetry and some
       drive revisions do not report state 3 until motion has started. */
    modbus_result_t result = select_mixed_control_mode(motor, 1U);
    if (result == MODBUS_OK) result = write_pa_temp(motor, P100S_PA_SPEED_SOURCE, 1U);
    if (result == MODBUS_OK) result = write_pa_temp(motor, P100S_PA_INTERNAL_SPEED_1, (uint16_t)speed);
    return result;
}

static modbus_result_t move_position(command_manager_t *manager, uint8_t index, p100s_handle_t *motor,
                                    uint8_t absolute, int16_t speed, int32_t pulses, int32_t *reported_position)
{
    (void)manager;
    (void)index;
    if (speed == 0 || speed < -5000 || speed > 5000)
        return MODBUS_ERROR_ARGUMENT;
    uint16_t ppr = 0U;
    /* Do not gate motion on 0x1016. The physical external SON input decides
       whether torque is enabled; run state remains available to the PC UI. */
    modbus_result_t result = p100s_read_parameter(motor, P100S_GROUP_PA, P100S_PA_PULSES_PER_REV, &ppr);
    if (result != MODBUS_OK || ppr == 0U) return result == MODBUS_OK ? MODBUS_ERROR_ARGUMENT : result;

    int32_t turns = pulses / (int32_t)ppr;
    int32_t remainder = pulses % (int32_t)ppr;
    if (turns < -30000 || turns > 30000) return MODBUS_ERROR_ARGUMENT;

    int32_t current_position = 0;
    int64_t target = pulses;
    if (absolute == 0U) {
        result = p100s_read_status(motor);
        if (result != MODBUS_OK) return result;
        current_position = motor->status.position;
        target = (int64_t)current_position + pulses;
    }
    if (target > INT32_MAX || target < INT32_MIN) return MODBUS_ERROR_ARGUMENT;

    result = select_mixed_control_mode(motor, 0U);
    if (result == MODBUS_OK) result = write_pa_temp(motor, P100S_PA_COMMAND_FORMAT, 3U);
    if (result == MODBUS_OK) result = p100s_write_parameter(motor, P100S_GROUP_P4, P100S_P4_MODE,
                                                            absolute != 0U ? 0U : 1U, P100S_STORAGE_SAVED);
    if (result == MODBUS_OK) result = p100s_write_parameter(motor, P100S_GROUP_P4, P100S_P4_P1_TURNS,
                                                            (uint16_t)turns, P100S_STORAGE_SAVED);
    if (result == MODBUS_OK) result = p100s_write_parameter(motor, P100S_GROUP_P4, P100S_P4_P1_PULSES,
                                                            (uint16_t)remainder, P100S_STORAGE_SAVED);
    if (result == MODBUS_OK) result = p100s_write_parameter(motor, P100S_GROUP_P4, P100S_P4_P1_SPEED,
                                                            (uint16_t)(speed < 0 ? -speed : speed), P100S_STORAGE_SAVED);
    if (result == MODBUS_OK) result = trigger_internal_position_1(motor);
    if (result == MODBUS_OK) *reported_position = (int32_t)target;
    return result;
}

void Command_Manager_Init(command_manager_t *m, p100s_handle_t *m1, p100s_handle_t *m2, UART_HandleTypeDef *uart)
{
    if (m == NULL) return;
    memset(m, 0, sizeof(*m));
    m->motors[0] = m1; m->motors[1] = m2; m->pc_uart = uart;
}

void Command_Process(void *context, const pc_command_packet_t *c)
{
    command_manager_t *m = (command_manager_t *)context;
    if (m == NULL || c == NULL) return;
    if (c->device_id < 1U || c->device_id > 2U) {
        (void)PC_Comm_SendResponse(c->device_id, c->command, PC_STATUS_ILLEGAL_PARAM, c->parameter, 0);
        return;
    }
    if (!is_supported(c->command)) {
        (void)PC_Comm_SendResponse(c->device_id, c->command, PC_STATUS_UNKNOWN_CMD, c->parameter, 0);
        return;
    }
    if (m->busy != 0U) {
        (void)PC_Comm_SendResponse(c->device_id, c->command, PC_STATUS_BUSY, c->parameter, 0);
        return;
    }
    if (c->valid_payload == 0U) {
        (void)PC_Comm_SendResponse(c->device_id, c->command, PC_STATUS_ILLEGAL_PARAM, c->parameter, 0);
        return;
    }

    m->busy = 1U;
    uint8_t index = (uint8_t)(c->device_id - 1U);
    p100s_handle_t *motor = m->motors[index];
    modbus_result_t result = MODBUS_ERROR_ARGUMENT;
    int16_t value = 0;
    int32_t position = 0;
    uint8_t response_data[4];
    uint8_t response_length = 2U;
    uint8_t response_is_raw = 0U;

    switch (c->command) {
    case PC_CMD_READ_PA: {
        uint16_t raw = 0U;
        result = p100s_read_parameter(motor, P100S_GROUP_PA, c->parameter, &raw);
        value = (int16_t)raw;
        break;
    }
    case PC_CMD_WRITE_PA:
        if (c->save_mode <= 1U) {
            value = c->value;
            result = p100s_write_parameter(motor, P100S_GROUP_PA, c->parameter, (uint16_t)c->value,
                                            c->save_mode == 1U ? P100S_STORAGE_SAVED : P100S_STORAGE_TEMPORARY);
        }
        break;
    case PC_CMD_SERVO_ON:
        result = write_pa_temp(motor, P100S_PA_FORCE_ENABLE, 1U);
        if (result == MODBUS_OK) m->servo_enabled[index] = 1U;
        value = 1;
        break;
    case PC_CMD_SERVO_OFF: {
        modbus_result_t stop_result = stop_motor(motor);
        result = write_pa_temp(motor, P100S_PA_FORCE_ENABLE, 0U);
        if (result == MODBUS_OK) m->servo_enabled[index] = 0U;
        if (stop_result != MODBUS_OK) result = stop_result;
        break;
    }
    case PC_CMD_SET_SPEED:
        value = read_i16(c->data);
        if (value >= -5000 && value <= 5000) {
            m->target_speed[index] = value;
            result = p100s_read_status(motor);
        }
        break;
    case PC_CMD_MOTOR_START:
        result = start_speed(m, index, motor);
        value = m->target_speed[index];
        break;
    case PC_CMD_MOTOR_STOP:
        result = stop_motor(motor);
        break;
    case PC_CMD_READ_SPEED: {
        uint16_t run_state = 0U;
        result = p100s_read_status(motor);
        if (result == MODBUS_OK) result = read_register(motor, P100S_REG_RUN_STATE, &run_state);
        if (result == MODBUS_OK) {
            value = motor->status.speed_raw;
            response_data[0] = (uint8_t)((uint16_t)value >> 8);
            response_data[1] = (uint8_t)value;
            response_data[2] = (uint8_t)(run_state >> 8);
            response_data[3] = (uint8_t)run_state;
            response_length = 4U;
            response_is_raw = 1U;
        }
        break;
    }
    case PC_CMD_READ_POSITION:
        result = p100s_read_status(motor);
        if (result == MODBUS_OK) position = motor->status.position;
        response_length = 4U;
        break;
    case PC_CMD_READ_ALARM: {
        uint16_t alarm = 0U;
        result = p100s_read_alarm(motor, &alarm);
        value = (int16_t)alarm;
        break;
    }
    case PC_CMD_RESET_ALARM:
        result = reset_alarm(motor);
        break;
    case PC_CMD_EMERGENCY_STOP: {
        modbus_result_t stop_result = stop_motor(motor);
        result = write_pa_temp(motor, P100S_PA_FORCE_ENABLE, 0U);
        m->servo_enabled[index] = 0U;
        m->target_speed[index] = 0;
        if (stop_result != MODBUS_OK) result = stop_result;
        break;
    }
    case PC_CMD_MOVE_RELATIVE:
    case PC_CMD_MOVE_ABSOLUTE:
        value = read_i16(c->data);
        result = move_position(m, index, motor, (uint8_t)(c->command == PC_CMD_MOVE_ABSOLUTE),
                               value, read_i32(c->data + 2U), &position);
        response_length = 4U;
        break;
    case PC_CMD_JOG: {
        int16_t speed = read_i16(c->data + 1U);
        int32_t step = read_i32(c->data + 3U);
        if (step == INT32_MIN) { result = MODBUS_ERROR_ARGUMENT; break; }
        if (step < 0) step = -step;
        if (c->data[0] == 0U) step = -step;
        else if (c->data[0] != 1U) { result = MODBUS_ERROR_ARGUMENT; break; }
        result = move_position(m, index, motor, 0U, speed, step, &position);
        response_length = 4U;
        break;
    }
    default:
        result = MODBUS_ERROR_ARGUMENT;
        break;
    }

    if (response_is_raw == 0U) {
        if (response_length == 4U) put_i32(response_data, position);
        else { response_data[0] = (uint8_t)((uint16_t)value >> 8); response_data[1] = (uint8_t)value; }
    }
    (void)PC_Comm_SendResponseData(c->device_id, c->command, map_status(result), c->parameter,
                                   response_data, response_length);
    m->busy = 0U;
}
