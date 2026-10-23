#include "p100s.h"
#include <string.h>

static uint8_t p100s_isvalid(const p100s_handle_t *motor) {
    return motor != NULL && motor->bus != NULL && motor->bus->exchange != NULL && motor->slave_id > 0U &&
           motor->slave_id <= 254U;
}

static modbus_result_t p100s_record(p100s_handle_t *motor, modbus_result_t result) {
    motor->last_result    = result;
    motor->exception_code = result == MODBUS_ERROR_EXCEPTION ? motor->bus->exception_code : 0U;
    return result;
}

/* 显式符号扩展，避免无符号超范围转有符号的实现相关行为。 */
static int32_t p100s_decode_position(const uint16_t *words) {
    int32_t high_word = words[1] >= 0x8000U ? (int32_t)words[1] - 65536 : (int32_t)words[1];
    return high_word * 65536 + words[0];
}

modbus_result_t p100s_init(p100s_handle_t *motor, modbus_handle_t *bus, uint8_t slave_id) {
    if (motor == NULL)
        return MODBUS_ERROR_ARGUMENT;
    memset(motor, 0, sizeof(*motor));
    motor->last_result = MODBUS_ERROR_ARGUMENT;
    if (bus == NULL || bus->exchange == NULL || slave_id == 0U || slave_id > 254U)
        return MODBUS_ERROR_ARGUMENT;
    motor->bus         = bus;
    motor->slave_id    = slave_id;
    motor->last_result = MODBUS_OK;
    return MODBUS_OK;
}

modbus_result_t p100s_read_status(p100s_handle_t *motor) {
    uint16_t values[20];
    if (!p100s_isvalid(motor))
        return MODBUS_ERROR_ARGUMENT;
    motor->status_valid    = 0U;
    modbus_result_t result = modbus_read_holding(motor->bus, motor->slave_id, 0x1000U, 20U, values);
    if (result == MODBUS_OK) {
        motor->status.speed_raw        = (int16_t)(values[0] >= 0x8000U ? (int32_t)values[0] - 65536 : values[0]);
        motor->status.position         = p100s_decode_position(&values[1]);
        motor->status.command_position = p100s_decode_position(&values[3]);
        motor->status.position_error   = p100s_decode_position(&values[5]);
        motor->status.control_mode     = values[9];
        motor->status.input_state      = values[15];
        motor->status.output_state     = values[16];
        motor->status.alarm_code       = values[19];
        motor->status_valid            = 1U;
    }
    return p100s_record(motor, result);
}

modbus_result_t p100s_read_alarm(p100s_handle_t *motor, uint16_t *alarm_code) {
    if (!p100s_isvalid(motor))
        return MODBUS_ERROR_ARGUMENT;
    return p100s_record(motor, modbus_read_holding(motor->bus, motor->slave_id, 0x1013U, 1U, alarm_code));
}

static uint8_t p100s_parameter_isvalid(p100s_parameter_group_t group, uint16_t number) {
    if (group == P100S_GROUP_PA)
        return number < 128U;
    return (group == P100S_GROUP_P3 || group == P100S_GROUP_P4) && number < 256U;
}

modbus_result_t
p100s_read_parameter(p100s_handle_t *motor, p100s_parameter_group_t group, uint16_t number, uint16_t *value) {
    if (!p100s_isvalid(motor))
        return MODBUS_ERROR_ARGUMENT;
    if (!p100s_parameter_isvalid(group, number))
        return p100s_record(motor, MODBUS_ERROR_ARGUMENT);
    return p100s_record(motor, modbus_read_holding(motor->bus, motor->slave_id, (uint16_t)group + number, 1U, value));
}

modbus_result_t p100s_write_parameter(
    p100s_handle_t *motor, p100s_parameter_group_t group, uint16_t number, uint16_t value, p100s_storage_t storage) {
    if (!p100s_isvalid(motor))
        return MODBUS_ERROR_ARGUMENT;
    if (!p100s_parameter_isvalid(group, number) ||
        (storage != P100S_STORAGE_SAVED && storage != P100S_STORAGE_TEMPORARY) ||
        (storage == P100S_STORAGE_TEMPORARY && group != P100S_GROUP_PA))
        return p100s_record(motor, MODBUS_ERROR_ARGUMENT);
    uint16_t address = (uint16_t)group + number;
    if (storage == P100S_STORAGE_TEMPORARY)
        address += 0x0080U;
    return p100s_record(motor, modbus_write_single(motor->bus, motor->slave_id, address, value));
}
