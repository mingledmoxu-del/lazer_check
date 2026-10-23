#ifndef P100S_H
#define P100S_H

#include "modbus_rtu.h"

typedef enum p100s_parameter_group_e {
    P100S_GROUP_PA = 0x0000,
    P100S_GROUP_P3 = 0x0100,
    P100S_GROUP_P4 = 0x0200
} p100s_parameter_group_t;

typedef enum p100s_storage_e { P100S_STORAGE_SAVED = 0, P100S_STORAGE_TEMPORARY } p100s_storage_t;

typedef struct p100s_status_s {
    int16_t  speed_raw;
    int32_t  position;
    int32_t  command_position;
    int32_t  position_error;
    uint16_t input_state;
    uint16_t output_state;
    uint16_t alarm_code;
    uint16_t control_mode; /* Live 0x1009, sampled in the same status transaction. */
} p100s_status_t;

typedef struct p100s_handle_s {
    modbus_handle_t *bus; /* 同一物理总线上的对象共享此句柄。 */
    uint8_t          slave_id;
    uint8_t          status_valid; /* 最近一次刷新失败时清零，历史数值保留。 */
    uint8_t          exception_code;
    modbus_result_t  last_result;
    p100s_status_t   status;
} p100s_handle_t;

modbus_result_t p100s_init(p100s_handle_t *motor, modbus_handle_t *bus, uint8_t slave_id);
modbus_result_t p100s_read_status(p100s_handle_t *motor);
modbus_result_t p100s_read_alarm(p100s_handle_t *motor, uint16_t *alarm_code);
/* number 范围校验仅防地址跨组，不表示所有编号在固件中均存在。 */
modbus_result_t
p100s_read_parameter(p100s_handle_t *motor, p100s_parameter_group_t group, uint16_t number, uint16_t *value);
modbus_result_t p100s_write_parameter(
    p100s_handle_t *motor, p100s_parameter_group_t group, uint16_t number, uint16_t value, p100s_storage_t storage);

#endif
