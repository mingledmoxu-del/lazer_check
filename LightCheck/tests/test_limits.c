#include "command_manager.h"
#include "pc_comm.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static command_manager_t manager;
static p100s_handle_t motors[2];
static modbus_handle_t bus;
static uint32_t now;
static unsigned holds[2], triggers[2];
static uint16_t forced[2];
static uint16_t current_mode[2];
static uint16_t forced_speed[2];
static uint16_t speed_parameters[2][6]; /* PA22..27, including every SP bank. */
static uint16_t position_parameters[2][5];
static uint8_t reject_position_parameter;
static uint8_t reject_speed_source;
static uint8_t mapped_speed_input;
static int16_t last_speed[2];
static uint8_t fail_stop, inject_trip;
static pc_status_t reply;
static uint8_t response[40], response_size;
static uint16_t feedback_mode[2];
static uint8_t fail_reads;
static uint8_t lose_trigger_reply;
static uint8_t speed_on_delay;
static uint8_t status_follows_setpoint;
static unsigned status_read_count[2];
static int16_t written_position_pulses[2];

uint32_t HAL_GetTick(void) { return now; }
void HAL_Delay(uint32_t ms) {
    now += ms;
    if (speed_on_delay) { motors[1].status.speed_raw=1;speed_on_delay=0; }
}
modbus_result_t modbus_read_holding(modbus_handle_t *b, uint8_t id,
    uint16_t address, uint16_t count, uint16_t *value)
{
    (void)b; (void)count;
    *value = address == 0x1009U ? current_mode[id - 1U] : 3U;
    return MODBUS_OK;
}
modbus_result_t p100s_read_parameter(p100s_handle_t *m, p100s_parameter_group_t group,
    uint16_t number, uint16_t *value)
{
    *value = group == P100S_GROUP_P3 && number == 32U ? feedback_mode[m->slave_id - 1U] :
             group == P100S_GROUP_P4 && number<5U ? position_parameters[m->slave_id-1U][number] :
             group == P100S_GROUP_PA && number >= 22U && number <= 27U ? speed_parameters[m->slave_id - 1U][number - 22U] :
             group == P100S_GROUP_PA && number == 95U ? 17U :
             group == P100S_GROUP_PA && number == 41U ? 100U :
             group == P100S_GROUP_P3 && number == 16U ? forced_speed[m->slave_id - 1U] :
             group == P100S_GROUP_P3 && number == 0U && mapped_speed_input ? 10U :
             group == P100S_GROUP_P3 && number == 18U ? forced[m->slave_id - 1U] :
             group == P100S_GROUP_PA && number == 11U ? 10000U : 0U;
    return MODBUS_OK;
}
modbus_result_t p100s_write_parameter(p100s_handle_t *m, p100s_parameter_group_t group,
    uint16_t number, uint16_t value, p100s_storage_t storage)
{
    (void)storage;
    unsigned i = m->slave_id - 1U;
    if (group==P100S_GROUP_P4 && number<5U && !reject_position_parameter) position_parameters[i][number]=value;
    if (group == P100S_GROUP_P4 && number == 3U) written_position_pulses[i]=(int16_t)value; /* P4-3 */
    if (group == P100S_GROUP_PA && number >= 22U && number <= 27U) {
        if (number == 22U && value == 1U) {
            /* A speed-source transition must never select a stale bank. */
            for (unsigned bank = 2U; bank < 6U; ++bank)
                assert(speed_parameters[i][bank] == 0U);
            if (reject_speed_source) return MODBUS_OK; /* Write ignored. */
        }
        if (fail_stop && number >= 24U && value == 0U) return MODBUS_ERROR_TIMEOUT;
        speed_parameters[i][number - 22U] = value;
    }
    if (group == P100S_GROUP_P3 && number == 32U) feedback_mode[i]=value;
    if (group == P100S_GROUP_PA && number == 24U) {
        if (fail_stop && value == 0U) return MODBUS_ERROR_TIMEOUT;
        last_speed[i] = (int16_t)value;
    }
    if (group == P100S_GROUP_P3 && number == 16U) {
        forced_speed[i] = value;
        if (value & 0x80U) assert(last_speed[i] == 0); /* Old PA24 cleared first. */
        current_mode[i] = (value & 0x80U) ? 1U : 0U;
    }
    if (group == P100S_GROUP_P3 && number == 18U) {
        /* Releasing HOLD without a fresh rising edge would resume old travel. */
        if ((forced[i] & 4U) && !(value & 4U))
            assert((value & 8U) && !(forced[i] & 8U));
        if (value & 4U) {
            ++holds[i];
            if (fail_stop) return MODBUS_ERROR_TIMEOUT;
        }
        uint8_t edge=(uint8_t)((value & 8U) && !(forced[i] & 8U));
        if (edge) ++triggers[i];
        forced[i] = value;
        if (edge && lose_trigger_reply) {
            lose_trigger_reply=0U;
            return MODBUS_ERROR_TIMEOUT; /* Applied, but reply was lost. */
        }
    }
    if (inject_trip && group == P100S_GROUP_P4 && number == 4U) {
        inject_trip = 0U;
        Command_Manager_SampleLimits(&manager, 1U, 0U);
    }
    return MODBUS_OK;
}
modbus_result_t p100s_read_status(p100s_handle_t *m) {
    ++status_read_count[m->slave_id-1U];
    if (status_follows_setpoint) m->status.speed_raw=last_speed[m->slave_id-1U];
    return fail_reads ? MODBUS_ERROR_TIMEOUT : MODBUS_OK;
}
modbus_result_t p100s_read_alarm(p100s_handle_t *m, uint16_t *alarm)
{ (void)m; *alarm = 0U; return MODBUS_OK; }
HAL_StatusTypeDef PC_Comm_SendResponse(uint8_t id, uint8_t command, pc_status_t status,
    uint16_t parameter, int16_t value)
{ (void)id; (void)command; (void)parameter; (void)value; reply = status; return HAL_OK; }
HAL_StatusTypeDef PC_Comm_SendResponseData(uint8_t id, uint8_t command, pc_status_t status,
    uint16_t parameter, const uint8_t *data, uint8_t size)
{
    (void)id; (void)command; (void)parameter;
    reply = status; response_size = size; memcpy(response, data, size); return HAL_OK;
}
static void sample(uint8_t a, uint8_t b, unsigned count)
{
    while (count--) { ++now; Command_Manager_SampleLimits(&manager, a, b); }
}
static void request(uint8_t id, uint8_t command)
{
    pc_command_packet_t c = {0};
    c.device_id = id; c.command = command; c.valid_payload = 1U;
    /* 0x14: 0.20 output rpm, +5 degrees. */
    c.data[1] = 20U; c.data[4] = 0x13U; c.data[5] = 0x88U;
    Command_Process(&manager, &c);
}
static void debug_request(uint8_t id, uint8_t action, uint16_t key)
{
    pc_command_packet_t c = {0};
    c.device_id = id; c.command = PC_CMD_LIMIT_DEBUG; c.valid_payload = 1U;
    c.parameter = key; c.data[0] = action;
    Command_Process(&manager, &c);
}
static void continuous_request(uint8_t id, int32_t angular_speed)
{
    pc_command_packet_t c = {0};
    c.device_id = id; c.command = PC_CMD_ROTATE_CONTINUOUS; c.valid_payload = 1U;
    uint32_t raw = (uint32_t)angular_speed;
    c.data[0] = (uint8_t)(raw >> 24); c.data[1] = (uint8_t)(raw >> 16);
    c.data[2] = (uint8_t)(raw >> 8); c.data[3] = (uint8_t)raw;
    Command_Process(&manager, &c);
}
static void keepalive(uint8_t id)
{
    pc_command_packet_t c = {0};
    c.device_id = id; c.command = PC_CMD_ROTATE_KEEPALIVE; c.valid_payload = 1U;
    c.parameter = PC_ROTATE_KEEPALIVE_KEY;
    Command_Process(&manager, &c);
}
static void test_continuous(void)
{
    manager.guard[0].reference_angle=0;manager.guard[0].soft_on=0;manager.guard[0].fixed_range=0;
    /* Clear enabled-limit inputs as well, to test both build configurations. */
    sample(0U, 0U, 30U); now += 200U; Command_Manager_Task(&manager);
    continuous_request(1U, 0);
    assert(reply == PC_STATUS_ILLEGAL_PARAM && !manager.continuous_active[0]);
    mapped_speed_input = 1U;
    continuous_request(1U, 5000);
    assert(reply == PC_STATUS_ILLEGAL_PARAM && !manager.continuous_active[0]);
    mapped_speed_input = 0U;
    forced_speed[0] = 0x07U;
    continuous_request(1U, 5000);
    assert(reply == PC_STATUS_SUCCESS && manager.continuous_active[0] && last_speed[0] == 42);
    assert(!(forced_speed[0] & 0x07U));
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_BUSY);
    now += 4000U; keepalive(1U);
    assert(reply == PC_STATUS_SUCCESS);
    now += 4000U; Command_Manager_Task(&manager);
    assert(manager.continuous_active[0]);
    fail_stop = 1U; now += 1001U; Command_Manager_Task(&manager);
    assert(!manager.continuous_active[0] && manager.continuous_stop_pending[0]);
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_BUSY);
    fail_stop = 0U; now += 200U; Command_Manager_Task(&manager);
    assert(last_speed[0] == 0 && !manager.continuous_stop_pending[0]);
    keepalive(1U);
    assert(reply == PC_STATUS_BUSY && !manager.continuous_active[0]);
    continuous_request(2U, -5000);
    assert(reply == PC_STATUS_SUCCESS && last_speed[1] == -42);
    request(2U, PC_CMD_MOTOR_STOP);
    assert(reply == PC_STATUS_SUCCESS && !manager.continuous_active[1] && last_speed[1] == 0);
    now += 200U; Command_Manager_Task(&manager);
    now = UINT32_MAX - 2000U;
    continuous_request(1U, 5000);
    assert(reply == PC_STATUS_SUCCESS);
    now += 5001U; Command_Manager_Task(&manager);
    assert(!manager.continuous_active[0] && last_speed[0] == 0);
}
int main(void)
{
    motors[0].slave_id = 1U; motors[1].slave_id = 2U;
    motors[0].bus = &bus; motors[1].bus = &bus;
    Command_Manager_Init(&manager, &motors[0], &motors[1], NULL);
    /* This suite models generic axes; fixed fixture policy has separate tests. */
    manager.guard[0].reference_angle=0;manager.guard[0].soft_on=0;manager.guard[0].fixed_range=0;
#if !MOTOR_LIMITS_ENABLED
    assert(manager.limits[0].active == 0U && manager.limits[0].trips == 0U);
    sample(1U, 1U, 100U); /* Unwired/high inputs cannot block the no-limit build. */
    Command_Manager_Task(&manager);
    assert(holds[0] == 0U && holds[1] == 0U);
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_SUCCESS && triggers[0] == 1U);
    request(2U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_SUCCESS && triggers[1] == 1U);
    manager.limits[0].active = 1U; manager.limits[0].trips = 10U;
    /* Model completed positive encoder travel; the new execution watchdog
       must not confuse a motionless transport stub with a working drive. */
    motors[0].status.position=motors[1].status.position=7000;
    now += 6000U; Command_Manager_Task(&manager);
    assert(holds[0] == 0U);
    inject_trip = 1U;
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_SUCCESS && triggers[0] == 2U);
    request(1U, PC_CMD_READ_SPEED);
    assert(reply == PC_STATUS_SUCCESS && response_size == 4U);
    debug_request(1U, 1U, PC_LIMIT_DEBUG_KEY);
    assert(reply == PC_STATUS_UNKNOWN_CMD && holds[0] == 0U);
    request(1U, PC_CMD_MOTOR_STOP);
    assert(reply == PC_STATUS_SUCCESS && (forced[0] & 4U));
    puts("no-limit build integration tests passed");
    test_continuous();
    return 0;
#else
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_LIMIT_ACTIVE && triggers[0] == 0U);
    sample(0U, 0U, 29U);
    assert(manager.limits[0].active);
    sample(0U, 0U, 1U);
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_SUCCESS && triggers[0] == 1U);

    unsigned other_holds = holds[1];
    sample(1U, 0U, 1U);
    /* A short pulse is latched even if cleared during blocking communication. */
    sample(0U, 0U, 30U);
    fail_stop = 1U; now += 200U;
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_LIMIT_ACTIVE && triggers[0] == 1U);
    assert(holds[1] == other_holds);
    fail_stop = 0U; motors[0].status.speed_raw = 10; now += 200U;
    Command_Manager_Task(&manager);
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_LIMIT_ACTIVE);
    motors[0].status.speed_raw = 0; now += 200U;
    Command_Manager_Task(&manager);
    assert(triggers[0] == 1U); /* Release alone never restarts. */
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_SUCCESS && triggers[0] == 2U);

    sample(1U, 0U, 1U); now += 200U;
    request(1U, PC_CMD_WRITE_PA);
    assert(reply == PC_STATUS_LIMIT_ACTIVE);
    request(2U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_SUCCESS && triggers[1] == 1U);
    request(1U, PC_CMD_READ_SPEED);
    assert(reply == PC_STATUS_SUCCESS && response_size == 5U && (response[4] & 1U));

    sample(0U, 0U, 30U); now += 200U;
    inject_trip = 1U;
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_LIMIT_ACTIVE && triggers[0] == 2U);
    now += 200U; Command_Manager_Task(&manager);
    assert(forced[0] & 4U);
    debug_request(1U, 1U, 0U);
    assert(reply == PC_STATUS_ILLEGAL_PARAM && !manager.limit_bypass[0]);
    debug_request(1U, 3U, PC_LIMIT_DEBUG_KEY);
    assert(reply == PC_STATUS_ILLEGAL_PARAM && !manager.limit_bypass[0]);
    forced[0] = 0U; fail_stop = 1U;
    debug_request(1U, 1U, PC_LIMIT_DEBUG_KEY);
    assert(reply != PC_STATUS_SUCCESS && !manager.limit_bypass[0]);
    fail_stop = 0U; motors[0].status.speed_raw = 10;
    debug_request(1U, 1U, PC_LIMIT_DEBUG_KEY);
    assert(reply == PC_STATUS_BUSY && !manager.limit_bypass[0]);
    motors[0].status.speed_raw = 0;
    debug_request(1U, 1U, PC_LIMIT_DEBUG_KEY);
    assert(reply == PC_STATUS_SUCCESS && manager.limit_bypass[0]);
    unsigned start_count = triggers[0];
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_SUCCESS && triggers[0] == start_count + 1U);
    request(1U, PC_CMD_READ_SPEED);
    assert((response[4] & 5U) == 5U); /* Raw active remains visible in bypass. */
    sample(1U, 1U, 1U);
    request(2U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_LIMIT_ACTIVE); /* Per-axis isolation. */
    now += 3000U;
    debug_request(1U, 2U, PC_LIMIT_DEBUG_KEY);
    assert(reply == PC_STATUS_SUCCESS);
    now += 3000U; Command_Manager_Task(&manager);
    assert(manager.limit_bypass[0]); /* Keepalive extended the lease. */
    sample(0U, 1U, 30U);
    forced[0] = 0U;
    now += 2001U; Command_Manager_Task(&manager);
    assert(!manager.limit_bypass[0] && (forced[0] & 4U)); /* Expiry stops even clear. */
    debug_request(1U, 2U, PC_LIMIT_DEBUG_KEY);
    assert(reply == PC_STATUS_BUSY && !manager.limit_bypass[0]);
    debug_request(1U, 1U, PC_LIMIT_DEBUG_KEY);
    assert(manager.limit_bypass[0]);
    forced[0] = 0U; motors[0].status.speed_raw = 10;
    debug_request(1U, 0U, PC_LIMIT_DEBUG_KEY);
    assert(reply == PC_STATUS_SUCCESS && !manager.limit_bypass[0]);
    request(1U, PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply == PC_STATUS_LIMIT_ACTIVE && (forced[0] & 4U));
    motors[0].status.speed_raw = 0; now += 200U; Command_Manager_Task(&manager);
    now = UINT32_MAX - 2000U;
    debug_request(1U, 1U, PC_LIMIT_DEBUG_KEY);
    assert(manager.limit_bypass[0]);
    now += 5001U; Command_Manager_Task(&manager);
    assert(!manager.limit_bypass[0]); /* Lease survives tick wrap. */
    Command_Manager_Init(&manager, &motors[0], &motors[1], NULL);
    assert(!manager.limit_bypass[0] && !manager.limit_bypass[1]);
    puts("limit protection integration tests passed");
    test_continuous();
    return 0;
#endif
}
