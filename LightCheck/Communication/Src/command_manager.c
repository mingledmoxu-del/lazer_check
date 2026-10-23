#include "command_manager.h"
#include "axis_motion.h"
#include "axis_guard_controller.h"
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
#define P100S_PA_ACCEL_TIME        40U
#define P100S_PA_DECEL_TIME        41U
#define P100S_PA_S_CURVE_TIME      42U
#define P100S_REG_CONTROL_MODE 0x1009U
#define P100S_REG_RUN_STATE     0x1016U
#define P100S_P3_FORCE_2          16U
#define P100S_P3_FORCE_4          18U
#define P100S_P4_MODE              0U
#define P100S_P4_COMPLETE_DELAY    1U
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

static uint8_t limit_blocked(const command_manager_t *m, uint8_t index)
{
#if MOTOR_LIMITS_ENABLED
    if (m->limit_bypass[index] != 0U)
        return (uint8_t)((int32_t)(HAL_GetTick() - m->limit_bypass_until[index]) >= 0);
    return (uint8_t)(m->limits[index].active != 0U ||
                    m->limits[index].trips != m->limit_handled[index]);
#else
    (void)m; (void)index;
    return 0U;
#endif
}

static uint8_t motion_command(uint8_t command)
{
    return (uint8_t)(command == PC_CMD_MOTOR_START || command == PC_CMD_SERVO_ON ||
                    command == PC_CMD_WRITE_PA || command == PC_CMD_SET_SPEED ||
                    command == PC_CMD_MOVE_RELATIVE || command == PC_CMD_MOVE_ABSOLUTE ||
                    command == PC_CMD_MOVE_OUTPUT_RELATIVE || command == PC_CMD_MOVE_OUTPUT_TRACKED || command == PC_CMD_JOG ||
                    command == PC_CMD_ROTATE_CONTINUOUS);
}

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
    if (command==PC_CMD_MOVE_AXES_POINT || command==PC_CMD_POINT_TIMING) return 1U;
    if (command>=PC_CMD_BANK_BEGIN && command<=PC_CMD_BANK_STATUS) return 1U;
    if (command==PC_CMD_MOVE_OUTPUT_TRACKED) return 1U;
    if (command>=PC_CMD_TRAJECTORY_BEGIN && command<=PC_CMD_TRAJECTORY_CLEAR) return 1U;
    if (command>=PC_CMD_READ_FEEDBACK && command<=PC_CMD_HOME_DUAL_LIMIT) return 1U;
    if (command == PC_CMD_LIMIT_DEBUG) return MOTOR_LIMITS_ENABLED != 0;
    return (uint8_t)((command >= PC_CMD_WRITE_PA && command <= PC_CMD_EMERGENCY_STOP) ||
                     (command >= PC_CMD_SET_SPEED && command <= PC_CMD_ROTATE_KEEPALIVE));
}

static int16_t read_i16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
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

/* One holding-register range transaction; conservative single-read fallback
   for drive revisions rejecting a range. Never retry an ambiguous write. */
static modbus_result_t read_parameters(p100s_handle_t *motor,p100s_parameter_group_t group,
                                      uint16_t first,uint16_t count,uint16_t *values)
{
    modbus_result_t r=modbus_read_holding(motor->bus,motor->slave_id,(uint16_t)group+first,count,values);
    if (r==MODBUS_ERROR_EXCEPTION) {
        for (uint16_t n=0;n<count;++n) {
            r=p100s_read_parameter(motor,group,(uint16_t)(first+n),values+n);
            if (r!=MODBUS_OK) break;
        }
    }
    return r;
}

static modbus_result_t write_pa_temp_verified(p100s_handle_t *motor, uint16_t number, uint16_t value)
{
    modbus_result_t result = write_pa_temp(motor, number, value);
    uint16_t actual = 0U;
    if (result == MODBUS_OK)
        result = p100s_read_parameter(motor, P100S_GROUP_PA, number, &actual);
    if (result == MODBUS_OK && actual != value) result = MODBUS_ERROR_ARGUMENT;
    return result;
}

static modbus_result_t write_position_verified(p100s_handle_t *motor, uint16_t number, uint16_t value)
{
    uint16_t actual=0U;
    modbus_result_t result=p100s_write_parameter(motor,P100S_GROUP_P4,number,value,P100S_STORAGE_SAVED);
    if (result==MODBUS_OK) result=p100s_read_parameter(motor,P100S_GROUP_P4,number,&actual);
    if (result==MODBUS_OK && actual!=value) result=MODBUS_ERROR_ARGUMENT;
    return result;
}

static modbus_result_t set_motion_profile(p100s_handle_t *motor,
                                          uint16_t accel_ms,
                                          uint16_t decel_ms,
                                          uint16_t s_curve_ms)
{
    if (accel_ms < 1U || accel_ms > 10000U || decel_ms < 1U || decel_ms > 10000U ||
        s_curve_ms > 1000U)
        return MODBUS_ERROR_ARGUMENT;

    modbus_result_t result = p100s_read_status(motor);
    if (result != MODBUS_OK) return result;
    if (motor->status.speed_raw < -5 || motor->status.speed_raw > 5) return MODBUS_ERROR_BUSY;

    result = write_pa_temp_verified(motor, P100S_PA_ACCEL_TIME, accel_ms);
    if (result == MODBUS_OK)
        result = write_pa_temp_verified(motor, P100S_PA_DECEL_TIME, decel_ms);
    if (result == MODBUS_OK)
        result = write_pa_temp_verified(motor, P100S_PA_S_CURVE_TIME, s_curve_ms);
    return result;
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

static modbus_result_t trigger_internal_position_1(command_manager_t *manager, uint8_t index, p100s_handle_t *motor)
{
    uint16_t value = 0U;
    modbus_result_t result = p100s_read_parameter(motor, P100S_GROUP_P3, P100S_P3_FORCE_4, &value);
    if (result != MODBUS_OK) return result;

    /* NEVER release HOLD in the CTRG-low transaction: that resumes the old
       unfinished segment before the new rising edge can be accepted. */
    uint16_t idle = (uint16_t)((value & (uint16_t)~P100S_FORCE_POSITION_MASK) |
                               (value & P100S_FORCE_HOLD));
    if (limit_blocked(manager, index)) return MODBUS_ERROR_BUSY;
    result = p100s_write_parameter(motor, P100S_GROUP_P3, P100S_P3_FORCE_4,
                                   idle, P100S_STORAGE_SAVED);
    if (result == MODBUS_OK) {
        uint16_t actual=0U;
        result=p100s_read_parameter(motor,P100S_GROUP_P3,P100S_P3_FORCE_4,&actual);
        if (result==MODBUS_OK && actual!=idle) result=MODBUS_ERROR_ARGUMENT;
    }
    /* Give the low edge a scan interval and require stable zero speed, not
       merely a successful HOLD write. No automatic trigger retry on failure. */
    if (result == MODBUS_OK) result = p100s_read_status(motor);
    if (result == MODBUS_OK && motor->status.speed_raw != 0) result = MODBUS_ERROR_BUSY;
    if (result == MODBUS_OK) {
        /* P4-1=20 ms uses command-stopped CMDOK, not just instantaneous ZSPD.
           Allow that interval plus scan margin, with CTRG held low. This is
           a settling interval, NOT proof that the drive accepted the edge. */
        HAL_Delay(60U);
        result = p100s_read_status(motor);
        if (result == MODBUS_OK && motor->status.speed_raw != 0) result = MODBUS_ERROR_BUSY;
    }
    if (result == MODBUS_OK && limit_blocked(manager, index)) return MODBUS_ERROR_BUSY;
    if (result == MODBUS_OK)
        result = p100s_write_parameter(motor, P100S_GROUP_P3, P100S_P3_FORCE_4,
                                       (uint16_t)((idle & (uint16_t)~P100S_FORCE_HOLD) | P100S_FORCE_CTRG), P100S_STORAGE_SAVED);
    if (result != MODBUS_OK) {
        /* A lost write reply is ambiguous. Reassert pause, never replay CTRG. */
        (void)update_forced_input(motor, P100S_P3_FORCE_4, P100S_FORCE_HOLD, 1U);
    }
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
    if (mode == 1U) {
        /* PA24 alone cannot stop analog/JOG/keypad speed, or SP1/SP2 selecting
           PA25..27. Zero ALL banks before selecting internal speed so a live
           physical selector cannot briefly apply an old nonzero setpoint.
           Keep SON, DI mappings and the saved power-on configuration intact. */
        for (uint16_t number = P100S_PA_INTERNAL_SPEED_1; number <= 27U; ++number) {
            uint16_t actual=0U;
            result=p100s_read_parameter(motor,P100S_GROUP_PA,number,&actual);
            if (result==MODBUS_OK && actual!=0U) result = write_pa_temp_verified(motor, number, 0U);
            if (result != MODBUS_OK) return result;
        }
        uint16_t source=0U;
        result=p100s_read_parameter(motor,P100S_GROUP_PA,P100S_PA_SPEED_SOURCE,&source);
        if (result==MODBUS_OK && source!=1U) result=write_pa_temp_verified(motor,P100S_PA_SPEED_SOURCE,1U);
        return result;
    }
    uint16_t forced = 0U;
    result = p100s_read_parameter(motor, P100S_GROUP_P3, P100S_P3_FORCE_4, &forced);
    if (result != MODBUS_OK) return result;
    /* Drop CTRG while paused, so the next start has a fresh edge. */
    forced = (uint16_t)((forced | P100S_FORCE_HOLD) & (uint16_t)~P100S_FORCE_CTRG);
    return p100s_write_parameter(motor, P100S_GROUP_P3, P100S_P3_FORCE_4, forced, P100S_STORAGE_SAVED);
}

static modbus_result_t prepare_speed(p100s_handle_t *motor)
{
    /* Internal speed 1 requires SP1=SP2=OFF and CINV=OFF. Refuse physical
       mappings that could select another (possibly nonzero) speed or reverse
       direction later; never rewrite the user's SON/DI assignments. */
    uint16_t functions[6];
    modbus_result_t check=read_parameters(motor,P100S_GROUP_P3,0U,6U,functions);
    if (check!=MODBUS_OK) return check;
    for (unsigned di=0;di<6U;++di)
        if (functions[di]>=9U && functions[di]<=11U) return MODBUS_ERROR_ARGUMENT;
    /* External SON is the motion authority. 0x1016 is telemetry and some
       drive revisions do not report state 3 until motion has started. */
    /* Clear the previous internal speed BEFORE changing CMODE, otherwise an
       old nonzero PA24 could briefly run in the previous direction. */
    modbus_result_t result = write_pa_temp(motor, P100S_PA_INTERNAL_SPEED_1, 0U);
    if (result == MODBUS_OK)
        result = update_forced_input(motor, P100S_P3_FORCE_2, 0x07U, 0U);
    if (result == MODBUS_OK) result = select_mixed_control_mode(motor, 1U);
    if (result == MODBUS_OK) result = write_pa_temp(motor, P100S_PA_SPEED_SOURCE, 1U);
    return result;
}

typedef struct {
    uint16_t mode,ppr,profile[2],functions[6],forced,source,setpoint;
} point_setup_t;

/* Read current values in two ranges (33 and 32 words, below the drive's
   documented 100-word maximum). Remember only explicit range rejection,
   never values; CRC/timeouts must abort without a retry. */
static modbus_result_t read_point_setup(command_manager_t *m,uint8_t i,point_setup_t *s)
{
    p100s_handle_t *motor=m->motors[i];uint16_t words[33];
    modbus_result_t r=MODBUS_ERROR_EXCEPTION;
    if (!(m->point_range_unsupported[i]&1U)) {
        r=modbus_read_holding(motor->bus,motor->slave_id,0x0100U,33U,words);
        if (r==MODBUS_ERROR_EXCEPTION) m->point_range_unsupported[i]|=1U;
    }
    if (r==MODBUS_OK) {
        memcpy(s->functions,words,sizeof(s->functions));s->forced=words[16];s->mode=words[32];
    } else if (r==MODBUS_ERROR_EXCEPTION) {
        r=read_parameters(motor,P100S_GROUP_P3,0U,6U,s->functions);
        if (r==MODBUS_OK) r=p100s_read_parameter(motor,P100S_GROUP_P3,16U,&s->forced);
        if (r==MODBUS_OK) r=p100s_read_parameter(motor,P100S_GROUP_P3,32U,&s->mode);
    }
    if (r!=MODBUS_OK) return r;
    r=MODBUS_ERROR_EXCEPTION;
    if (!(m->point_range_unsupported[i]&2U)) {
        r=modbus_read_holding(motor->bus,motor->slave_id,11U,32U,words);
        if (r==MODBUS_ERROR_EXCEPTION) m->point_range_unsupported[i]|=2U;
    }
    if (r==MODBUS_OK) {
        s->ppr=words[0];s->source=words[11];s->setpoint=words[13];
        s->profile[0]=words[30];s->profile[1]=words[31];
    } else if (r==MODBUS_ERROR_EXCEPTION) {
        r=p100s_read_parameter(motor,P100S_GROUP_PA,11U,&s->ppr);
        if (r==MODBUS_OK) r=read_parameters(motor,P100S_GROUP_PA,41U,2U,s->profile);
        if (r==MODBUS_OK) r=p100s_read_parameter(motor,P100S_GROUP_PA,22U,&s->source);
        if (r==MODBUS_OK) r=p100s_read_parameter(motor,P100S_GROUP_PA,24U,&s->setpoint);
    }
    return r;
}

static modbus_result_t prepare_point_speed(p100s_handle_t *motor, const point_setup_t *setup)
{
    uint16_t functions[6],source=0U,setpoint=0U,forced=0U,control=motor->status.control_mode;
    modbus_result_t r=MODBUS_OK;
    if (setup) {
        memcpy(functions,setup->functions,sizeof(functions));
        source=setup->source;setpoint=setup->setpoint;forced=setup->forced;
    } else r=read_parameters(motor,P100S_GROUP_P3,0U,6U,functions);
    if (r!=MODBUS_OK) return r;
    for (unsigned n=0;n<6U;++n)
        if (functions[n]>=9U && functions[n]<=11U) return MODBUS_ERROR_ARGUMENT;
    if (!setup && r==MODBUS_OK) r=p100s_read_parameter(motor,P100S_GROUP_P3,P100S_P3_FORCE_2,&forced);
    if (!setup && r==MODBUS_OK) {
        uint16_t speeds[3];
        r=modbus_read_holding(motor->bus,motor->slave_id,P100S_PA_SPEED_SOURCE,3U,speeds);
        if (r==MODBUS_OK) {source=speeds[0];setpoint=speeds[2];}
        else if (r==MODBUS_ERROR_EXCEPTION) {
            /* Some revisions reject PA23 in a range. Read only the two
               required endpoints; never retry CRC/timeouts or any write. */
            r=p100s_read_parameter(motor,P100S_GROUP_PA,P100S_PA_SPEED_SOURCE,&source);
            if (r==MODBUS_OK) r=p100s_read_parameter(motor,P100S_GROUP_PA,P100S_PA_INTERNAL_SPEED_1,&setpoint);
        }
    }
    if (r!=MODBUS_OK) return r;
    /* Reuse only a freshly verified internal-speed mode with no selector or
       inversion override. Otherwise take the original full preparation path. */
    if (control==1U && source==1U && (forced&0x87U)==0x80U) {
        if (setpoint) return write_pa_temp_verified(motor,P100S_PA_INTERNAL_SPEED_1,0U);
        return MODBUS_OK;
    }
    r=prepare_speed(motor);
    if (r==MODBUS_OK) r=read_register(motor,P100S_REG_CONTROL_MODE,&control);
    if (r==MODBUS_OK) r=p100s_read_parameter(motor,P100S_GROUP_PA,P100S_PA_SPEED_SOURCE,&source);
    if (r==MODBUS_OK && (control!=1U || source!=1U)) r=MODBUS_ERROR_ARGUMENT;
    return r;
}

modbus_result_t Command_Manager_PrepareVelocity(command_manager_t *m, uint8_t index)
{
    if (index>1U || limit_blocked(m,index)) return MODBUS_ERROR_BUSY;
    m->position_watch[index]=0U;
    m->point_completed[index]=0U;
    return prepare_speed(m->motors[index]);
}
modbus_result_t Command_Manager_UpdateVelocity(command_manager_t *m, uint8_t index, int16_t rpm)
{
    if (index>1U || rpm < -5000 || rpm > 5000) return MODBUS_ERROR_ARGUMENT;
    if (limit_blocked(m,index)) return MODBUS_ERROR_BUSY;
    return write_pa_temp(m->motors[index],P100S_PA_INTERNAL_SPEED_1,(uint16_t)rpm);
}
static modbus_result_t start_speed(command_manager_t *manager, uint8_t index, p100s_handle_t *motor)
{
    (void)motor;
    int16_t speed=manager->target_speed[index];
    if (speed==0 || speed < -5000 || speed > 5000) return MODBUS_ERROR_ARGUMENT;
    modbus_result_t result=Command_Manager_PrepareVelocity(manager,index);
    if (result==MODBUS_OK) result=Command_Manager_UpdateVelocity(manager,index,speed);
    return result;
}

static modbus_result_t move_position(command_manager_t *manager, uint8_t index, p100s_handle_t *motor,
                                    uint8_t absolute, int16_t speed, int32_t pulses, int32_t *reported_position)
{
    if (limit_blocked(manager, index)) return MODBUS_ERROR_BUSY;
    if (speed == 0 || speed < -5000 || speed > 5000)
        return MODBUS_ERROR_ARGUMENT;
    uint16_t ppr = 0U;
    modbus_result_t stationary = p100s_read_status(motor);
    if (stationary != MODBUS_OK) return stationary;
    if (motor->status.speed_raw != 0) return MODBUS_ERROR_BUSY;
    /* Do not gate motion on 0x1016. The physical external SON input decides
       whether torque is enabled; run state remains available to the PC UI. */
    modbus_result_t result = p100s_read_parameter(motor, P100S_GROUP_PA, P100S_PA_PULSES_PER_REV, &ppr);
    if (result != MODBUS_OK || ppr == 0U) return result == MODBUS_OK ? MODBUS_ERROR_ARGUMENT : result;

    int32_t turns = pulses / (int32_t)ppr;
    int32_t remainder = pulses % (int32_t)ppr;
    if (turns < -30000 || turns > 30000) return MODBUS_ERROR_ARGUMENT;

    int32_t current_position = motor->status.position;
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
    /* A successful Modbus write alone does not prove the new task was loaded. */
    if (result == MODBUS_OK) result = write_position_verified(motor,P100S_P4_COMPLETE_DELAY,20U);
    if (result == MODBUS_OK) result = write_position_verified(motor,P100S_P4_MODE,absolute != 0U ? 0U : 1U);
    if (result == MODBUS_OK) result = write_position_verified(motor,P100S_P4_P1_TURNS,(uint16_t)turns);
    if (result == MODBUS_OK) result = write_position_verified(motor,P100S_P4_P1_PULSES,(uint16_t)remainder);
    if (result == MODBUS_OK) result = write_position_verified(motor,P100S_P4_P1_SPEED,(uint16_t)(speed < 0 ? -speed : speed));
    if (result == MODBUS_OK && limit_blocked(manager, index)) return MODBUS_ERROR_BUSY;
    if (result == MODBUS_OK) result = trigger_internal_position_1(manager, index, motor);
    if (result == MODBUS_OK) {
        int64_t delta=target-current_position;
        manager->position_noise[index]=(uint16_t)(ppr/1000U+2U);
        manager->position_watch[index]=(uint8_t)(delta>manager->position_noise[index] || delta<-(int64_t)manager->position_noise[index]);
        manager->position_direction[index]=delta>0 ? 1 : -1;
        manager->position_start[index]=current_position;
        manager->position_started_at[index]=HAL_GetTick();
        manager->position_poll_at[index]=HAL_GetTick()+100U;
        *reported_position = (int32_t)target;
    }
    return result;
}

void Command_Manager_Init(command_manager_t *m, p100s_handle_t *m1, p100s_handle_t *m2, UART_HandleTypeDef *uart)
{
    if (m == NULL) return;
    memset(m, 0, sizeof(*m));
    m->motors[0] = m1; m->motors[1] = m2; m->pc_uart = uart;
    m->gear_ratio_num[0] = 50U; m->gear_ratio_num[1] = 50U;
    m->gear_ratio_den[0] = 1U;  m->gear_ratio_den[1] = 1U;
    GuardInputs_Init(&m->guard_inputs);
    AxisGuard_Init(&m->guard[0]);AxisGuard_Init(&m->guard[1]);
    /* Axis 1 fixture calibration: horizontal=0, sensor reference=+25 deg.
       A reference must still be established after every reset. */
    m->guard[0].reference_angle=25000;
    m->guard[0].fixed_range=1U;
    m->guard[0].minimum=-35000; m->guard[0].maximum=35000;
    m->guard[0].soft_on=1U; /* Fixed fixture range; wait for a trustworthy reference. */
#if MOTOR_LIMITS_ENABLED
    LimitSwitch_Init(&m->limits[0]);
    LimitSwitch_Init(&m->limits[1]);
#endif
}

void Command_Manager_SampleLimits(command_manager_t *m, uint8_t limit1, uint8_t limit2)
{
#if MOTOR_LIMITS_ENABLED
    if (m == NULL) return;
    LimitSwitch_Sample(&m->limits[0], limit1);
    LimitSwitch_Sample(&m->limits[1], limit2);
#else
    (void)m; (void)limit1; (void)limit2;
#endif
}

void Command_Manager_SampleInputs(command_manager_t *m, uint8_t raw_mask)
{
    if (m) GuardInputs_Sample(&m->guard_inputs,raw_mask);
}
modbus_result_t Command_Manager_RunVelocity(command_manager_t *m, uint8_t i, int16_t rpm)
{
    m->target_speed[i]=rpm;
    return start_speed(m,i,m->motors[i]);
}
modbus_result_t Command_Manager_StopAxis(command_manager_t *m, uint8_t i)
{
    m->position_watch[i]=0U;
    m->point_active[i]=0U;
    m->point_completed[i]=0U;m->point_settling[i]=0U;
    return stop_motor(m->motors[i]);
}

uint16_t Command_Manager_PointFlags(command_manager_t *m,uint8_t i)
{
    uint16_t flags=m->point_active[i] ? 0x20U : 0U;
    int64_t error=(int64_t)m->motors[i]->status.position-m->point_target[i];
    int64_t tolerance=m->point_tolerance[i] ? m->point_tolerance[i] : m->point_ppr[i]/200U+1U;
    if (m->point_completed[i] && !m->point_active[i] && m->guard[i].zero_valid &&
        !m->guard[i].fault && !m->guard[i].stop_pending && !m->motors[i]->status.alarm_code &&
        m->motors[i]->status.speed_raw==0 && error>=-tolerance && error<=tolerance) flags|=0x40U;
    return flags;
}

/* Latest typed target owns the axis. Do not release/resume an old P4 segment:
   use signed velocity plus actual encoder feedback for these point tasks. */
static void point_fault(command_manager_t *m,uint8_t i,uint8_t reason)
{
    axis_guard_t *g=&m->guard[i];
    g->fault=reason;g->phase=HOME_FAULT;g->stop_pending=1U;g->retry_at=HAL_GetTick();
    if (reason==GUARD_COMM || reason==GUARD_FEEDBACK || reason==GUARD_ALARM) g->zero_valid=0U;
    (void)Command_Manager_StopAxis(m,i);
}

static modbus_result_t start_point(command_manager_t *m,uint8_t i,int16_t rpm,int32_t delta,uint16_t verified_ppr,const point_setup_t *setup)
{
    uint32_t init_started=HAL_GetTick();
    if (limit_blocked(m,i)) return MODBUS_ERROR_BUSY;
    p100s_handle_t *motor=m->motors[i];
    uint8_t completed=(uint8_t)((Command_Manager_PointFlags(m,i)&0x40U)!=0U);
    modbus_result_t r=verified_ppr ? MODBUS_OK : p100s_read_status(motor);
    if (r==MODBUS_OK && motor->status.speed_raw!=0) r=Command_Manager_StopAxis(m,i);
    /* The initial independent read counts. A preceding completed point already
       supplies two zeros; one new zero revalidates that proof before reuse. */
    uint8_t zeros=(uint8_t)(r==MODBUS_OK && motor->status.speed_raw==0 ? 1U : 0U);
    if (completed && zeros && (Command_Manager_PointFlags(m,i)&0x40U)) zeros=2U;
    uint32_t until=HAL_GetTick()+2000U;
    while (r==MODBUS_OK && zeros<2U) {
        HAL_Delay(20U);
        r=p100s_read_status(motor);
        if (r!=MODBUS_OK) break;
        zeros=motor->status.speed_raw==0 ? (uint8_t)(zeros+1U) : 0U;
        if ((int32_t)(HAL_GetTick()-until)>=0) { r=MODBUS_ERROR_BUSY;break; }
    }
    if (r!=MODBUS_OK) {
        point_fault(m,i,r==MODBUS_ERROR_BUSY ? GUARD_EXECUTION : GUARD_COMM);
        return r;
    }
    uint16_t mode=3U,ppr=verified_ppr;
    uint16_t profile[2]={0U,0U};
    if (r==MODBUS_OK && !verified_ppr) r=p100s_read_parameter(motor,P100S_GROUP_P3,32U,&mode);
    if (r==MODBUS_OK && !verified_ppr) r=p100s_read_parameter(motor,P100S_GROUP_PA,11U,&ppr);
    if (setup) memcpy(profile,setup->profile,sizeof(profile));
    else if (r==MODBUS_OK) r=read_parameters(motor,P100S_GROUP_PA,P100S_PA_DECEL_TIME,2U,profile);
    if (r!=MODBUS_OK) return r;
    if (mode!=3U || !ppr || !profile[0] || profile[0]>10000U || profile[1]>1000U || rpm<1 || rpm>5000) return MODBUS_ERROR_ARGUMENT;
    int64_t target=(int64_t)motor->status.position+delta;
    if (target<INT32_MIN || target>INT32_MAX) return MODBUS_ERROR_ARGUMENT;
    axis_guard_t *g=&m->guard[i];
    if (g->soft_on) {
        int32_t angle=0;
        if (!AxisGuard_Angle(g,(int32_t)target,mode,ppr,m->gear_ratio_num[i],m->gear_ratio_den[i],&angle))
            return MODBUS_ERROR_ARGUMENT;
        if (angle<g->minimum || angle>g->maximum) return MODBUS_ERROR_ARGUMENT;
    }
    r=prepare_point_speed(motor,setup);
    if (r!=MODBUS_OK) { point_fault(m,i,GUARD_COMM);return r; }
    m->position_watch[i]=0U;m->point_active[i]=1U;m->point_zero_samples[i]=0U;
    m->point_settling[i]=0U;m->point_completed[i]=0U;
    m->point_target[i]=(int32_t)target;m->point_last[i]=motor->status.position;
    m->point_progress_position[i]=motor->status.position;
    m->point_max_rpm[i]=rpm;m->point_sent_rpm[i]=0;m->point_ppr[i]=ppr;
    m->point_tolerance[i]=ppr/200U+1U;
    m->point_decel_ms[i]=profile[0];m->point_s_curve_ms[i]=profile[1];
    m->point_units_at[i]=HAL_GetTick();m->point_sample_at[i]=HAL_GetTick();m->point_sample_valid[i]=1U;
    m->point_poll_at[i]=HAL_GetTick();m->point_progress_at[i]=HAL_GetTick();
    uint64_t duration=(uint64_t)(delta<0 ? -(int64_t)delta : delta)*60000U/((uint64_t)ppr*rpm);
    duration=duration*4U+15000U;
    if (duration>3600000U) duration=3600000U;
    m->point_deadline[i]=HAL_GetTick()+(uint32_t)duration;
    m->point_init_ms[i]=HAL_GetTick()-init_started;
    m->point_cycle_started[i]=HAL_GetTick();
    m->point_cycle_ms[i]=m->point_gap_ms[i]=m->point_max_gap_ms[i]=0U;
    return MODBUS_OK;
}

static void axes_point(command_manager_t *m,const pc_command_packet_t *c,const pc_command_packet_t *reply_to)
{
    uint8_t data[8]={0};pc_status_t status=PC_STATUS_SUCCESS;
    if (c->device_id!=1U || c->parameter!=PC_AXES_POINT_KEY || (c->data_length!=10U && c->data_length!=12U)) {
        status=PC_STATUS_ILLEGAL_PARAM;goto reply;
    }
    uint16_t tolerance=c->data_length==12U ? read_u16(c->data+10) : 0U;
    if (c->data_length==12U && (tolerance<10U || tolerance>500U)) {status=PC_STATUS_ILLEGAL_PARAM;goto reply;}
    if (Trajectory_Active(&m->trajectory) || m->point_active[0] || m->point_active[1]) {
        status=PC_STATUS_BUSY;goto reply;
    }
    for (unsigned i=0;i<2U;++i) {
        axis_guard_t *g=&m->guard[i];
        if (g->stop_pending || g->dual_home || g->dual_pending || g->fault ||
            (g->phase>=HOME_SEEK && g->phase<=HOME_FINAL_STOP) ||
            m->continuous_active[i] || m->continuous_stop_pending[i]) { status=PC_STATUS_BUSY;goto reply; }
        if (!g->zero_valid) { status=PC_STATUS_NOT_REFERENCED;goto reply; }
    }
    uint32_t trips[2][6];uint8_t monitored[2]={0,0};
    for (unsigned i=0;i<2U;++i) {
        axis_guard_t *g=&m->guard[i];
        uint8_t channels[5]={g->negative,g->positive,g->home,g->dual_channel[0],g->dual_channel[1]};
        for (unsigned n=0;n<5U;++n)
            if (channels[n]<6U && ((n<3U && g->hardware_on) || (n>=3U && g->dual_enabled)))
                monitored[i]|=(uint8_t)(1U<<channels[n]);
        for (unsigned ch=0;ch<6U;++ch) trips[i][ch]=AxisGuard_Trips(&m->guard_inputs,g,(uint8_t)ch);
    }
    m->busy=1U;
    for (uint8_t i=0;i<2U;++i) {
        uint32_t began=HAL_GetTick();uint16_t mode=0U,ppr=0U;int32_t angle=0;
        p100s_handle_t *motor=m->motors[i];axis_motion_command_t motion;
        point_setup_t setup={0};
        modbus_result_t r=p100s_read_status(motor);
        if (r==MODBUS_OK && (motor->status.speed_raw || motor->status.alarm_code)) r=MODBUS_ERROR_BUSY;
        if (r==MODBUS_OK) r=read_point_setup(m,i,&setup);
        mode=setup.mode;ppr=setup.ppr;
        if (r!=MODBUS_OK) { status=map_status(r);goto abort; }
        if (!AxisGuard_Angle(&m->guard[i],motor->status.position,mode,ppr,
                            m->gear_ratio_num[i],m->gear_ratio_den[i],&angle)) {
            m->guard[i].zero_valid=0U;status=PC_STATUS_NOT_REFERENCED;goto abort;
        }
        int32_t basis=motor->status.position;
        int64_t delta=(int64_t)read_i32(c->data+2U+4U*i)-angle;
        if (delta<INT32_MIN || delta>INT32_MAX) {status=PC_STATUS_RANGE;goto abort;}
        status=GuardController_PointAllowed(m,i,angle,(int32_t)delta);
        if (status!=PC_STATUS_SUCCESS) goto abort;
        if (!AxisMotion_ConvertRelative(read_u16(c->data),(int32_t)delta,
                m->gear_ratio_num[i],m->gear_ratio_den[i],ppr,&motion)) {
            /* A zero-distance axis is still armed at its current encoder pose. */
            if (delta || !AxisMotion_ConvertRelative(read_u16(c->data),1,
                    m->gear_ratio_num[i],m->gear_ratio_den[i],ppr,&motion)) {
                status=PC_STATUS_ILLEGAL_PARAM;goto abort;
            }
            motion.motor_pulses=0;
        }
        if (m->guard[i].angle_sign<0 && motion.motor_pulses==INT32_MIN) {status=PC_STATUS_RANGE;goto abort;}
        int32_t raw_delta=m->guard[i].angle_sign<0 ? -motion.motor_pulses : motion.motor_pulses;
        int64_t absolute_target=(int64_t)basis+raw_delta;
        if (absolute_target<INT32_MIN || absolute_target>INT32_MAX) {status=PC_STATUS_RANGE;goto abort;}
        r=start_point(m,i,motion.motor_speed_rpm,
            raw_delta,ppr,&setup);
        if (r!=MODBUS_OK) {status=map_status(r);goto abort;}
        m->point_target[i]=(int32_t)absolute_target;
        if (tolerance) {
            uint64_t counts=(uint64_t)tolerance*ppr*m->gear_ratio_num[i]/(360000ULL*m->gear_ratio_den[i]);
            if (!counts) {status=PC_STATUS_ILLEGAL_PARAM;goto abort;}
            m->point_tolerance[i]=(uint32_t)counts;
        }
        put_i32(data+4U*i,(int32_t)(HAL_GetTick()-began));
    }
    /* No velocity task runs between preparation of the two axes. Reject a
       changed sensor input before either task is permitted to start. */
    for (unsigned i=0;i<2U;++i) for (unsigned ch=0;ch<6U;++ch)
        if ((monitored[i]&(1U<<ch)) && trips[i][ch]!=AxisGuard_Trips(&m->guard_inputs,&m->guard[i],(uint8_t)ch)) {
            status=PC_STATUS_LIMIT_ACTIVE;goto abort;
        }
    for (unsigned i=0;i<2U;++i) {
        m->point_poll_at[i]=HAL_GetTick();m->point_cycle_started[i]=HAL_GetTick();
    }
    goto armed;
abort:
    for (uint8_t i=0;i<2U;++i) {
        modbus_result_t stopped=Command_Manager_StopAxis(m,i);
        if (stopped!=MODBUS_OK || status==PC_STATUS_SERVO_NO_RESPONSE || status==PC_STATUS_RS485_ERROR || status==PC_STATUS_CRC_ERROR)
            point_fault(m,i,GUARD_COMM);
    }
armed:
    m->busy=0U;
reply:
    (void)PC_Comm_SendResponseData(reply_to->device_id,reply_to->command,status,reply_to->parameter,data,sizeof(data));
}

static uint8_t bank_reference(command_manager_t *m,point_bank_t *b)
{
    for (unsigned i=0;i<2U;++i) {
        axis_guard_t *g=&m->guard[i];
        if (!g->zero_valid || g->origin!=b->origin[i] || g->reference_angle!=b->reference[i] ||
            g->ppr!=b->ppr[i] || g->angle_sign!=b->sign[i] ||
            m->gear_ratio_num[i]!=b->ratio_num[i] || m->gear_ratio_den[i]!=b->ratio_den[i]) return 0U;
    }
    return 1U;
}
static pc_status_t bank_seal(command_manager_t *m,point_bank_t *b)
{
    if (!b->count || b->loaded!=b->count) return PC_STATUS_ILLEGAL_PARAM;
    b->ready=0U;
    for (uint8_t i=0;i<2U;++i) {
        axis_guard_t *g=&m->guard[i];p100s_handle_t *motor=m->motors[i];
        if (!g->zero_valid) return PC_STATUS_NOT_REFERENCED;
        if (g->fault || g->stop_pending || g->dual_pending || g->dual_home ||
            (g->phase>=HOME_SEEK && g->phase<=HOME_FINAL_STOP) ||
            m->continuous_active[i] || m->continuous_stop_pending[i]) return PC_STATUS_BUSY;
        uint16_t mode=0,ppr=0;int32_t angle=0;
        point_setup_t setup={0};
        modbus_result_t r=p100s_read_status(motor);
        if (r==MODBUS_OK) r=read_point_setup(m,i,&setup);
        mode=setup.mode;ppr=setup.ppr;
        if (r!=MODBUS_OK) return map_status(r);
        if (motor->status.speed_raw || motor->status.alarm_code) return PC_STATUS_BUSY;
        if (!AxisGuard_Angle(g,motor->status.position,mode,ppr,m->gear_ratio_num[i],m->gear_ratio_den[i],&angle)) {
            g->zero_valid=0U;return PC_STATUS_NOT_REFERENCED;
        }
        for (unsigned n=0;n<b->count;++n) {
            int64_t delta=(int64_t)b->points[n][i]-angle;axis_motion_command_t converted;
            if ((g->soft_on && (b->points[n][i]<g->minimum || b->points[n][i]>g->maximum)) ||
                delta<INT32_MIN || delta>INT32_MAX) return PC_STATUS_RANGE;
            if (!AxisMotion_ConvertRelative(b->speed,delta ? (int32_t)delta : 1,
                    m->gear_ratio_num[i],m->gear_ratio_den[i],ppr,&converted)) return PC_STATUS_ILLEGAL_PARAM;
        }
        /* Prepare mode once while stationary; no nonzero velocity or point
           task is issued by a preload. Final independent zero validates it. */
        r=prepare_point_speed(motor,&setup);
        if (r==MODBUS_OK) {HAL_Delay(20U);r=p100s_read_status(motor);}
        if (r!=MODBUS_OK) return map_status(r);
        if (motor->status.speed_raw || motor->status.alarm_code) return PC_STATUS_BUSY;
        b->origin[i]=g->origin;b->reference[i]=g->reference_angle;b->ppr[i]=ppr;b->sign[i]=g->angle_sign;
        b->ratio_num[i]=m->gear_ratio_num[i];b->ratio_den[i]=m->gear_ratio_den[i];
    }
    b->ready=1U;return PC_STATUS_SUCCESS;
}
static void bank_command(command_manager_t *m,const pc_command_packet_t *c)
{
    pc_status_t status=PC_STATUS_SUCCESS;uint8_t data[11]={0};
    if (c->device_id!=1U || !c->parameter || c->parameter>POINT_BANK_SLOTS) {
        (void)PC_Comm_SendResponse(c->device_id,c->command,PC_STATUS_ILLEGAL_PARAM,c->parameter,0);return;
    }
    point_bank_t *b=&m->point_banks[c->parameter-1U];
    if (b->ready && !bank_reference(m,b)) b->ready=0U;
    if (c->command!=PC_CMD_BANK_STATUS &&
        (m->point_active[0] || m->point_active[1] || Trajectory_Active(&m->trajectory))) {status=PC_STATUS_BUSY;goto reply;}
    if (c->command!=PC_CMD_BANK_STATUS) for (unsigned i=0;i<2U;++i) {
        axis_guard_t *g=&m->guard[i];
        if (m->continuous_active[i] || m->continuous_stop_pending[i] || m->position_watch[i] ||
            g->stop_pending || g->dual_pending || g->dual_home || (g->phase>=HOME_SEEK && g->phase<=HOME_FINAL_STOP)) {
            status=PC_STATUS_BUSY;goto reply;
        }
    }
    if (c->command==PC_CMD_BANK_BEGIN) {
        uint32_t token=(uint32_t)read_i32(c->data);uint16_t speed=read_u16(c->data+4),tolerance=read_u16(c->data+6);uint8_t count=c->data[8];
        if (!token || token==b->token || !speed || speed>10000U || tolerance<10U || tolerance>500U || !count || count>POINT_BANK_CAPACITY) {status=PC_STATUS_ILLEGAL_PARAM;goto reply;}
        memset(b,0,sizeof(*b));b->token=token;b->speed=speed;b->tolerance=tolerance;b->count=count;
    } else if (c->command!=PC_CMD_BANK_STATUS && (uint32_t)read_i32(c->data)!=b->token) {
        status=PC_STATUS_ILLEGAL_PARAM;
    } else if (c->command==PC_CMD_BANK_POINT) {
        uint8_t n=c->data[4];
        if (!b->token || b->ready || n!=b->loaded || n>=b->count) status=PC_STATUS_ILLEGAL_PARAM;
        else {b->points[n][0]=read_i32(c->data+5);b->points[n][1]=read_i32(c->data+9);++b->loaded;}
    } else if (c->command==PC_CMD_BANK_SEAL) {
        m->busy=1U;status=bank_seal(m,b);m->busy=0U;
        if (status!=PC_STATUS_SUCCESS) {
            for (uint8_t i=0;i<2U;++i) if (Command_Manager_StopAxis(m,i)!=MODBUS_OK) point_fault(m,i,GUARD_COMM);
        }
    } else if (c->command==PC_CMD_BANK_RUN) {
        uint8_t n=c->data[4];
        if (!b->ready || !bank_reference(m,b)) {b->ready=0;status=PC_STATUS_NOT_REFERENCED;}
        else if (n>=b->count) status=PC_STATUS_ILLEGAL_PARAM;
        else {
            pc_command_packet_t run={0};run.device_id=1;run.command=PC_CMD_MOVE_AXES_POINT;
            run.parameter=PC_AXES_POINT_KEY;run.data_length=12;run.valid_payload=1;
            run.data[0]=(uint8_t)(b->speed>>8);run.data[1]=(uint8_t)b->speed;
            put_i32(run.data+2,b->points[n][0]);put_i32(run.data+6,b->points[n][1]);
            run.data[10]=(uint8_t)(b->tolerance>>8);run.data[11]=(uint8_t)b->tolerance;
            axes_point(m,&run,c);return;
        }
    }
reply:
    put_i32(data,(int32_t)b->token);data[4]=(uint8_t)(b->speed>>8);data[5]=(uint8_t)b->speed;
    data[6]=(uint8_t)(b->tolerance>>8);data[7]=(uint8_t)b->tolerance;
    data[8]=b->count;data[9]=b->loaded;data[10]=b->ready;
    (void)PC_Comm_SendResponseData(c->device_id,c->command,status,c->parameter,data,sizeof(data));
}

static void point_task(command_manager_t *m)
{
    if (!m || m->busy) return;
    m->busy=1U;
    for (uint8_t i=0;i<2U;++i) {
        if (!m->point_active[i] || (int32_t)(HAL_GetTick()-m->point_poll_at[i])<0) continue;
        p100s_handle_t *motor=m->motors[i];axis_guard_t *g=&m->guard[i];
        uint32_t cycle_started=HAL_GetTick();
        m->point_gap_ms[i]=cycle_started-m->point_cycle_started[i];
        if (m->point_gap_ms[i]>m->point_max_gap_ms[i]) m->point_max_gap_ms[i]=m->point_gap_ms[i];
        m->point_cycle_started[i]=cycle_started;
        if (g->stop_pending || g->fault || limit_blocked(m,i)) {
            point_fault(m,i,g->fault ? g->fault : GUARD_SENSOR);continue;
        }
        if ((int32_t)(HAL_GetTick()-m->point_poll_at[i])>1000 ||
            (int32_t)(HAL_GetTick()-m->point_deadline[i])>=0) { point_fault(m,i,GUARD_TIMEOUT);continue; }
        uint32_t previous_sample=m->point_sample_at[i];
        m->point_sample_valid[i]=0U;
        modbus_result_t r=p100s_read_status(motor);
        uint32_t sampled_at=HAL_GetTick();
        uint16_t mode=3U,ppr=m->point_ppr[i];
        if (r==MODBUS_OK && HAL_GetTick()-m->point_units_at[i]>=100U) {
            r=p100s_read_parameter(motor,P100S_GROUP_P3,32U,&mode);
            if (r==MODBUS_OK) r=p100s_read_parameter(motor,P100S_GROUP_PA,11U,&ppr);
            m->point_units_at[i]=HAL_GetTick();
        }
        if (r!=MODBUS_OK) { point_fault(m,i,GUARD_COMM);continue; }
        if (mode!=3U || ppr!=m->point_ppr[i]) { point_fault(m,i,GUARD_FEEDBACK);continue; }
        if (motor->status.alarm_code) { point_fault(m,i,GUARD_ALARM);continue; }
        if (g->soft_on) {
            int32_t angle=0;int32_t margin=g->dual_enabled ? GUARD_DUAL_FEEDBACK_MARGIN : 0;
            if (!AxisGuard_Angle(g,motor->status.position,mode,ppr,
                                m->gear_ratio_num[i],m->gear_ratio_den[i],&angle)) {
                point_fault(m,i,GUARD_FEEDBACK);continue;
            }
            if ((int64_t)angle<(int64_t)g->minimum-margin || (int64_t)angle>(int64_t)g->maximum+margin) {
                point_fault(m,i,GUARD_RANGE);continue;
            }
        }
        m->point_sample_at[i]=sampled_at;m->point_sample_valid[i]=1U;
        int64_t step=(int64_t)motor->status.position-m->point_last[i];
        int64_t noise=ppr/1000U+2U;
        if ((m->point_sent_rpm[i]>0 && step < -noise) ||
            (m->point_sent_rpm[i]<0 && step > noise)) { point_fault(m,i,GUARD_EXECUTION);continue; }
        int64_t progress=(int64_t)motor->status.position-m->point_progress_position[i];
        if (progress>noise || progress < -noise) {
            m->point_progress_at[i]=HAL_GetTick();m->point_progress_position[i]=motor->status.position;
        }
        if (!m->point_sent_rpm[i] || step*m->point_sent_rpm[i]>0) m->point_last[i]=motor->status.position;
        int64_t error=(int64_t)m->point_target[i]-motor->status.position;
        int64_t distance=error<0 ? -error : error;
        /* About 0.036 output degrees with PA11=10000 and a 50:1 gearbox. */
        uint32_t brake_tolerance=ppr/200U+1U;
        uint32_t tolerance=m->point_tolerance[i] ? m->point_tolerance[i] : brake_tolerance;
        if (brake_tolerance>tolerance) brake_tolerance=tolerance;
        int16_t desired=0;
        if (distance<=tolerance) m->point_settling[i]=1U;
        if (m->point_settling[i]) {
            /* Hold zero while braking; do not chase encoder jitter or start
               a correction from a single rounded zero-speed observation. */
            if (!m->point_sent_rpm[i] && motor->status.speed_raw==0)
                ++m->point_zero_samples[i];
            else m->point_zero_samples[i]=0U;
            if (m->point_zero_samples[i]>=2U) {
                if (distance<=tolerance) {
                    m->point_active[i]=0U;m->point_completed[i]=1U;
                } else {
                    m->point_settling[i]=0U;m->point_zero_samples[i]=0U;
                }
            }
        } else if (distance>tolerance) {
            uint32_t reaction=(HAL_GetTick()-previous_sample)*2U+20U;
            if (reaction<100U) reaction=100U;
            if (reaction>1000U) reaction=1000U;
            /* Finish tolerance is independently configurable. Do not subtract
               the larger business window from braking distance, which would
               retain the same slow crawl outside that enlarged window. */
            uint16_t speed=AxisMotion_PointSpeed((uint64_t)distance,brake_tolerance,ppr,
                (uint16_t)m->point_max_rpm[i],m->point_decel_ms[i],m->point_s_curve_ms[i],reaction);
            desired=(int16_t)(error<0 ? -speed : speed);
            if (HAL_GetTick()-m->point_progress_at[i]>3000U) { point_fault(m,i,GUARD_EXECUTION);continue; }
            m->point_zero_samples[i]=0U;
        }
        /* If a small overshoot needs correction, first command zero and wait
           for real zero speed. Do not classify normal braking as wrong-way. */
        if (motor->status.speed_raw && (m->point_sent_rpm[i]==0 ||
            (int32_t)desired*m->point_sent_rpm[i]<0)) desired=0;
        /* Ramp up over approximately 200 ms rather than stepping immediately
           to the requested speed. Braking remains unconstrained so existing
           distance/limit protection can always reduce speed immediately. */
        int32_t sent=m->point_sent_rpm[i];
        if ((int32_t)desired*sent<0) desired=0;
        else if ((desired<0 ? -desired : desired)>(sent<0 ? -sent : sent)) {
            uint32_t dt=HAL_GetTick()-previous_sample;
            if (dt>100U) dt=100U;
            uint32_t rise=((uint32_t)m->point_max_rpm[i]*dt+199U)/200U;
            if (!rise) rise=1U;
            int32_t speed=(sent<0 ? -sent : sent)+(int32_t)rise;
            int32_t target=desired<0 ? -desired : desired;
            if (speed>target) speed=target;
            desired=(int16_t)(desired<0 ? -speed : speed);
        }
        if (desired!=m->point_sent_rpm[i]) {
            /* Keep setpoint readback: a drive echo alone need not prove its
               effective setpoint changed, especially on braking commands. */
            r=write_pa_temp_verified(motor,P100S_PA_INTERNAL_SPEED_1,(uint16_t)desired);
            if (r!=MODBUS_OK) { point_fault(m,i,GUARD_COMM);continue; }
            m->point_sent_rpm[i]=desired;
            m->point_last[i]=motor->status.position;
        }
        /* Deadline from cycle start: bus work no longer adds a fixed 50 ms.
           No catch-up loop if the bus overruns; execute once on the next turn. */
        m->point_poll_at[i]=cycle_started+(m->point_settling[i] ? 20U : 50U);
        m->point_cycle_ms[i]=HAL_GetTick()-cycle_started;
    }
    m->busy=0U;
}
static void request_continuous_stop(command_manager_t *m, uint8_t i)
{
    if (m->continuous_active[i] || m->continuous_stop_pending[i]) {
        m->continuous_active[i] = 0U;
        m->continuous_stop_pending[i] = 1U;
        m->continuous_retry_at[i] = HAL_GetTick();
    }
}

/* Independent of the currently disabled mechanical limit implementation. */
static void continuous_watchdog(command_manager_t *m)
{
    if (m == NULL || m->busy) return;
    m->busy = 1U;
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (m->continuous_active[i] && (int32_t)(HAL_GetTick() - m->continuous_until[i]) >= 0)
            request_continuous_stop(m, i);
        if (!m->continuous_stop_pending[i] ||
            (int32_t)(HAL_GetTick() - m->continuous_retry_at[i]) < 0) continue;
        m->target_speed[i] = 0;
        modbus_result_t result = stop_motor(m->motors[i]);
        if (result == MODBUS_OK) result = p100s_read_status(m->motors[i]);
        if (result == MODBUS_OK && m->motors[i]->status.speed_raw == 0)
            m->continuous_stop_pending[i] = 0U;
        m->continuous_retry_at[i] = HAL_GetTick() + 200U;
    }
    m->busy = 0U;
}

static void position_watchdog(command_manager_t *m)
{
    if (!m || m->busy) return;
    m->busy=1U;
    for (uint8_t i=0;i<2;++i) {
        if (!m->position_watch[i] || (int32_t)(HAL_GetTick()-m->position_poll_at[i])<0) continue;
        m->position_poll_at[i]=HAL_GetTick()+100U;
        modbus_result_t result=p100s_read_status(m->motors[i]);
        int64_t travel=((int64_t)m->motors[i]->status.position-m->position_start[i])*m->position_direction[i];
        uint8_t elapsed=(uint8_t)(HAL_GetTick()-m->position_started_at[i]>=3000U);
        if (result==MODBUS_OK && travel>=-(int64_t)m->position_noise[i] &&
            (!elapsed || travel>m->position_noise[i])) {
            if (elapsed) m->position_watch[i]=0U;
            continue;
        }
        /* Wrong initial direction, no measurable start in 3 s, or feedback
           loss: latch a stop. Never replay CTRG or redefine the coordinate. */
        axis_guard_t *g=&m->guard[i];
        g->fault=result==MODBUS_OK ? GUARD_EXECUTION : GUARD_COMM;
        if (result!=MODBUS_OK) g->zero_valid=0U;
        g->phase=HOME_FAULT;g->stop_pending=1U;g->zero_samples=0U;
        g->retry_at=HAL_GetTick();
        (void)Command_Manager_StopAxis(m,i);
    }
    m->busy=0U;
}

void Command_Manager_Task(command_manager_t *m)
{
    position_watchdog(m);
    Trajectory_Task(m);
    GuardController_Task(m);
    continuous_watchdog(m);
    point_task(m);
#if MOTOR_LIMITS_ENABLED
    if (m == NULL || m->busy != 0U) return;
    m->busy = 1U;
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (m->limit_bypass[i] != 0U) {
            if ((int32_t)(HAL_GetTick() - m->limit_bypass_until[i]) < 0) continue;
            m->limit_bypass[i] = 0U;
            /* Lease expiry always requests STOP, even with a clear input. */
            m->limit_handled[i] = m->limits[i].trips - 1U;
            m->limit_retry_at[i] = HAL_GetTick();
        }
        uint32_t trip = m->limits[i].trips;
        if (trip == m->limit_handled[i] ||
            (int32_t)(HAL_GetTick() - m->limit_retry_at[i]) < 0) continue;
        m->target_speed[i] = 0;
        request_continuous_stop(m, i);
        /* Never acknowledge a trip until STOP succeeded and zero speed was
           observed. A second edge during blocking Modbus is retained. */
        modbus_result_t result = stop_motor(m->motors[i]);
        if (result == MODBUS_OK) result = p100s_read_status(m->motors[i]);
        if (result == MODBUS_OK && m->motors[i]->status.speed_raw == 0)
            m->limit_handled[i] = trip;
        m->limit_retry_at[i] = HAL_GetTick() + 200U;
    }
    m->busy = 0U;
#else
    (void)m;
#endif
}
void Command_Manager_TransportLost(command_manager_t *m)
{
    if (!m) return;
    if (Trajectory_Active(&m->trajectory)) Trajectory_Cancel(m,TJ_USER_STOP);
    else memset(&m->trajectory,0,sizeof(m->trajectory));
    for (uint8_t i=0;i<2;++i) {
        m->position_watch[i]=0U;
        m->point_active[i]=0U;
        m->point_completed[i]=0U;
        GuardController_Cancel(m,i);
        m->continuous_active[i]=0;
        m->continuous_stop_pending[i]=1;
        m->continuous_retry_at[i]=HAL_GetTick();
        m->target_speed[i]=0;
        m->guard[i].stop_pending=1;
        m->guard[i].retry_at=HAL_GetTick();
    }
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

    if (c->command==PC_CMD_MOVE_OUTPUT_TRACKED && m->point_active[c->device_id-1U]) {
        uint8_t axis=(uint8_t)(c->device_id-1U);
        modbus_result_t stopped=Command_Manager_StopAxis(m,axis);
        if (stopped!=MODBUS_OK) {
            point_fault(m,axis,GUARD_COMM);
            (void)PC_Comm_SendResponse(c->device_id,c->command,map_status(stopped),c->parameter,0);return;
        }
    }

    /* A received STOP must not run one more point velocity update first. */
    if (c->command==PC_CMD_MOTOR_STOP || c->command==PC_CMD_EMERGENCY_STOP)
        m->point_active[c->device_id-1U]=0U;
    /* A received STOP must not run one more trajectory velocity update first. */
    if (Trajectory_Active(&m->trajectory) &&
        (c->command==PC_CMD_MOTOR_STOP || c->command==PC_CMD_EMERGENCY_STOP ||
         (c->command==PC_CMD_TRAJECTORY_STOP && c->device_id==1U &&
          (!c->parameter || c->parameter==m->trajectory.session))))
        Trajectory_Cancel(m,TJ_USER_STOP);
    /* Prioritize a latched trip ahead of every incoming PC packet. */
    m->point_preflight=(uint8_t)((c->command==PC_CMD_MOVE_AXES_POINT || c->command==PC_CMD_BANK_RUN) &&
        !m->point_active[0] && !m->point_active[1] && !Trajectory_Active(&m->trajectory));
    for (unsigned i=0;i<2U;++i) {
        axis_guard_t *g=&m->guard[i];
        if (m->continuous_active[i] || m->continuous_stop_pending[i] || m->position_watch[i] ||
            g->dual_home || g->dual_pending || g->stop_pending ||
            (g->phase>=HOME_SEEK && g->phase<=HOME_FINAL_STOP)) m->point_preflight=0U;
    }
    Command_Manager_Task(m);
    m->point_preflight=0U;
    if (c->command>=PC_CMD_BANK_BEGIN && c->command<=PC_CMD_BANK_STATUS) {bank_command(m,c);return;}
    if (c->command==PC_CMD_POINT_TIMING) {
        uint8_t data[32];
        for (unsigned i=0;i<2U;++i) {
            put_i32(data+16U*i,(int32_t)m->point_init_ms[i]);
            put_i32(data+16U*i+4U,(int32_t)m->point_cycle_ms[i]);
            put_i32(data+16U*i+8U,(int32_t)m->point_gap_ms[i]);
            put_i32(data+16U*i+12U,(int32_t)m->point_max_gap_ms[i]);
        }
        (void)PC_Comm_SendResponseData(c->device_id,c->command,PC_STATUS_SUCCESS,c->parameter,data,sizeof(data));return;
    }
    if (c->command==PC_CMD_MOVE_AXES_POINT) { axes_point(m,c,c);return; }
    uint8_t index = (uint8_t)(c->device_id - 1U);
    if ((m->point_active[0] || m->point_active[1]) &&
        c->command>=PC_CMD_TRAJECTORY_BEGIN && c->command<=PC_CMD_TRAJECTORY_START) {
        (void)PC_Comm_SendResponse(c->device_id,c->command,PC_STATUS_BUSY,c->parameter,0);return;
    }
    if (c->command>=PC_CMD_TRAJECTORY_BEGIN && c->command<=PC_CMD_TRAJECTORY_CLEAR) {
        m->busy=1;Trajectory_Command(m,c);m->busy=0;return;
    }
    if (Trajectory_Active(&m->trajectory)) {
        if (c->command==PC_CMD_MOTOR_STOP || c->command==PC_CMD_EMERGENCY_STOP) {
            Trajectory_Cancel(m,TJ_USER_STOP);
            (void)PC_Comm_SendResponse(c->device_id,c->command,PC_STATUS_SUCCESS,c->parameter,0);return;
        }
        /* Trajectory diagnostics use cached samples: polling must not add bus
           transactions or change either axis configuration while following. */
        if (c->command==PC_CMD_READ_SPEED) {
            uint8_t data[4]={(uint8_t)((uint16_t)m->motors[index]->status.speed_raw>>8),
                (uint8_t)m->motors[index]->status.speed_raw,0,0}; /* Run state unknown in cached snapshot. */
            (void)PC_Comm_SendResponseData(c->device_id,c->command,PC_STATUS_SUCCESS,c->parameter,data,4);return;
        }
        if (c->command==PC_CMD_READ_POSITION) {
            uint8_t data[4];put_i32(data,m->motors[index]->status.position);
            (void)PC_Comm_SendResponseData(c->device_id,c->command,PC_STATUS_SUCCESS,c->parameter,data,4);return;
        }
        if (c->command==PC_CMD_READ_ALARM) {
            (void)PC_Comm_SendResponse(c->device_id,c->command,PC_STATUS_SUCCESS,c->parameter,
                (int16_t)m->motors[index]->status.alarm_code);return;
        }
        if (c->command!=PC_CMD_READ_FEEDBACK && c->command!=PC_CMD_READ_GUARD) {
            (void)PC_Comm_SendResponse(c->device_id,c->command,PC_STATUS_BUSY,c->parameter,0);return;
        }
    }
    if (c->command==PC_CMD_SET_MOTION_PROFILE &&
        (m->continuous_active[index] || m->continuous_stop_pending[index] || m->guard[index].stop_pending)) {
        (void)PC_Comm_SendResponse(c->device_id,c->command,PC_STATUS_BUSY,c->parameter,0);
        return;
    }
    if ((m->point_active[0] || m->point_active[1]) &&
        ((c->command>=PC_CMD_TRAJECTORY_BEGIN && c->command<=PC_CMD_TRAJECTORY_START) ||
         c->command==PC_CMD_CONFIG_GUARD || c->command==PC_CMD_SET_ZERO ||
         c->command==PC_CMD_SET_REFERENCE || c->command==PC_CMD_CONFIG_DUAL_LIMIT ||
         c->command==PC_CMD_HOME || c->command==PC_CMD_HOME_DUAL_LIMIT)) {
        (void)PC_Comm_SendResponse(c->device_id,c->command,PC_STATUS_BUSY,c->parameter,0);return;
    }
    if (m->point_active[index] && (motion_command(c->command) || c->command==PC_CMD_SET_MOTION_PROFILE) && c->command!=PC_CMD_MOVE_OUTPUT_TRACKED) {
        (void)PC_Comm_SendResponse(c->device_id,c->command,PC_STATUS_BUSY,c->parameter,0);return;
    }
    if (c->command>=PC_CMD_READ_FEEDBACK && c->command<=PC_CMD_HOME_DUAL_LIMIT) {
        if (c->command!=PC_CMD_READ_FEEDBACK && c->command!=PC_CMD_READ_GUARD && c->command!=PC_CMD_READ_DUAL_LIMIT)
            for (unsigned n=0;n<POINT_BANK_SLOTS;++n) m->point_banks[n].ready=0U;
        m->busy=1;
        GuardController_Command(m,c);
        m->busy=0;
        return;
    }
    if (motion_command(c->command) &&
        (m->continuous_active[index] || m->continuous_stop_pending[index])) {
        (void)PC_Comm_SendResponse(c->device_id, c->command, PC_STATUS_BUSY, c->parameter, 0);
        return;
    }
    if (motion_command(c->command) && limit_blocked(m, index)) {
        (void)PC_Comm_SendResponse(c->device_id, c->command, PC_STATUS_LIMIT_ACTIVE, c->parameter, 0);
        return;
    }
    if (motion_command(c->command)) {
        pc_status_t checked=GuardController_Preflight(m,c);
        if (checked!=PC_STATUS_SUCCESS) {
            (void)PC_Comm_SendResponse(c->device_id,c->command,checked,c->parameter,0);
            return;
        }
    }

    m->busy = 1U;
    p100s_handle_t *motor = m->motors[index];
    modbus_result_t result = MODBUS_ERROR_ARGUMENT;
    int16_t value = 0;
    int32_t position = 0;
    uint8_t response_data[5];
    uint8_t response_length = 2U;
    uint8_t response_is_raw = 0U;

    switch (c->command) {
    case PC_CMD_ROTATE_CONTINUOUS: {
        int16_t rpm = 0;
        if (!AxisMotion_ConvertVelocity(read_i32(c->data), m->gear_ratio_num[index],
                                        m->gear_ratio_den[index], &rpm)) break;
        result = p100s_read_status(motor);
        if (result == MODBUS_OK && motor->status.speed_raw != 0) result = MODBUS_ERROR_BUSY;
        uint16_t alarm = 0U;
        if (result == MODBUS_OK) result = p100s_read_alarm(motor, &alarm);
        if (result == MODBUS_OK && alarm != 0U) result = MODBUS_ERROR_BUSY;
        if (result != MODBUS_OK) break;
        m->target_speed[index] = rpm;
        result = start_speed(m, index, motor);
        if (result == MODBUS_OK) {
            m->continuous_active[index] = 1U;
            m->continuous_until[index] = HAL_GetTick() + PC_ROTATE_LEASE_MS;
        } else {
            /* A write may have executed despite a missing response. */
            m->continuous_stop_pending[index] = 1U;
            m->continuous_retry_at[index] = HAL_GetTick();
        }
        value = rpm; /* Report quantized motor rpm so PC can show actual deg/s. */
        break;
    }
    case PC_CMD_ROTATE_KEEPALIVE:
        if (c->parameter != PC_ROTATE_KEEPALIVE_KEY) break;
        if (m->guard[index].phase==HOME_DONE || m->guard[index].phase==HOME_FAULT) {
            result=MODBUS_OK;value=0;break; /* Terminal home status, never a restart. */
        }
        if (!m->continuous_active[index]) { result = MODBUS_ERROR_BUSY; break; }
        m->continuous_until[index] = HAL_GetTick() + PC_ROTATE_LEASE_MS;
        result = MODBUS_OK;
        value = m->target_speed[index];
        break;
#if MOTOR_LIMITS_ENABLED
    case PC_CMD_LIMIT_DEBUG:
        /* Explicit per-axis session mode; never saved to flash. */
        if (c->parameter != PC_LIMIT_DEBUG_KEY || c->data[0] > 2U) break;
        if (c->data[0] == 2U) {
            /* A late keepalive must not silently re-enable expired bypass. */
            if (m->limit_bypass[index] == 0U) { result = MODBUS_ERROR_BUSY; break; }
            m->limit_bypass_until[index] = HAL_GetTick() + PC_LIMIT_DEBUG_LEASE_MS;
            result = MODBUS_OK;
        } else if (c->data[0] == 0U) {
            m->limit_bypass[index] = 0U;
            m->target_speed[index] = 0;
            m->limit_handled[index] = m->limits[index].trips - 1U;
            m->limit_retry_at[index] = HAL_GetTick();
            result = MODBUS_OK;
        } else {
            result = stop_motor(motor);
            if (result == MODBUS_OK) result = p100s_read_status(motor);
            if (result == MODBUS_OK && motor->status.speed_raw != 0) result = MODBUS_ERROR_BUSY;
            if (result == MODBUS_OK) {
                m->target_speed[index] = 0;
                m->limit_bypass[index] = 1U;
                m->limit_bypass_until[index] = HAL_GetTick() + PC_LIMIT_DEBUG_LEASE_MS;
            }
        }
        value = m->limit_bypass[index];
        break;
#endif
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
        GuardController_Cancel(m,index);
        request_continuous_stop(m, index);
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
        GuardController_Cancel(m,index);
        request_continuous_stop(m, index);
        result = Command_Manager_StopAxis(m,index);
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
#if MOTOR_LIMITS_ENABLED
            response_data[4] = (uint8_t)((m->limits[index].active != 0U ? 1U : 0U) |
                                       (m->limits[index].trips != m->limit_handled[index] ? 2U : 0U) |
                                       (m->limit_bypass[index] != 0U ? 4U : 0U));
            response_length = 5U;
#else
            /* Legacy 4-byte telemetry: no limit channel is monitored. */
            response_length = 4U;
#endif
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
        GuardController_Cancel(m,index);
        if (m->continuous_active[index] || m->continuous_stop_pending[index]) {
            request_continuous_stop(m, index);
            result = stop_motor(motor);
            if (result == MODBUS_OK) result = p100s_read_status(motor);
            if (result != MODBUS_OK) break;
            if (motor->status.speed_raw != 0) { result = MODBUS_ERROR_BUSY; break; }
        }
        result = reset_alarm(motor);
        break;
    case PC_CMD_EMERGENCY_STOP: {
        GuardController_Cancel(m,index);
        request_continuous_stop(m, index);
        modbus_result_t stop_result = Command_Manager_StopAxis(m,index);
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
    case PC_CMD_MOVE_OUTPUT_RELATIVE:
    case PC_CMD_MOVE_OUTPUT_TRACKED: {
        uint16_t ppr = 0U;
        axis_motion_command_t motion;
        uint16_t output_speed = read_u16(c->data);
        int32_t output_angle = read_i32(c->data + 2U);
        result = p100s_read_parameter(motor, P100S_GROUP_PA, P100S_PA_PULSES_PER_REV, &ppr);
        if (result == MODBUS_OK &&
            !AxisMotion_ConvertRelative(output_speed, output_angle,
                                        m->gear_ratio_num[index], m->gear_ratio_den[index],
                                        ppr, &motion))
            result = MODBUS_ERROR_ARGUMENT;
        if (result == MODBUS_OK) {
            int32_t ignored_motor_position = 0;
            if (m->guard[index].angle_sign<0 && motion.motor_pulses==INT32_MIN) result=MODBUS_ERROR_ARGUMENT;
            else if (c->command==PC_CMD_MOVE_OUTPUT_TRACKED)
                result=start_point(m,index,motion.motor_speed_rpm,
                    m->guard[index].angle_sign<0 ? -motion.motor_pulses : motion.motor_pulses,0U,NULL);
            else result = move_position(m, index, motor, 0U, motion.motor_speed_rpm,
                                   m->guard[index].angle_sign<0 ? -motion.motor_pulses : motion.motor_pulses, &ignored_motor_position);
        }
        if (result == MODBUS_OK) position = output_angle;
        response_length = 4U;
        break;
    }
    case PC_CMD_SET_MOTION_PROFILE:
        result = set_motion_profile(motor, read_u16(c->data), read_u16(c->data + 2U),
                                    read_u16(c->data + 4U));
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
    pc_status_t status = map_status(result);
    if (motion_command(c->command) && limit_blocked(m, index)) status = PC_STATUS_LIMIT_ACTIVE;
    (void)PC_Comm_SendResponseData(c->device_id, c->command, status, c->parameter,
                                   response_data, response_length);
    m->busy = 0U;
}
