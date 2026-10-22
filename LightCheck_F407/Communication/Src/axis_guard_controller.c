#include "axis_guard_controller.h"
#include "axis_motion.h"
#include "pc_comm.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>

static int32_t i32(const uint8_t *p) { return (int32_t)(((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]); }
static void p16(uint8_t *p, uint16_t n) { p[0]=(uint8_t)(n>>8); p[1]=(uint8_t)n; }
static void p32(uint8_t *p, int32_t n) { uint32_t v=(uint32_t)n; p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v; }
static uint8_t homing(const axis_guard_t *g) { return g->phase >= HOME_SEEK && g->phase <= HOME_FINAL_STOP; }
static uint8_t channel(const axis_guard_t *g, unsigned role) { return role==0 ? g->negative : role==1 ? g->positive : g->home; }
static void acknowledge(command_manager_t *m, unsigned i)
{
    for (unsigned r=0; r<3; ++r) m->guard[i].seen[r]=AxisGuard_Trips(&m->guard_inputs,&m->guard[i],channel(&m->guard[i],r));
}
static pc_status_t status(modbus_result_t r)
{
    if (r==MODBUS_OK) return PC_STATUS_SUCCESS;
    if (r==MODBUS_ERROR_ARGUMENT) return PC_STATUS_ILLEGAL_PARAM;
    if (r==MODBUS_ERROR_BUSY) return PC_STATUS_BUSY;
    return r==MODBUS_ERROR_TIMEOUT ? PC_STATUS_SERVO_NO_RESPONSE : PC_STATUS_RS485_ERROR;
}
static modbus_result_t units(command_manager_t *m, unsigned i, uint16_t *mode, uint16_t *ppr)
{
    modbus_result_t r=p100s_read_parameter(m->motors[i],P100S_GROUP_P3,32,mode);
    if (r==MODBUS_OK) r=p100s_read_parameter(m->motors[i],P100S_GROUP_PA,11,ppr);
    if (r==MODBUS_OK && !*ppr) r=MODBUS_ERROR_ARGUMENT;
    return r;
}
static int coordinate(command_manager_t *m, unsigned i, int32_t *angle)
{
    uint16_t mode=0,ppr=0;
    if (units(m,i,&mode,&ppr)!=MODBUS_OK ||
        !AxisGuard_Angle(&m->guard[i],m->motors[i]->status.position,mode,ppr,
                        m->gear_ratio_num[i],m->gear_ratio_den[i],angle)) {
        m->guard[i].zero_valid=0;
        return 0;
    }
    return 1;
}
static void fault(command_manager_t *m, unsigned i, uint8_t reason)
{
    axis_guard_t *g=&m->guard[i];
    if (homing(g)) { g->phase=HOME_FAULT; g->zero_valid=0; }
    g->fault=reason; g->stop_pending=1; g->zero_samples=0; g->retry_at=HAL_GetTick();
    if (reason==GUARD_FEEDBACK || reason==GUARD_COMM || reason==GUARD_ALARM) g->zero_valid=0;
    if (m->continuous_active[i]) {
        m->continuous_stop_pending[i]=1;m->continuous_retry_at[i]=HAL_GetTick();
    }
    m->continuous_active[i]=0;
    m->target_speed[i]=0;
}
void GuardController_Cancel(command_manager_t *m, uint8_t i)
{
    if (homing(&m->guard[i])) fault(m,i,GUARD_CANCELLED);
}
static int prepare_units(command_manager_t *m, unsigned i)
{
    uint16_t mode=0,ppr=0;
    if (units(m,i,&mode,&ppr)!=MODBUS_OK) return 0;
    if (mode!=3) {
        if (p100s_write_parameter(m->motors[i],P100S_GROUP_P3,32,3,P100S_STORAGE_SAVED)!=MODBUS_OK ||
            units(m,i,&mode,&ppr)!=MODBUS_OK || mode!=3) return 0;
    }
    if (p100s_read_status(m->motors[i])!=MODBUS_OK || m->motors[i]->status.speed_raw!=0 ||
        m->motors[i]->status.alarm_code) return 0;
    m->guard[i].ppr=ppr;
    return 1;
}
static uint8_t blocked_direction(command_manager_t *m, unsigned i, int32_t direction)
{
    axis_guard_t *g=&m->guard[i];
    if (!g->hardware_on) return 0;
    if (direction<0 && AxisGuard_Active(&m->guard_inputs,g,g->negative)) return 1;
    if (direction>0 && AxisGuard_Active(&m->guard_inputs,g,g->positive)) return 1;
    if (g->home_at_end && AxisGuard_Active(&m->guard_inputs,g,g->home) &&
        (!g->confirmed || !g->home_at_end || direction*g->home_direction>0)) return 1;
    return 0;
}
pc_status_t GuardController_Preflight(command_manager_t *m, const pc_command_packet_t *c)
{
    unsigned i=c->device_id-1U;
    axis_guard_t *g=&m->guard[i];
    if (g->stop_pending || homing(g)) return PC_STATUS_BUSY;
    if (g->phase==HOME_DONE || g->phase==HOME_FAULT) g->phase=HOME_IDLE;
    if (g->fixed_range && !g->zero_valid) return PC_STATUS_NOT_REFERENCED;
    if (!g->hardware_on && !g->soft_on) return PC_STATUS_SUCCESS;
    int32_t delta=0,angle=0;
    if (c->command==PC_CMD_MOVE_OUTPUT_RELATIVE) delta=i32(c->data+2);
    else if (c->command==PC_CMD_ROTATE_CONTINUOUS) {
        if (g->soft_on) return PC_STATUS_RANGE; /* No unbounded motion in a bounded coordinate. */
        delta=i32(c->data);
    } else if (c->command==PC_CMD_MOVE_RELATIVE || c->command==PC_CMD_JOG) {
        uint16_t mode=0,ppr=0;
        if (units(m,i,&mode,&ppr)!=MODBUS_OK) return PC_STATUS_SERVO_NO_RESPONSE;
        int64_t pulses=c->command==PC_CMD_JOG ? i32(c->data+3) : i32(c->data+2);
        if (c->command==PC_CMD_JOG) { if (pulses<0) pulses=-pulses; if (c->data[0]==0) pulses=-pulses; }
        int64_t scaled=pulses*360000LL*m->gear_ratio_den[i]/((int64_t)ppr*m->gear_ratio_num[i]);
        if (scaled>INT32_MAX || scaled<INT32_MIN) return PC_STATUS_RANGE;
        delta=(int32_t)scaled;
    } else {
        /* Generic PA writes, legacy speed starts and raw absolute motion could
           bypass coordinate checks. Use the typed output-axis commands instead. */
        return PC_STATUS_RANGE;
    }
    if (blocked_direction(m,i,delta)) return PC_STATUS_LIMIT_ACTIVE;
    if (g->soft_on) {
        if (p100s_read_status(m->motors[i])!=MODBUS_OK) return PC_STATUS_SERVO_NO_RESPONSE;
        if (!coordinate(m,i,&angle)) { fault(m,i,GUARD_FEEDBACK); return PC_STATUS_NOT_REFERENCED; }
        if (!AxisGuard_TargetAllowed(g,angle,delta)) return PC_STATUS_RANGE;
    }
    return PC_STATUS_SUCCESS;
}
static void start_stage(command_manager_t *m, unsigned i, uint8_t phase, int16_t speed)
{
    axis_guard_t *g=&m->guard[i];
    if (Command_Manager_RunVelocity(m,(uint8_t)i,speed)!=MODBUS_OK) { fault(m,i,GUARD_COMM); return; }
    g->phase=phase; g->stage_start=m->motors[i]->status.position; g->zero_samples=0;
    g->progress_at=HAL_GetTick();
    acknowledge(m,i);
}
static void home_task(command_manager_t *m, unsigned i)
{
    axis_guard_t *g=&m->guard[i]; p100s_handle_t *motor=m->motors[i];
    uint16_t mode=0,ppr=0;
    if (!m->continuous_active[i] || (int32_t)(HAL_GetTick()-m->continuous_until[i])>=0 ||
        (int32_t)(HAL_GetTick()-g->home_deadline)>=0) { fault(m,i,GUARD_TIMEOUT); return; }
    if (units(m,i,&mode,&ppr)!=MODBUS_OK || mode!=3 || ppr!=g->ppr) { fault(m,i,GUARD_FEEDBACK); return; }
    int64_t step=(int64_t)motor->status.position-g->home_last;
    if (step>INT32_MAX || step<INT32_MIN) { fault(m,i,GUARD_FEEDBACK); return; }
    if (g->phase==HOME_SEEK || g->phase==HOME_LATCH || g->phase==HOME_BACKOFF) {
        if (step) g->progress_at=HAL_GetTick();
        else if (HAL_GetTick()-g->progress_at>3000U) { fault(m,i,GUARD_FEEDBACK); return; }
        int sign=g->home_rpm>0 ? 1 : -1;
        if (g->phase==HOME_BACKOFF) sign=-sign;
        if (step*sign < -(int64_t)(g->ppr/100U+1U)) { fault(m,i,GUARD_FEEDBACK); return; }
    }
    g->travel_counts+=(uint64_t)(step<0 ? -step : step);
    g->home_last=motor->status.position;
    if (g->travel_counts*360000ULL*m->gear_ratio_den[i] >
        (uint64_t)g->home_budget*g->ppr*m->gear_ratio_num[i]) { fault(m,i,GUARD_RANGE); return; }
    for (unsigned role=0; role<2; ++role) {
        uint8_t ch=channel(g,role);
        if (ch!=g->home && (AxisGuard_Active(&m->guard_inputs,g,ch) ||
            AxisGuard_Trips(&m->guard_inputs,g,ch)!=g->seen[role])) { fault(m,i,GUARD_SENSOR); return; }
    }
    uint8_t hit=(uint8_t)(AxisGuard_Active(&m->guard_inputs,g,g->home) ||
        AxisGuard_Trips(&m->guard_inputs,g,g->home)!=g->seen[2]);
    if ((g->phase==HOME_SEEK || g->phase==HOME_LATCH) && hit) {
        if (Command_Manager_StopAxis(m,(uint8_t)i)!=MODBUS_OK) { fault(m,i,GUARD_COMM); return; }
        g->phase=g->phase==HOME_SEEK ? HOME_CONTACT_STOP : HOME_FINAL_STOP;
        g->zero_samples=0;
    } else if (g->phase==HOME_BACKOFF) {
        int64_t d=(int64_t)motor->status.position-g->stage_start;
        if ((uint64_t)(d<0 ? -d : d)*360000ULL*m->gear_ratio_den[i] >
            (uint64_t)g->backoff_budget*g->ppr*m->gear_ratio_num[i]) { fault(m,i,GUARD_SENSOR); return; }
        if (!AxisGuard_Active(&m->guard_inputs,g,g->home)) {
            if (Command_Manager_StopAxis(m,(uint8_t)i)!=MODBUS_OK) { fault(m,i,GUARD_COMM); return; }
            g->phase=HOME_BACKOFF_STOP; g->zero_samples=0;
        }
    } else if (g->phase==HOME_CONTACT_STOP || g->phase==HOME_BACKOFF_STOP || g->phase==HOME_FINAL_STOP) {
        g->zero_samples=motor->status.speed_raw==0 ? (uint8_t)(g->zero_samples+1U) : 0U;
        if (g->zero_samples<2) return;
        if (g->phase==HOME_CONTACT_STOP) start_stage(m,i,HOME_BACKOFF,(int16_t)-g->home_rpm);
        else if (g->phase==HOME_BACKOFF_STOP) {
            if (AxisGuard_Active(&m->guard_inputs,g,g->home)) { fault(m,i,GUARD_SENSOR); return; }
            int16_t slow=(int16_t)(g->home_rpm/2);
            if (!slow) slow=g->home_rpm>0 ? 1 : -1;
            start_stage(m,i,HOME_LATCH,slow);
        } else {
            if (!AxisGuard_Active(&m->guard_inputs,g,g->home)) { fault(m,i,GUARD_SENSOR); return; }
            g->origin=motor->status.position; g->zero_valid=1; g->phase=HOME_DONE; g->fault=GUARD_OK;
            g->reference_angle=g->fixed_range ? 25000 : 0;
            if (g->fixed_range) g->soft_on=1U;
            memset(&m->trajectory,0,sizeof(m->trajectory));
            m->continuous_active[i]=0; m->target_speed[i]=0; acknowledge(m,i);
        }
    }
}
void GuardController_Task(command_manager_t *m)
{
    if (!m || m->busy || Trajectory_Active(&m->trajectory)) return;
    m->busy=1;
    for (unsigned i=0; i<2; ++i) {
        axis_guard_t *g=&m->guard[i];
        if (g->hardware_on && !homing(g) && !g->stop_pending) {
            for (unsigned r=0; r<(g->home_at_end ? 3U : 2U); ++r)
                if (AxisGuard_Trips(&m->guard_inputs,g,channel(g,r))!=g->seen[r]) fault(m,i,GUARD_SENSOR);
        }
        if (g->stop_pending) {
            if ((int32_t)(HAL_GetTick()-g->retry_at)>=0) {
                modbus_result_t result=Command_Manager_StopAxis(m,(uint8_t)i);
                if (result==MODBUS_OK) result=p100s_read_status(m->motors[i]);
                if (result==MODBUS_OK && m->motors[i]->status.speed_raw==0) {
                    g->stop_pending=0; acknowledge(m,i);
                }
                g->retry_at=HAL_GetTick()+200U;
            }
            continue;
        }
        if ((!g->soft_on && !homing(g)) || (int32_t)(HAL_GetTick()-g->poll_at)<0) continue;
        if (g->soft_on && !g->zero_valid && !homing(g)) continue; /* Fault is latched; await explicit re-reference. */
        g->poll_at=HAL_GetTick()+100U;
        if (p100s_read_status(m->motors[i])!=MODBUS_OK) { fault(m,i,GUARD_COMM); continue; }
        if (m->motors[i]->status.alarm_code) { fault(m,i,GUARD_ALARM); continue; }
        if (homing(g)) home_task(m,i);
        else {
            int32_t angle=0;
            if (!coordinate(m,i,&angle)) fault(m,i,GUARD_FEEDBACK);
            else if (angle<g->minimum || angle>g->maximum) fault(m,i,GUARD_RANGE);
        }
    }
    m->busy=0;
}
static pc_status_t configure(command_manager_t *m, unsigned i, const pc_command_packet_t *c)
{
    axis_guard_t *g=&m->guard[i];
    if (homing(g) || m->continuous_active[i]) return PC_STATUS_BUSY;
    modbus_result_t r=Command_Manager_StopAxis(m,(uint8_t)i);
    if (r==MODBUS_OK) r=p100s_read_status(m->motors[i]);
    if (r!=MODBUS_OK) return status(r);
    if (m->motors[i]->status.speed_raw!=0 || g->stop_pending) return PC_STATUS_BUSY;
    if (c->parameter==1) {
        for (unsigned n=0;n<3;++n) if (c->data[n]>=6 && c->data[n]!=255) return PC_STATUS_ILLEGAL_PARAM;
        if (c->data[3]>1 || (c->data[0]!=255 && c->data[0]==c->data[1])) return PC_STATUS_ILLEGAL_PARAM;
        axis_guard_t *other=&m->guard[1U-i];
        if (other->hardware_on)
            for (unsigned n=0;n<3;++n) for (unsigned r0=0;r0<3;++r0)
                if (c->data[n]!=255 && c->data[n]==channel(other,r0)) return PC_STATUS_ILLEGAL_PARAM;
        if (g->negative!=c->data[0] || g->positive!=c->data[1] || g->home!=c->data[2] || g->active_high!=c->data[3]) {
            if (g->soft_on || g->hardware_on) return PC_STATUS_BUSY; /* Disarm explicitly first. */
            g->zero_valid=0;
        }
        g->negative=c->data[0];g->positive=c->data[1];g->home=c->data[2];g->active_high=c->data[3];
        acknowledge(m,i);
    } else if (c->parameter==2) {
        int32_t lo=i32(c->data),hi=i32(c->data+4);
        if (lo>0 || hi<0 || lo>=hi || lo < -36000000 || hi>36000000) return PC_STATUS_ILLEGAL_PARAM;
        if (g->fixed_range && (lo!=-35000 || hi!=35000)) return PC_STATUS_RANGE;
        if (g->soft_on) {
            int32_t angle=0;
            if (!coordinate(m,i,&angle)) return PC_STATUS_NOT_REFERENCED;
            if (angle<lo || angle>hi) return PC_STATUS_RANGE;
        }
        g->minimum=lo;g->maximum=hi;
    } else if (c->parameter==3) {
        uint8_t flags=c->data[0]; int8_t dir=c->data[1]==255 ? -1 : c->data[1]==1 ? 1 : 0;
        if (flags>15 || (c->data[1]!=0 && c->data[1]!=1 && c->data[1]!=255) || ((flags&4) && !dir)) return PC_STATUS_ILLEGAL_PARAM;
        if ((flags&1) && g->negative==255 && g->positive==255 && g->home==255) return PC_STATUS_ILLEGAL_PARAM;
        if (flags&1) {
            axis_guard_t *other=&m->guard[1U-i];
            if (other->hardware_on)
                for (unsigned a=0;a<3;++a) for (unsigned b=0;b<3;++b)
                    if (channel(g,a)!=255 && channel(g,a)==channel(other,b)) return PC_STATUS_ILLEGAL_PARAM;
        }
        if ((flags&2) && !g->zero_valid) return PC_STATUS_NOT_REFERENCED;
        if (g->fixed_range && g->zero_valid && !(flags&2)) return PC_STATUS_RANGE;
        if ((flags&8) && ((dir<0 && g->home!=255 && g->home==g->positive) ||
                         (dir>0 && g->home!=255 && g->home==g->negative))) return PC_STATUS_ILLEGAL_PARAM;
        if (flags&2) {
            int32_t angle=0;
            if (!coordinate(m,i,&angle)) return PC_STATUS_NOT_REFERENCED;
            if (angle<g->minimum || angle>g->maximum) return PC_STATUS_RANGE;
        }
        g->hardware_on=flags&1;g->soft_on=(flags>>1)&1;g->confirmed=(flags>>2)&1;g->home_at_end=(flags>>3)&1;g->home_direction=dir;
        acknowledge(m,i);
    } else return PC_STATUS_ILLEGAL_PARAM;
    g->fault=GUARD_OK;
    return PC_STATUS_SUCCESS;
}
void GuardController_Command(command_manager_t *m, const pc_command_packet_t *c)
{
    unsigned i=c->device_id-1U; axis_guard_t *g=&m->guard[i];
    uint8_t data[40]={0}; uint8_t size=2; pc_status_t result=PC_STATUS_SUCCESS;
    if (c->command==PC_CMD_READ_GUARD) {
        data[0]=g->negative;data[1]=g->positive;data[2]=g->home;data[3]=g->active_high;
        data[4]=(uint8_t)g->home_direction;data[5]=g->confirmed;data[6]=g->hardware_on;data[7]=g->soft_on;
        p32(data+8,g->minimum);p32(data+12,g->maximum);data[16]=g->home_at_end;size=17;
    } else if (c->command==PC_CMD_CONFIG_GUARD) result=configure(m,i,c);
    else if (c->command==PC_CMD_READ_FEEDBACK) {
        uint16_t mode=0,ppr=0,bits=65535;int32_t angle=0;
        uint8_t cached=Trajectory_Active(&m->trajectory);
        modbus_result_t r=cached ? MODBUS_OK : p100s_read_status(m->motors[i]);
        if (cached) { mode=3;ppr=g->ppr; }
        else if (r==MODBUS_OK) r=units(m,i,&mode,&ppr);
        if (r!=MODBUS_OK) result=status(r);
        else {
            if (!cached) (void)p100s_read_parameter(m->motors[i],P100S_GROUP_PA,95,&bits);
            if (!AxisGuard_Angle(g,m->motors[i]->status.position,mode,ppr,m->gear_ratio_num[i],m->gear_ratio_den[i],&angle)) {
                g->zero_valid=0;
                if (g->soft_on && !homing(g)) fault(m,i,GUARD_FEEDBACK);
            }
            p32(data,m->motors[i]->status.position);p32(data+4,m->motors[i]->status.command_position);p32(data+8,m->motors[i]->status.position_error);
            p16(data+12,mode);p16(data+14,ppr);p16(data+16,bits);p32(data+18,angle);
            p16(data+22,(uint16_t)(g->zero_valid | (g->soft_on<<1) | (g->hardware_on<<2) | (g->confirmed<<3) | (g->stop_pending<<4)));
            data[24]=m->guard_inputs.raw;
            for (unsigned ch=0;ch<6;++ch) data[25]|=(uint8_t)(AxisGuard_Active(&m->guard_inputs,g,(uint8_t)ch)<<ch);
            data[26]=g->phase;data[27]=g->fault;p32(data+28,g->origin);
            p16(data+32,(uint16_t)m->motors[i]->status.speed_raw);size=34;
        }
    } else if (c->command==PC_CMD_SET_ZERO || c->command==PC_CMD_SET_REFERENCE) {
        int32_t angle=c->command==PC_CMD_SET_REFERENCE ? i32(c->data) : (g->fixed_range ? 25000 : 0);
        if (c->command==PC_CMD_SET_REFERENCE) { p32(data,angle);size=4; }
        if (c->parameter!=(c->command==PC_CMD_SET_REFERENCE ? 0x5246U : 0x5A52U) ||
            angle < -36000000 || angle > 36000000) result=PC_STATUS_ILLEGAL_PARAM;
        else if ((g->fixed_range && (angle < -35000 || angle > 35000)) ||
                 (g->soft_on && (angle<g->minimum || angle>g->maximum))) result=PC_STATUS_RANGE;
        else if (homing(g) || m->continuous_active[i] || g->stop_pending) result=PC_STATUS_BUSY;
        else if (Command_Manager_StopAxis(m,(uint8_t)i)!=MODBUS_OK || !prepare_units(m,i)) result=PC_STATUS_BUSY;
        else {
            uint16_t first_ppr=g->ppr,mode=0,ppr=0;
            HAL_Delay(20U);
            modbus_result_t r=p100s_read_status(m->motors[i]);
            if (r!=MODBUS_OK) result=status(r);
            else if (m->motors[i]->status.speed_raw || m->motors[i]->status.alarm_code) result=PC_STATUS_BUSY;
            else if (units(m,i,&mode,&ppr)!=MODBUS_OK || mode!=3U || ppr!=first_ppr) result=PC_STATUS_NOT_REFERENCED;
            else {
                g->origin=m->motors[i]->status.position;g->reference_angle=angle;
                g->zero_valid=1;g->phase=HOME_IDLE;g->fault=0;
                if (g->fixed_range) g->soft_on=1U;
                /* Every reference change invalidates buffered targets in the
                   previous coordinate system; never restart a stale task. */
                memset(&m->trajectory,0,sizeof(m->trajectory));
            }
        }
    } else if (c->command==PC_CMD_HOME) {
        int32_t speed=i32(c->data),budget=i32(c->data+4),backoff=i32(c->data+8),timeout=i32(c->data+12);
        int16_t rpm=0;
        if (c->parameter!=0x484D || speed<=0 || speed>5000 || budget<=0 || budget>360000 ||
            backoff<=0 || backoff>10000 || backoff>budget || timeout<1000 || timeout>120000 ||
            !AxisMotion_ConvertVelocity(speed,m->gear_ratio_num[i],m->gear_ratio_den[i],&rpm)) result=PC_STATUS_ILLEGAL_PARAM;
        else if (!g->hardware_on || !g->confirmed || !g->home_direction || g->home==255) result=PC_STATUS_NOT_REFERENCED;
        else if (homing(g) || m->continuous_active[i] || g->stop_pending) result=PC_STATUS_BUSY;
        else if (AxisGuard_Active(&m->guard_inputs,g,g->home) ||
                 AxisGuard_Active(&m->guard_inputs,g,g->negative) || AxisGuard_Active(&m->guard_inputs,g,g->positive)) result=PC_STATUS_LIMIT_ACTIVE;
        else if (Command_Manager_StopAxis(m,(uint8_t)i)!=MODBUS_OK || !prepare_units(m,i)) result=PC_STATUS_BUSY;
        else {
            g->zero_valid=0;g->fault=0;g->home_rpm=(int16_t)(rpm*g->home_direction);
            g->home_budget=(uint32_t)budget;g->backoff_budget=(uint32_t)backoff;
            g->home_deadline=HAL_GetTick()+(uint32_t)timeout;g->home_start=g->home_last=m->motors[i]->status.position;g->travel_counts=0;
            m->continuous_active[i]=1;m->continuous_until[i]=HAL_GetTick()+PC_ROTATE_LEASE_MS;
            start_stage(m,i,HOME_SEEK,g->home_rpm);
            if (g->phase==HOME_FAULT) result=PC_STATUS_RS485_ERROR;
        }
    }
    (void)PC_Comm_SendResponseData(c->device_id,c->command,result,c->parameter,data,size);
}
