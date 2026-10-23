/* Experimental buffered, common-clock trajectory follower over RS485.
 * Not a hard-synchronous interpolator: slave writes and feedback are sequential.
 * All bus I/O stays in the foreground; the ISR only latches sensor inputs. */
#include "trajectory.h"
#include "command_manager.h"
#include "axis_guard_controller.h"
#include "pc_comm.h"
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#define TJ_MAX_GAP 1000U
#define TJ_MARGIN 1000 /* fixed 1 degree point clearance, plus braking check */
static uint16_t u16(const uint8_t *p) { return (uint16_t)(((uint16_t)p[0]<<8)|p[1]); }
static uint32_t u32(const uint8_t *p) { return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }
static void put16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)(v>>8);p[1]=(uint8_t)v; }
static void put32(uint8_t *p,uint32_t v) { p[0]=(uint8_t)(v>>24);p[1]=(uint8_t)(v>>16);p[2]=(uint8_t)(v>>8);p[3]=(uint8_t)v; }
static int64_t magnitude(int64_t v) { return v<0 ? -v : v; }
static uint8_t channel(const axis_guard_t *g,unsigned r) { return r==0 ? g->negative : r==1 ? g->positive : g->home; }
uint8_t Trajectory_Active(const trajectory_t *t)
{ return t->state==TJ_RUNNING || t->state==TJ_SETTLING || t->state==TJ_STOPPING; }
void Trajectory_Cancel(command_manager_t *m,uint8_t reason)
{
    trajectory_t *t=&m->trajectory;
    if (!Trajectory_Active(t)) return;
    if (t->state!=TJ_STOPPING) {
        t->state=TJ_STOPPING;t->fault=reason;t->zero_samples=0;t->retry_at=HAL_GetTick();
        t->velocity[0]=t->velocity[1]=0;
        if (reason==TJ_COMM || reason==TJ_REFERENCE)
            m->guard[0].zero_valid=m->guard[1].zero_valid=0;
    }
}
static uint8_t input_trip(command_manager_t *m)
{
    trajectory_t *t=&m->trajectory;
    for (unsigned i=0;i<2;++i) {
        axis_guard_t *g=&m->guard[i];
        if (g->dual_enabled)
            for (unsigned n=0;n<2;++n)
                if (AxisGuard_Active(&m->guard_inputs,g,g->dual_channel[n]) ||
                    AxisGuard_Trips(&m->guard_inputs,g,g->dual_channel[n])!=g->dual_seen[n]) return 1;
        if (!g->hardware_on) continue;
        for (unsigned r=0;r<(g->home_at_end ? 3U : 2U);++r) {
            uint8_t ch=channel(g,r);
            if (AxisGuard_Active(&m->guard_inputs,g,ch) ||
                AxisGuard_Trips(&m->guard_inputs,g,ch)!=t->trips[i][r]) return 1;
        }
    }
    return 0;
}
static int read_pose(command_manager_t *m,unsigned i,uint8_t check_units)
{
    axis_guard_t *g=&m->guard[i];uint16_t mode=3,ppr=g->ppr;
    m->trajectory.sample_valid[i]=0U;
    if ((!i && !g->soft_on) || !g->zero_valid || g->stop_pending || g->fault) return TJ_REFERENCE;
    if (check_units) {
        if (p100s_read_parameter(m->motors[i],P100S_GROUP_P3,32,&mode)!=MODBUS_OK ||
            p100s_read_parameter(m->motors[i],P100S_GROUP_PA,11,&ppr)!=MODBUS_OK) return TJ_COMM;
    }
    if (p100s_read_status(m->motors[i])!=MODBUS_OK) return TJ_COMM;
    m->trajectory.sampled_at[i]=HAL_GetTick();m->trajectory.sample_valid[i]=1U;
    if (m->motors[i]->status.alarm_code) return TJ_ALARM;
    if (!AxisGuard_Angle(g,m->motors[i]->status.position,mode,ppr,
        m->gear_ratio_num[i],m->gear_ratio_den[i],&m->trajectory.actual[i])) return TJ_REFERENCE;
    if (g->soft_on && (m->trajectory.actual[i]<g->minimum || m->trajectory.actual[i]>g->maximum)) return TJ_RANGE;
    return TJ_OK;
}
static int validate(command_manager_t *m)
{
    trajectory_t *t=&m->trajectory;
    if (t->loaded!=t->count || t->count<2 || t->points[0].time!=0 ||
        t->points[t->count-1].time<1000U) return 0;
    for (unsigned i=0;i<2;++i) {
        axis_guard_t *g=&m->guard[i];
        if ((!i && !g->soft_on) || !g->zero_valid) return 0;
        int64_t previous_v=0;uint32_t previous_dt=0;
        for (unsigned n=0;n<t->count;++n) {
            trajectory_point_t *p=&t->points[n];
            if (g->soft_on && (p->angle[i]<(int64_t)g->minimum+TJ_MARGIN ||
                p->angle[i]>(int64_t)g->maximum-TJ_MARGIN)) return 0;
            if (!n) continue;
            uint32_t dt=p->time-t->points[n-1].time;
            int64_t v=((int64_t)p->angle[i]-t->points[n-1].angle[i])*1000/dt;
            if (magnitude(v)>t->maximum_speed) return 0;
            if (n>1 && magnitude(v-previous_v)*2000 >
                (int64_t)t->maximum_accel*(dt+previous_dt)) return 0;
            previous_v=v;previous_dt=dt;
        }
    }
    return 1;
}
static void sample(const trajectory_t *t,uint32_t elapsed,unsigned axis,int32_t *position,int32_t *velocity)
{
    if (elapsed>=t->points[t->count-1].time) {
        *position=t->points[t->count-1].angle[axis];*velocity=0;return;
    }
    unsigned n=1;
    while (n+1U<t->count && t->points[n].time<=elapsed) ++n;
    const trajectory_point_t *a=&t->points[n-1],*b=&t->points[n];
    int64_t delta=(int64_t)b->angle[axis]-a->angle[axis];
    uint32_t dt=b->time-a->time;
    *position=(int32_t)(a->angle[axis]+delta*(elapsed-a->time)/dt);
    *velocity=(int32_t)(delta*1000/dt);
}
static void stopping(command_manager_t *m)
{
    trajectory_t *t=&m->trajectory;
    if ((int32_t)(HAL_GetTick()-t->retry_at)<0) return;
    uint8_t zero=1;
    for (unsigned i=0;i<2;++i) {
        /* Always attempt BOTH axes even if the first fails. */
        modbus_result_t r=Command_Manager_StopAxis(m,(uint8_t)i);
        modbus_result_t feedback=p100s_read_status(m->motors[i]);
        t->sample_valid[i]=(uint8_t)(feedback==MODBUS_OK);
        if (feedback==MODBUS_OK) t->sampled_at[i]=HAL_GetTick();
        if (r!=MODBUS_OK || feedback!=MODBUS_OK || m->motors[i]->status.speed_raw!=0) zero=0;
    }
    t->zero_samples=zero ? (uint8_t)(t->zero_samples+1U) : 0;
    t->retry_at=HAL_GetTick()+200U;
    if (t->zero_samples>=2) t->state=t->fault==TJ_OK ? TJ_DONE :
        t->fault==TJ_USER_STOP ? TJ_CANCELLED : TJ_FAULT;
}
void Trajectory_Task(command_manager_t *m)
{
    if (!m || m->busy || !Trajectory_Active(&m->trajectory)) return;
    trajectory_t *t=&m->trajectory;uint32_t now=HAL_GetTick();
    m->busy=1;
    if (t->state!=TJ_STOPPING) {
        if (now-t->last_tick>t->max_gap) t->max_gap=now-t->last_tick;
        if (input_trip(m)) Trajectory_Cancel(m,TJ_LIMIT);
        else if ((int32_t)(now-t->lease_until)>=0) Trajectory_Cancel(m,TJ_HEARTBEAT);
        else if (now-t->last_tick>TJ_MAX_GAP) Trajectory_Cancel(m,TJ_DEADLINE);
    }
    if (t->state==TJ_STOPPING) { stopping(m);m->busy=0;return; }
    if ((int32_t)(now-t->next_tick)<0) { m->busy=0;return; }
    uint32_t gap=now-t->last_tick;
    if (gap>t->max_gap) t->max_gap=gap;
    uint8_t units=(uint8_t)(now-t->units_at>=1000U);
    uint32_t dt=gap ? gap : t->cycle_ms;
    int16_t rpm[2]={0,0};uint8_t settled=1;
    /* Sample both axes BEFORE advancing the shared path. No per-axis clocks. */
    for (unsigned i=0;i<2;++i) {
        int reason=read_pose(m,i,units);
        if (reason) { Trajectory_Cancel(m,(uint8_t)reason);break; }
    }
    uint32_t rate=1000U;
    if (t->state!=TJ_STOPPING && t->adaptive && t->state==TJ_RUNNING) {
        int64_t worst=0;
        for (unsigned i=0;i<2;++i) {
            int32_t target,ff;sample(t,t->elapsed,i,&target,&ff);
            int64_t error=magnitude((int64_t)target-t->actual[i]);
            if (error>worst) worst=error;
        }
        int64_t slow=t->maximum_error/4,hold=t->maximum_error*3/4;
        if (worst>slow) rate=worst>=hold ? 0U : (uint32_t)((hold-worst)*1000/(hold-slow));
        uint32_t advance=(uint32_t)((uint64_t)gap*rate/1000U);
        uint32_t end=t->points[t->count-1].time;
        if (advance>end-t->elapsed) advance=end-t->elapsed;
        /* Bound the next target for BOTH axes, even on the very first tick.
           An immobile axis cannot let the path run away up to a hard trip. */
        while (advance) {
            uint8_t fits=1U;
            for (unsigned i=0;i<2;++i) {
                int32_t target,ff;sample(t,t->elapsed+advance,i,&target,&ff);
                if (magnitude((int64_t)target-t->actual[i])>hold) fits=0U;
            }
            if (fits) break;
            advance/=2U;
        }
        rate=gap ? (uint32_t)((uint64_t)advance*1000U/gap) : 0U;
        t->elapsed+=advance;
    } else if (!t->adaptive) t->elapsed=HAL_GetTick()-t->started;
    if (t->adaptive && HAL_GetTick()-t->started > t->points[t->count-1].time*4U+30000U)
        Trajectory_Cancel(m,TJ_PROGRESS_TIMEOUT);
    for (unsigned i=0;i<2 && t->state!=TJ_STOPPING;++i) {
        int32_t target=0,feedforward=0;
        sample(t,t->elapsed,i,&target,&feedforward);
        int64_t error=(int64_t)target-t->actual[i];t->error[i]=(int32_t)error;
        if (magnitude(error)>t->maximum_error) { Trajectory_Cancel(m,TJ_TRACKING);break; }
        int64_t desired=(int64_t)feedforward*rate/1000+error*2;
        if (t->elapsed>=t->points[t->count-1].time) {
            if (t->state!=TJ_SETTLING) t->settling_at=HAL_GetTick();
            t->state=TJ_SETTLING;
            if (magnitude(error)<=t->tolerance) desired=0;
            else settled=0;
            if (m->motors[i]->status.speed_raw!=0) settled=0;
        } else settled=0;
        if (desired>t->maximum_speed) desired=t->maximum_speed;
        if (desired < -t->maximum_speed) desired=-t->maximum_speed;
        int64_t step=(int64_t)t->maximum_accel*dt/1000;
        if (desired>t->velocity[i]+step) desired=t->velocity[i]+step;
        if (desired<t->velocity[i]-step) desired=t->velocity[i]-step;
        /* Reserve a bus-delay distance and ideal braking distance. This is
           conservative software screening, NOT a certified stop guarantee. */
        int64_t reserve=magnitude(desired)*TJ_MAX_GAP/1000+
            desired*desired/(2*t->maximum_accel)+TJ_MARGIN;
        axis_guard_t *g=&m->guard[i];
        if (g->soft_on && ((desired>0 && (int64_t)g->maximum-t->actual[i]<=reserve) ||
            (desired<0 && (int64_t)t->actual[i]-g->minimum<=reserve))) {
            Trajectory_Cancel(m,TJ_RANGE);break;
        }
        t->velocity[i]=(int32_t)desired;
        int64_t numerator=desired*m->gear_ratio_num[i],denom=(int64_t)6000*m->gear_ratio_den[i];
        if (!denom) { Trajectory_Cancel(m,TJ_REFERENCE);break; }
        int64_t speed=(magnitude(numerator)+denom/2)/denom;
        if (numerator<0) speed=-speed;
        if (speed < -5000 || speed>5000) { Trajectory_Cancel(m,TJ_RANGE);break; }
        rpm[i]=(int16_t)(speed*(g->angle_sign<0 ? -1 : 1)); /* physical angle -> drive sign */
        if (t->state==TJ_SETTLING && rpm[i]!=0) settled=0;
    }
    if (units) t->units_at=HAL_GetTick();
    if (t->state!=TJ_STOPPING && HAL_GetTick()-now>TJ_MAX_GAP) Trajectory_Cancel(m,TJ_DEADLINE);
    if (t->state!=TJ_STOPPING && input_trip(m)) Trajectory_Cancel(m,TJ_LIMIT);
    if (t->state!=TJ_STOPPING) {
        for (unsigned i=0;i<2;++i) {
            if (input_trip(m)) { Trajectory_Cancel(m,TJ_LIMIT);break; }
            if (Command_Manager_UpdateVelocity(m,(uint8_t)i,rpm[i])!=MODBUS_OK) {
                Trajectory_Cancel(m,TJ_COMM);break;
            }
        }
    }
    if (t->state==TJ_SETTLING) {
        t->zero_samples=settled ? (uint8_t)(t->zero_samples+1U) : 0;
        if (t->zero_samples>=2) Trajectory_Cancel(m,TJ_OK);
        else if (HAL_GetTick()-t->settling_at>5000U) Trajectory_Cancel(m,TJ_SETTLE_TIMEOUT);
    }
    t->last_cycle=HAL_GetTick()-now;
    t->last_tick=now;t->next_tick=now+t->cycle_ms;
    if (t->state==TJ_STOPPING) stopping(m);
    m->busy=0;
}
static void reply(const pc_command_packet_t *c,trajectory_t *t,pc_status_t status)
{
    uint8_t data[40]={0};put16(data,t->session);data[2]=t->state;data[3]=t->fault;
    put16(data+4,t->count);put16(data+6,t->loaded);
    put32(data+8,t->elapsed);put32(data+12,t->loaded ? t->points[t->loaded-1].time : 0);
    put32(data+16,t->max_gap);put32(data+20,t->last_cycle);
    for (unsigned i=0;i<2;++i) { put32(data+24+i*4,(uint32_t)t->actual[i]);put32(data+32+i*4,(uint32_t)t->error[i]); }
    (void)PC_Comm_SendResponseData(c->device_id,c->command,status,c->parameter,data,sizeof(data));
}
static void axes_reply(command_manager_t *m,const pc_command_packet_t *c)
{
    uint8_t data[40]={0};trajectory_t *t=&m->trajectory;
    uint8_t cached=Trajectory_Active(t);
    for (unsigned i=0;i<2;++i) {
        axis_guard_t *g=&m->guard[i];uint16_t mode=3U,ppr=g->ppr;
        uint8_t valid=t->sample_valid[i];int32_t angle=0;
        uint8_t point_cached=(uint8_t)(!c->parameter && m->point_active[i] && m->point_sample_valid[i] &&
            HAL_GetTick()-m->point_sample_at[i]<=200U);
        if (point_cached) {
            valid=1U;ppr=m->point_ppr[i];t->sampled_at[i]=m->point_sample_at[i];
        } else if (!cached || c->parameter==0x4652U) {
            valid=(uint8_t)(p100s_read_status(m->motors[i])==MODBUS_OK);
            if (valid) { t->sampled_at[i]=HAL_GetTick();
                if ((m->point_active[i] || m->point_completed[i]) && m->point_sample_valid[i] &&
                    HAL_GetTick()-m->point_units_at[i]<1000U) { mode=3U;ppr=m->point_ppr[i]; }
                else valid=(uint8_t)(p100s_read_parameter(m->motors[i],P100S_GROUP_P3,32U,&mode)==MODBUS_OK &&
                    p100s_read_parameter(m->motors[i],P100S_GROUP_PA,11U,&ppr)==MODBUS_OK); }
            t->sample_valid[i]=valid;
        }
        uint8_t referenced=(uint8_t)(valid && AxisGuard_Angle(g,m->motors[i]->status.position,
            mode,ppr,m->gear_ratio_num[i],m->gear_ratio_den[i],&angle));
        /* Do not silently reuse an old origin after a drive disappears or its
           feedback units/counter become invalid. Require explicit recapture. */
        if (!referenced) g->zero_valid=0U;
        unsigned n=i*20U;
        put32(data+n,(uint32_t)angle);put32(data+n+4,(uint32_t)m->motors[i]->status.position);
        put16(data+n+8,(uint16_t)m->motors[i]->status.speed_raw);
        put16(data+n+10,(uint16_t)(referenced | (g->soft_on<<1) | ((g->hardware_on || g->dual_enabled)<<2) | (g->stop_pending<<4) | Command_Manager_PointFlags(m,(uint8_t)i)));
        /* Age is measured from completion of the drive's feedback reply, not
           an assertion that two sequential encoder reads were simultaneous. */
        put32(data+n+12,valid ? HAL_GetTick()-t->sampled_at[i] : UINT32_MAX);
        put16(data+n+16,m->motors[i]->status.alarm_code);
        data[n+18]=g->fault;data[n+19]=valid ? (uint8_t)(!c->parameter && (cached || point_cached)) : 2U;
    }
    for (unsigned i=0;i<2;++i)
        put32(data+i*20U+12U,data[i*20U+19U]!=2U ? HAL_GetTick()-t->sampled_at[i] : UINT32_MAX);
    (void)PC_Comm_SendResponseData(c->device_id,c->command,PC_STATUS_SUCCESS,c->parameter,data,sizeof(data));
}
void Trajectory_Command(command_manager_t *m,const pc_command_packet_t *c)
{
    trajectory_t *t=&m->trajectory;pc_status_t status=PC_STATUS_SUCCESS;
    if (c->device_id!=1U) { reply(c,t,PC_STATUS_ILLEGAL_PARAM);return; }
    if (c->command==PC_CMD_READ_AXES) { axes_reply(m,c);return; }
    if (c->command==PC_CMD_TRAJECTORY_STATUS) { reply(c,t,status);return; }
    if (c->command==PC_CMD_TRAJECTORY_CLEAR) {
        if (!t->session || c->parameter!=t->session) status=PC_STATUS_ILLEGAL_PARAM;
        else if (t->state!=TJ_FAULT && t->state!=TJ_CANCELLED && t->state!=TJ_DONE) status=PC_STATUS_BUSY;
        else {
            /* Read-only verification: clearing a fault must never initiate
               motion, clear a reference, or reset a drive alarm. Keep the
               fault snapshot intact on every rejected acknowledgement. */
            for (unsigned pass=0;pass<2 && status==PC_STATUS_SUCCESS;++pass) {
                for (unsigned i=0;i<2;++i) {
                    axis_guard_t *g=&m->guard[i];
                    int32_t saved=t->actual[i];int reason=read_pose(m,i,1U);t->actual[i]=saved;
                    if (reason) { status=reason==TJ_COMM ? PC_STATUS_SERVO_NO_RESPONSE :
                        reason==TJ_RANGE ? PC_STATUS_RANGE : PC_STATUS_NOT_REFERENCED;break; }
                    if (m->continuous_active[i] || m->continuous_stop_pending[i] ||
                        (g->phase>=HOME_SEEK && g->phase<=HOME_FINAL_STOP) || m->motors[i]->status.speed_raw) {
                        status=PC_STATUS_BUSY;break;
                    }
                    if (g->hardware_on) {
                        for (unsigned r=0;r<(g->home_at_end ? 3U : 2U);++r)
                            if (AxisGuard_Active(&m->guard_inputs,g,channel(g,r))) status=PC_STATUS_LIMIT_ACTIVE;
                    }
                }
                if (!pass && status==PC_STATUS_SUCCESS) HAL_Delay(20U);
            }
            /* Invalidate the entire old session/buffer. START and late
               heartbeats cannot resurrect the acknowledged task. */
            if (status==PC_STATUS_SUCCESS) memset(t,0,sizeof(*t));
        }
        reply(c,t,status);return;
    }
    if (c->command==PC_CMD_TRAJECTORY_STOP && (!c->parameter || c->parameter==t->session)) {
        if (Trajectory_Active(t)) Trajectory_Cancel(m,TJ_USER_STOP);
        else {
            uint8_t reason=t->state==TJ_FAULT ? t->fault : TJ_USER_STOP;
            t->state=TJ_STOPPING;t->fault=reason;t->zero_samples=0;t->retry_at=HAL_GetTick();
        }
        /* This explicit group STOP also stops legacy motion outside a trajectory. */
        for (unsigned i=0;i<2;++i) {
            GuardController_Cancel(m,(uint8_t)i);
            m->continuous_active[i]=m->continuous_stop_pending[i]=0;m->target_speed[i]=0;
        }
        reply(c,t,status);return;
    }
    if (c->command==PC_CMD_TRAJECTORY_BEGIN) {
        uint16_t count=u16(c->data),encoded_cycle=u16(c->data+2);
        uint8_t adaptive=(uint8_t)((encoded_cycle & 0x8000U)!=0U);
        uint16_t cycle=encoded_cycle & 0x7FFFU;
        int32_t speed=(int32_t)u32(c->data+4),accel=(int32_t)u32(c->data+8);
        int32_t error=(int32_t)u32(c->data+12),tolerance=(int32_t)u32(c->data+16);
        if (Trajectory_Active(t) || m->continuous_active[0] || m->continuous_active[1]) status=PC_STATUS_BUSY;
        else if (!c->parameter || c->parameter==t->session || count<2 || count>TRAJECTORY_CAPACITY || cycle<50 || cycle>500 ||
            speed<1 || speed>60000 || accel<1 || accel>120000 || error<500 || error>10000 ||
            tolerance<50 || tolerance>1000 || tolerance>=error) status=PC_STATUS_ILLEGAL_PARAM;
        else { memset(t,0,sizeof(*t));t->session=c->parameter;t->count=count;t->cycle_ms=cycle;
            t->maximum_speed=speed;t->maximum_accel=accel;t->maximum_error=error;t->tolerance=tolerance;
            t->adaptive=adaptive;t->state=TJ_UPLOADING; }
        reply(c,t,status);return;
    }
    if (!t->session || c->parameter!=t->session) { reply(c,t,PC_STATUS_ILLEGAL_PARAM);return; }
    if (c->command==PC_CMD_TRAJECTORY_KEEPALIVE) {
        /* Terminal packets acknowledge state but can never restart/extend it. */
        if (t->state==TJ_RUNNING || t->state==TJ_SETTLING) t->lease_until=HAL_GetTick()+TRAJECTORY_LEASE_MS;
    } else if (c->command==PC_CMD_TRAJECTORY_POINT) {
        uint16_t n=u16(c->data);uint32_t time=u32(c->data+2);
        if (t->state!=TJ_UPLOADING) status=PC_STATUS_BUSY;
        else if (n!=t->loaded || n>=t->count || time>600000U || (!n && time) ||
            (n && (time<=t->points[n-1].time || time-t->points[n-1].time<50U))) status=PC_STATUS_ILLEGAL_PARAM;
        else { t->points[n].time=time;t->points[n].angle[0]=(int32_t)u32(c->data+6);
            t->points[n].angle[1]=(int32_t)u32(c->data+10);++t->loaded; }
    } else if (c->command==PC_CMD_TRAJECTORY_SEAL) {
        if (t->state!=TJ_UPLOADING) status=PC_STATUS_BUSY;
        else if (!validate(m)) status=PC_STATUS_RANGE;
        else t->state=TJ_READY;
    } else if (c->command==PC_CMD_TRAJECTORY_START) {
        if (t->state!=TJ_READY || !validate(m)) status=PC_STATUS_BUSY;
        else {
            t->state=TJ_RUNNING; /* Prevent a duplicate start even on partial setup. */
            t->started=t->last_tick=HAL_GetTick();t->lease_until=HAL_GetTick()+TRAJECTORY_LEASE_MS;
            for (unsigned i=0;i<2;++i) {
                axis_guard_t *g=&m->guard[i];
                if (m->continuous_active[i] || m->continuous_stop_pending[i] ||
                    (g->phase>=HOME_SEEK && g->phase<=HOME_FINAL_STOP)) { Trajectory_Cancel(m,TJ_REFERENCE);status=PC_STATUS_BUSY;break; }
                for (unsigned r=0;r<3;++r) t->trips[i][r]=AxisGuard_Trips(&m->guard_inputs,g,channel(g,r));
                int reason=read_pose(m,i,1);
                if (reason || m->motors[i]->status.speed_raw ||
                    magnitude((int64_t)t->actual[i]-t->points[0].angle[i])>t->tolerance) {
                    Trajectory_Cancel(m,reason ? (uint8_t)reason : TJ_START_POSITION);status=PC_STATUS_NOT_REFERENCED;break;
                }
            }
            if (t->state!=TJ_STOPPING && input_trip(m)) { Trajectory_Cancel(m,TJ_LIMIT);status=PC_STATUS_LIMIT_ACTIVE; }
            if (t->state!=TJ_STOPPING) for (unsigned i=0;i<2;++i) {
                modbus_result_t r=Command_Manager_PrepareVelocity(m,(uint8_t)i);
                if (r!=MODBUS_OK) {
                    Trajectory_Cancel(m,r==MODBUS_ERROR_ARGUMENT ? TJ_REFERENCE : TJ_COMM);
                    status=r==MODBUS_ERROR_ARGUMENT ? PC_STATUS_ILLEGAL_PARAM : PC_STATUS_SERVO_NO_RESPONSE;break;
                }
            }
            if (t->state!=TJ_STOPPING) {
                t->started=t->last_tick=t->units_at=HAL_GetTick();t->next_tick=HAL_GetTick();
                t->lease_until=HAL_GetTick()+TRAJECTORY_LEASE_MS;t->zero_samples=0;
                m->guard[0].phase=m->guard[1].phase=HOME_IDLE;
            } else stopping(m);
        }
    } else status=PC_STATUS_UNKNOWN_CMD;
    reply(c,t,status);
}
