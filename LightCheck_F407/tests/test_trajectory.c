#define main legacy_limits_main
#include "test_limits.c"
#undef main

static void w16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)(v>>8);p[1]=(uint8_t)v; }
static void w32(uint8_t *p,int32_t v) { uint32_t u=(uint32_t)v;p[0]=(uint8_t)(u>>24);p[1]=(uint8_t)(u>>16);p[2]=(uint8_t)(u>>8);p[3]=(uint8_t)u; }
static void tj(uint8_t op,uint16_t session,const uint8_t *data,uint8_t length)
{
    pc_command_packet_t c={0};c.device_id=1;c.command=op;c.parameter=session;c.valid_payload=1;c.data_length=length;
    if (length) memcpy(c.data,data,length);
    Command_Process(&manager,&c);
}
static void setup_tj(void)
{
    memset(motors,0,sizeof(motors));memset(forced,0,sizeof(forced));memset(current_mode,0,sizeof(current_mode));
    memset(forced_speed,0,sizeof(forced_speed));memset(last_speed,0,sizeof(last_speed));
    fail_stop=fail_reads=mapped_speed_input=inject_trip=speed_on_delay=0;now=1000;
    motors[0].slave_id=1;motors[1].slave_id=2;motors[0].bus=motors[1].bus=&bus;
    Command_Manager_Init(&manager,&motors[0],&motors[1],NULL);
    for (unsigned i=0;i<2;++i) {
        feedback_mode[i]=3;manager.guard[i].zero_valid=manager.guard[i].soft_on=1;manager.guard[i].ppr=10000;
    }
    for (unsigned n=0;n<30;++n) { ++now;Command_Manager_SampleInputs(&manager,0); }
}
static void upload(void)
{
    uint8_t header[20]={0};w16(header,3);w16(header+2,250);w32(header+4,10000);w32(header+8,30000);
    w32(header+12,5000);w32(header+16,500);
    tj(PC_CMD_TRAJECTORY_BEGIN,77,header,20);assert(reply==PC_STATUS_SUCCESS);
    for (unsigned n=0;n<3;++n) {
        uint8_t p[14];w16(p,(uint16_t)n);w32(p+2,(int32_t)n*2500);
        w32(p+6,n==1 ? 26000 : 25000);w32(p+10,n==1 ? 1000 : 0);
        tj(PC_CMD_TRAJECTORY_POINT,77,p,14);assert(reply==PC_STATUS_SUCCESS);
    }
    tj(PC_CMD_TRAJECTORY_SEAL,77,NULL,0);assert(reply==PC_STATUS_SUCCESS && manager.trajectory.state==TJ_READY);
}
static void test_adaptive_and_positions(void)
{
    setup_tj();upload();
    /* Wire flag opts in, legacy BEGIN remains fixed-time. */
    uint8_t header[20]={0};w16(header,3);w16(header+2,0x8000U|100U);
    w32(header+4,10000);w32(header+8,30000);w32(header+12,750);w32(header+16,100);
    tj(PC_CMD_TRAJECTORY_BEGIN,88,header,20);
    assert(reply==PC_STATUS_SUCCESS && manager.trajectory.adaptive && manager.trajectory.cycle_ms==100U);
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    trajectory_t *t=&manager.trajectory;t->adaptive=1U;t->maximum_error=750;t->tolerance=100;t->cycle_ms=100;
    for (unsigned n=0;n<60;++n) {
        now+=100;tj(PC_CMD_TRAJECTORY_KEEPALIVE,77,NULL,0);Command_Manager_Task(&manager);
    }
    assert(t->state==TJ_RUNNING && t->fault==TJ_OK);
    assert(t->elapsed>0U && t->elapsed<2500U);
    assert(t->error[0]==t->error[1] && t->error[0]<=563);
    assert(last_speed[0]>0 && last_speed[1]>0); /* Hold path, still correct position. */
    uint32_t paused=t->elapsed;
    for (unsigned n=0;n<100 && Trajectory_Active(t);++n) {
        /* Both axes catch up to the shared phase, then traverse the remaining path. */
        uint32_t phase=t->elapsed;
        int32_t delta=phase<=2500 ? (int32_t)phase*1000/2500 : (5000-(int32_t)phase)*1000/2500;
        motors[0].status.position=motors[1].status.position=delta*500/360;
        now+=100;tj(PC_CMD_TRAJECTORY_KEEPALIVE,77,NULL,0);Command_Manager_Task(&manager);
    }
    assert(t->elapsed>paused && t->state==TJ_DONE && t->fault==TJ_OK);
    /* Terminal trajectory snapshot stays frozen; independent position reads are fresh. */
    t->fault=TJ_TRACKING;t->state=TJ_FAULT;
    int32_t frozen=t->actual[0];motors[0].status.position=1000;
    tj(PC_CMD_READ_AXES,0,NULL,0);
    assert(reply==PC_STATUS_SUCCESS && response_size==40U && response[19]==0U);
    assert((response[11]&1U) && response[3]!=0U);
    assert(t->fault==TJ_TRACKING && t->actual[0]==frozen);
    manager.guard[1].zero_valid=0;
    tj(PC_CMD_READ_AXES,0,NULL,0);assert(!(response[31]&1U));
    fail_reads=1;tj(PC_CMD_READ_AXES,0,NULL,0);
    assert(response[19]==2U && !(response[11]&1U) && response[12]==0xFFU);
    fail_reads=0;
    tj(PC_CMD_READ_AXES,0,NULL,0);
    assert(!manager.guard[0].zero_valid && !(response[11]&1U)); /* No silent revalidation after a lost drive. */
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    Command_Manager_Task(&manager);t=&manager.trajectory;t->next_tick=now+500U;
    fail_reads=1;tj(PC_CMD_READ_AXES,0,NULL,0);
    assert(response[19]==1U && t->fault==TJ_OK); /* Cached query adds no RS485 reads. */
    fail_reads=0;
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    t=&manager.trajectory;t->adaptive=1;t->maximum_error=750;t->tolerance=100;
    for (unsigned n=0;n<205 && t->state==TJ_RUNNING;++n) {
        now+=250;tj(PC_CMD_TRAJECTORY_KEEPALIVE,77,NULL,0);Command_Manager_Task(&manager);
    }
    assert(t->fault==TJ_PROGRESS_TIMEOUT); /* A stalled plant never runs forever. */
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    t=&manager.trajectory;t->adaptive=1;t->maximum_error=750;t->tolerance=100;t->cycle_ms=100;
    for (unsigned n=0;n<70;++n) {
        int32_t delta=(int32_t)t->elapsed*1000/2500;
        motors[0].status.position=delta*500/360; /* Only axis 1 follows. */
        now+=100;tj(PC_CMD_TRAJECTORY_KEEPALIVE,77,NULL,0);Command_Manager_Task(&manager);
    }
    assert(t->state==TJ_RUNNING && t->elapsed<2500 && t->error[1]>t->error[0]);
    assert(t->error[1]<=563); /* Axis 2's lag slows the shared path for both. */
    setup_tj();upload();manager.guard[1].soft_on=0;
    for (unsigned n=0;n<manager.trajectory.count;++n) manager.trajectory.points[n].angle[1]+=100000;
    motors[1].status.position=138889; /* About +100deg, beyond the unused +/-90 draft. */
    tj(PC_CMD_TRAJECTORY_START,77,NULL,0);assert(reply==PC_STATUS_SUCCESS);
    now+=250;Command_Manager_Task(&manager);
    assert(manager.trajectory.state==TJ_RUNNING && manager.trajectory.actual[1]>90000);
    manager.guard[1].zero_valid=0;now+=250;Command_Manager_Task(&manager);
    assert(manager.trajectory.fault==TJ_REFERENCE); /* No bounds does not mean no reference. */
    setup_tj();upload();manager.guard[0].soft_on=0;
    tj(PC_CMD_TRAJECTORY_START,77,NULL,0);assert(reply!=PC_STATUS_SUCCESS);
}
static void test_clear_fault(void)
{
    setup_tj();upload();
    tj(PC_CMD_TRAJECTORY_CLEAR,77,NULL,0);assert(reply==PC_STATUS_BUSY); /* Preload is not a fault. */
    tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    tj(PC_CMD_TRAJECTORY_CLEAR,77,NULL,0);assert(reply==PC_STATUS_BUSY);
    trajectory_t *t=&manager.trajectory;t->state=TJ_FAULT;t->fault=TJ_TRACKING;
    t->error[0]=1031;int32_t saved=t->actual[0];
    tj(PC_CMD_TRAJECTORY_CLEAR,78,NULL,0);
    assert(reply==PC_STATUS_ILLEGAL_PARAM && t->fault==TJ_TRACKING);
    motors[1].status.speed_raw=1;
    tj(PC_CMD_TRAJECTORY_CLEAR,77,NULL,0);
    assert(reply==PC_STATUS_BUSY && t->fault==TJ_TRACKING && t->error[0]==1031 && t->actual[0]==saved);
    motors[1].status.speed_raw=0;speed_on_delay=1;
    tj(PC_CMD_TRAJECTORY_CLEAR,77,NULL,0);
    assert(reply==PC_STATUS_BUSY && t->fault==TJ_TRACKING); /* Second zero observation required. */
    motors[1].status.speed_raw=0;manager.guard[1].zero_valid=0;
    tj(PC_CMD_TRAJECTORY_CLEAR,77,NULL,0);assert(reply!=PC_STATUS_SUCCESS && t->fault==TJ_TRACKING);
    manager.guard[1].zero_valid=1;motors[0].status.alarm_code=3;
    tj(PC_CMD_TRAJECTORY_CLEAR,77,NULL,0);assert(reply!=PC_STATUS_SUCCESS && t->fault==TJ_TRACKING);
    motors[0].status.alarm_code=0;
    /* Guard polling may have latched the drive alarm; a trajectory ACK cannot erase it. */
    manager.guard[0].fault=0;manager.guard[0].stop_pending=0;manager.guard[0].zero_valid=1;
    unsigned old_holds=holds[0]+holds[1],old_triggers=triggers[0]+triggers[1];
    int32_t origin=manager.guard[0].origin;
    tj(PC_CMD_TRAJECTORY_CLEAR,77,NULL,0);
    assert(reply==PC_STATUS_SUCCESS && t->state==TJ_IDLE && !t->fault && !t->session && !t->count);
    assert(holds[0]+holds[1]==old_holds && triggers[0]+triggers[1]==old_triggers);
    assert(manager.guard[0].zero_valid && manager.guard[0].origin==origin);
    tj(PC_CMD_TRAJECTORY_KEEPALIVE,77,NULL,0);assert(reply==PC_STATUS_ILLEGAL_PARAM && t->state==TJ_IDLE);
    tj(PC_CMD_TRAJECTORY_START,77,NULL,0);assert(reply==PC_STATUS_ILLEGAL_PARAM && t->state==TJ_IDLE);
}
int main(void)
{
    setup_tj();upload();assert(last_speed[0]==0 && last_speed[1]==0);
    tj(PC_CMD_TRAJECTORY_START,78,NULL,0);assert(reply==PC_STATUS_ILLEGAL_PARAM);
    tj(PC_CMD_TRAJECTORY_START,77,NULL,0);assert(reply==PC_STATUS_SUCCESS);
    unsigned h0=holds[0],h1=holds[1];
    for (unsigned n=0;n<25;++n) {
        now+=250;
        tj(PC_CMD_TRAJECTORY_KEEPALIVE,77,NULL,0);
        assert(reply==PC_STATUS_SUCCESS);
        Command_Manager_Task(&manager);
    }
    assert(manager.trajectory.state==TJ_DONE);
    assert(holds[0]==h0 && holds[1]==h1); /* Speed following never uses position HOLD/CTRG. */
    tj(PC_CMD_TRAJECTORY_START,77,NULL,0);assert(reply==PC_STATUS_BUSY);
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    tj(PC_CMD_TRAJECTORY_STOP,77,NULL,0);assert(manager.trajectory.state==TJ_STOPPING);
    now+=250;Command_Manager_Task(&manager);now+=250;Command_Manager_Task(&manager);
    assert(manager.trajectory.state==TJ_CANCELLED && !last_speed[0] && !last_speed[1]);
    tj(PC_CMD_TRAJECTORY_KEEPALIVE,77,NULL,0);assert(manager.trajectory.state==TJ_CANCELLED);
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    now+=1001;Command_Manager_Task(&manager);assert(manager.trajectory.fault==TJ_DEADLINE);
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    now+=3001;Command_Manager_Task(&manager);assert(manager.trajectory.fault==TJ_HEARTBEAT);
    setup_tj();upload();motors[0].status.position=3000;
    tj(PC_CMD_TRAJECTORY_START,77,NULL,0);assert(reply!=PC_STATUS_SUCCESS && manager.trajectory.fault==TJ_START_POSITION);
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    fail_reads=1;now+=250;Command_Manager_Task(&manager);
    assert(manager.trajectory.fault==TJ_COMM && !manager.guard[0].zero_valid && !manager.guard[1].zero_valid);
    fail_reads=0;
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    manager.guard[0].hardware_on=1;manager.guard[0].negative=0;
    ++now;Command_Manager_SampleInputs(&manager,1);
    for (unsigned n=0;n<31;++n) { ++now;Command_Manager_SampleInputs(&manager,0); }
    Command_Manager_Task(&manager);assert(manager.trajectory.fault==TJ_LIMIT); /* A short edge is not lost. */
    setup_tj();upload();manager.guard[1].zero_valid=0;
    tj(PC_CMD_TRAJECTORY_START,77,NULL,0);assert(reply!=PC_STATUS_SUCCESS);
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    request(1,PC_CMD_MOVE_OUTPUT_RELATIVE);assert(reply==PC_STATUS_BUSY);
    request(2,PC_CMD_MOTOR_STOP);assert(reply==PC_STATUS_SUCCESS && manager.trajectory.state==TJ_STOPPING);
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    feedback_mode[1]=0;now+=1000;Command_Manager_Task(&manager);
    assert(manager.trajectory.fault==TJ_REFERENCE && !manager.guard[0].zero_valid && !manager.guard[1].zero_valid);
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    motors[1].status.position=20000;now+=250;Command_Manager_Task(&manager);
    assert(manager.trajectory.fault==TJ_TRACKING);
    setup_tj();upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    motors[0].status.position=16000;now+=250;Command_Manager_Task(&manager);
    assert(manager.trajectory.fault==TJ_RANGE);
    setup_tj();upload();manager.guard[0].maximum=26000;
    tj(PC_CMD_TRAJECTORY_START,77,NULL,0);assert(reply!=PC_STATUS_SUCCESS); /* Revalidate edited bounds. */
    setup_tj();
    h0=holds[0];h1=holds[1];fail_stop=1;
    tj(PC_CMD_TRAJECTORY_STOP,0,NULL,0);Command_Manager_Task(&manager);
    assert(holds[0]>h0 && holds[1]>h1 && manager.trajectory.state==TJ_STOPPING);
    fail_stop=0;now+=250;Command_Manager_Task(&manager);now+=250;Command_Manager_Task(&manager);
    assert(manager.trajectory.state==TJ_CANCELLED); /* Group STOP also handles non-trajectory mode. */
    setup_tj();now=UINT32_MAX-500U;upload();tj(PC_CMD_TRAJECTORY_START,77,NULL,0);
    for (unsigned n=0;n<5;++n) { now+=250;tj(PC_CMD_TRAJECTORY_KEEPALIVE,77,NULL,0); }
    assert(manager.trajectory.state==TJ_RUNNING && manager.trajectory.fault==TJ_OK);
    test_adaptive_and_positions();
    test_clear_fault();
    puts("buffered trajectory, group stop, input latch, reference, deadline and lease tests passed");
    return 0;
}
