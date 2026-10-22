#define main legacy_limits_test_main
#include "test_limits.c"
#undef main

static void p32(uint8_t *p,int32_t v) {uint32_t n=(uint32_t)v;p[0]=n>>24;p[1]=n>>16;p[2]=n>>8;p[3]=n;}
static void send(uint8_t op,uint16_t key,const uint8_t *d,unsigned n)
{
    pc_command_packet_t c={0};c.device_id=1;c.command=op;c.parameter=key;c.valid_payload=1;c.data_length=n;
    if(n)memcpy(c.data,d,n);
    Command_Process(&manager,&c);
}
static void step(uint8_t raw)
{
    for(unsigned n=0;n<100;++n){++now;Command_Manager_SampleInputs(&manager,raw);}
    Command_Manager_Task(&manager);
}
static void setup_dual(int sign,uint8_t valid,uint8_t high)
{
    memset(motors,0,sizeof(motors));memset(forced,0,sizeof(forced));memset(forced_speed,0,sizeof(forced_speed));
    memset(speed_parameters,0,sizeof(speed_parameters));fail_reads=fail_stop=inject_trip=mapped_speed_input=0;
    motors[0].slave_id=1;motors[1].slave_id=2;motors[0].bus=motors[1].bus=&bus;
    feedback_mode[0]=feedback_mode[1]=3;now=1000;
    Command_Manager_Init(&manager,&motors[0],&motors[1],NULL);
    step(high ? 0 : 5);
    uint8_t d[14]={1,2,0,high,(uint8_t)sign,valid};p32(d+6,2000);p32(d+10,28000);
    send(PC_CMD_CONFIG_DUAL_LIMIT,0x444C,d,14);assert(reply==PC_STATUS_SUCCESS);
}
static void search(int direction)
{
    uint8_t d[13]={(uint8_t)direction};p32(d+1,300);p32(d+5,40000);p32(d+9,120000);
    send(PC_CMD_HOME_DUAL_LIMIT,0x4448,d,13);
}
static void measured_search(void)
{
    uint8_t d[13]={255};p32(d+1,500);p32(d+5,45000);p32(d+9,120000);
    send(PC_CMD_HOME_DUAL_LIMIT,0x4449,d,13);
}
static void measured_setup(void)
{
    setup_dual(-1,3,1);
    uint8_t d[14]={1,2,0,1,255,3};p32(d+6,-29000);p32(d+10,35000);
    send(PC_CMD_CONFIG_DUAL_LIMIT,0x444C,d,14);assert(reply==PC_STATUS_SUCCESS);
    assert(manager.guard[0].minimum==-29000 && manager.guard[0].maximum==35000);
    assert(!manager.guard[0].zero_valid && !manager.continuous_active[0]);
}
int main(void)
{
    setup_dual(1,3,1);search(-1);assert(reply==PC_STATUS_SUCCESS && last_speed[0]<0);
    motors[0].status.position=-1000;step(4);
    send(PC_CMD_ROTATE_KEEPALIVE,PC_ROTATE_KEEPALIVE_KEY,NULL,0);assert(reply==PC_STATUS_SUCCESS);
    step(4);step(4);
    axis_guard_t *g=&manager.guard[0];
    assert(g->phase==HOME_DONE && g->zero_valid && g->dual_last_hit==1 && g->reference_angle==2000);
    int32_t angle=0;assert(AxisGuard_Angle(g,-1000,3,10000,50,1,&angle)&&angle==2000);
    assert(!manager.continuous_active[0] && !g->stop_pending);
    uint8_t move[6]={0,10};p32(move+2,-1000);send(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);
    assert(reply==PC_STATUS_LIMIT_ACTIVE);
    p32(move+2,1000);send(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);assert(reply==PC_STATUS_SUCCESS);

    setup_dual(-1,3,1);search(1);assert(reply==PC_STATUS_SUCCESS && last_speed[0]>0);
    motors[0].status.position=1000;step(1);step(1);step(1);
    assert(g->phase==HOME_DONE && g->dual_last_hit==2 && g->reference_angle==28000);
    assert(AxisGuard_Angle(g,2000,3,10000,50,1,&angle)&&angle==27280);
    p32(move+2,1000);send(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);assert(reply==PC_STATUS_LIMIT_ACTIVE);
    /* Away from raised sensor: physical negative translates to raw positive. */
    p32(move+2,-1000);send(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);assert(reply==PC_STATUS_SUCCESS);
    assert(written_position_pulses[0]>0);
    send(PC_CMD_SET_ZERO,0x5A52,NULL,0);assert(reply==PC_STATUS_BUSY);

    setup_dual(1,0,1);search(1);assert(reply==PC_STATUS_NOT_REFERENCED);
    step(4);step(4);step(4);assert(!g->zero_valid && g->fault==GUARD_SENSOR);
    setup_dual(1,3,1);search(1);step(5);assert(!g->zero_valid && g->phase==HOME_FAULT);
    setup_dual(1,3,1);search(1);motors[0].status.position=-1000;step(0);
    assert(g->phase==HOME_FAULT && g->fault==GUARD_FEEDBACK);
    setup_dual(1,3,1);search(1);now+=5001;step(0);assert(g->phase==HOME_FAULT);
    setup_dual(1,3,1);search(1);manager.continuous_until[0]=now+10000;now+=3100;step(0);
    assert(g->phase==HOME_FAULT && g->fault==GUARD_FEEDBACK);
    setup_dual(1,3,1);search(1);motors[0].status.position=100000;step(0);
    assert(g->phase==HOME_FAULT && g->fault==GUARD_RANGE);
    setup_dual(1,3,1);search(1);fail_stop=1;step(4);step(4);
    assert(g->stop_pending && !g->zero_valid);
    fail_stop=0;step(4);step(4);assert(g->zero_valid && !g->stop_pending && !manager.continuous_active[0]);
    setup_dual(1,3,1);search(1);send(PC_CMD_MOTOR_STOP,0,NULL,0);
    assert(!g->dual_home && g->phase==HOME_FAULT && !g->zero_valid);
    setup_dual(1,3,1);search(1);step(4);step(0);step(0);assert(!g->zero_valid);
    setup_dual(1,3,1);search(1);step(4);feedback_mode[0]=0;step(4);step(4);
    assert(!g->zero_valid && g->fault==GUARD_FEEDBACK);
    setup_dual(1,3,0);search(-1);motors[0].status.position=-100;step(1);step(1);step(1);
    assert(g->zero_valid && g->dual_last_hit==1);
    /* Ordinary motion/stationary contact establishes either measured pose too. */
    setup_dual(1,3,1);motors[0].status.position=900;step(1);step(1);step(1);
    assert(g->zero_valid && g->reference_angle==28000 && g->dual_last_hit==2);
    send(PC_CMD_READ_DUAL_LIMIT,0,NULL,0);assert(reply==PC_STATUS_SUCCESS && response_size==16 && response[14]==2);
    uint8_t bad[14]={1,2,2,1,1,3};send(PC_CMD_CONFIG_DUAL_LIMIT,0x444C,bad,14);
    assert(reply==PC_STATUS_ILLEGAL_PARAM);
    /* Transport loss also stops ordinary position commands (no active lease),
       retries failed STOPs, and never restarts an interrupted homing task. */
    setup_dual(1,3,1);manager.trajectory.state=TJ_READY;manager.trajectory.session=77;
    Command_Manager_TransportLost(&manager);assert(!manager.trajectory.session);
    assert(manager.continuous_stop_pending[0] && manager.continuous_stop_pending[1]);
    current_mode[0]=1U;speed_parameters[0][2]=10U; /* Stop must clear a live setpoint. */
    fail_stop=1;step(0);assert(g->stop_pending && manager.continuous_stop_pending[0]);
    fail_stop=0;step(0);step(0);step(0);
    assert(!g->stop_pending && !manager.continuous_stop_pending[0] && !manager.continuous_stop_pending[1]);
    setup_dual(1,3,1);search(1);Command_Manager_TransportLost(&manager);
    assert(g->phase==HOME_FAULT && !g->dual_home && !g->zero_valid && !manager.continuous_active[0]);
    /* Measured fixture: raw negative raises the head; DI0 establishes +35deg. */
    measured_setup();measured_search();assert(reply==PC_STATUS_SUCCESS && last_speed[0]<0);
    motors[0].status.position=-51400;step(1);step(1);step(1);
    assert(g->phase==HOME_DONE && g->zero_valid && g->reference_angle==35000 && g->dual_last_hit==2);
    send(PC_CMD_MOTOR_STOP,0,NULL,0);
    assert(g->zero_valid && g->reference_angle==35000);
    assert(AxisGuard_Angle(g,-51400,3,10000,50,1,&angle) && angle==35000);
    p32(move+2,1000);send(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);assert(reply==PC_STATUS_LIMIT_ACTIVE);
    p32(move+2,-35000);send(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);
    assert(reply==PC_STATUS_SUCCESS && written_position_pulses[0]>0); /* return toward zero */
    measured_setup();measured_search();motors[0].status.position=-100;step(4);step(4);step(4);
    assert(g->phase==HOME_FAULT && g->fault==GUARD_SENSOR && !g->zero_valid);
    measured_setup();search(1);motors[0].status.position=1000;step(4);step(4);step(4);
    assert(g->phase==HOME_DONE && g->zero_valid && g->reference_angle==-29000);
    setup_dual(1,3,1);measured_search();assert(reply==PC_STATUS_ILLEGAL_PARAM);
    /* Actual optocoupler polarity: idle DI0/DI2=1, triggered input=0. */
    setup_dual(-1,3,0);
    uint8_t measured_low[14]={1,2,0,0,255,3};p32(measured_low+6,-29000);p32(measured_low+10,35000);
    send(PC_CMD_CONFIG_DUAL_LIMIT,0x444C,measured_low,14);assert(reply==PC_STATUS_SUCCESS);
    measured_search();assert(reply==PC_STATUS_SUCCESS && last_speed[0]<0);
    motors[0].status.position=-1000;step(4);step(4);step(4); /* DI0=0, DI2=1 */
    assert(g->phase==HOME_DONE && g->zero_valid && g->reference_angle==35000 && g->dual_last_hit==2);
    setup_dual(-1,3,0);send(PC_CMD_CONFIG_DUAL_LIMIT,0x444C,measured_low,14);
    measured_search();motors[0].status.position=-100;step(1);step(1);step(1); /* unexpected DI2 */
    assert(g->phase==HOME_FAULT && !g->zero_valid && g->fault==GUARD_SENSOR);
    measured_setup();measured_search();motors[0].status.position=-51400;step(1);step(1);step(1);
    /* Real log: stopped count drifts from captured origin by -10 counts,
       physical angle +35.007deg. Do not turn HOME_DONE into a stop loop. */
    motors[0].status.position=g->origin-10;step(1);step(1);step(1);
    assert(!g->stop_pending && g->fault==GUARD_OK && g->zero_valid);
    assert(AxisGuard_Angle(g,motors[0].status.position,3,10000,50,1,&angle) && angle==35007);
    assert(AxisGuard_TargetAllowed(g,35007,-35007));
    assert(!AxisGuard_TargetAllowed(g,35007,1));
    assert(!AxisGuard_TargetAllowed(g,35051,-35051));
    assert(AxisGuard_TargetAllowed(g,-29007,29007));
    assert(!AxisGuard_TargetAllowed(g,-29007,-1));
    motors[0].status.position=g->origin-100;step(1);
    assert(g->fault==GUARD_RANGE); /* 0.072deg is beyond feedback tolerance. */
    for(int speed=1200;speed<=9000;speed+=7800) {
        measured_setup();uint8_t fast[13]={255};p32(fast+1,speed);p32(fast+5,45000);p32(fast+9,120000);
        send(PC_CMD_HOME_DUAL_LIMIT,0x4449,fast,13);
        assert(reply==PC_STATUS_SUCCESS && last_speed[0]<0);
    }
    measured_setup();uint8_t too_fast[13]={255};p32(too_fast+1,9001);p32(too_fast+5,45000);p32(too_fast+9,120000);
    send(PC_CMD_HOME_DUAL_LIMIT,0x4449,too_fast,13);assert(reply==PC_STATUS_ILLEGAL_PARAM);
    puts("dual measured limits, sign, homing, stop, polarity and failure tests passed");
    return 0;
}
