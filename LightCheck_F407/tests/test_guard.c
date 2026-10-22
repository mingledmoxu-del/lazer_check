/* Reuse the established fake drive/PC transport, never access hardware. */
#define main legacy_limits_test_main
#include "test_limits.c"
#undef main

static void put32(uint8_t *p, int32_t v)
{
    uint32_t n=(uint32_t)v;p[0]=(uint8_t)(n>>24);p[1]=(uint8_t)(n>>16);p[2]=(uint8_t)(n>>8);p[3]=(uint8_t)n;
}
static void cmd(uint8_t op,uint16_t parameter,const uint8_t *data,unsigned size)
{
    pc_command_packet_t c={0};c.device_id=1;c.command=op;c.parameter=parameter;c.valid_payload=1;c.data_length=(uint8_t)size;
    if (size) memcpy(c.data,data,size);
    Command_Process(&manager,&c);
}
static void tick(uint8_t raw,unsigned ms)
{
    while(ms--) {++now;Command_Manager_SampleInputs(&manager,raw);}
}
static void task(uint8_t raw)
{
    tick(raw,100);Command_Manager_Task(&manager);
}
static void setup(void)
{
    memset(motors,0,sizeof(motors));memset(forced,0,sizeof(forced));memset(forced_speed,0,sizeof(forced_speed));
    memset(current_mode,0,sizeof(current_mode));memset(last_speed,0,sizeof(last_speed));
    memset(holds,0,sizeof(holds));fail_stop=fail_reads=inject_trip=mapped_speed_input=0;now=1000;
    motors[0].slave_id=1;motors[1].slave_id=2;motors[0].bus=motors[1].bus=&bus;
    feedback_mode[0]=feedback_mode[1]=3;
    Command_Manager_Init(&manager,&motors[0],&motors[1],NULL);
    /* Generic guard tests below use an uncalibrated axis; fixture tested separately. */
    manager.guard[0].reference_angle=0;
    manager.guard[0].fixed_range=0;
    manager.guard[0].soft_on=0;
    manager.guard[0].minimum=-90000;manager.guard[0].maximum=90000;
    tick(0,30);
}
static void configure_home(void)
{
    uint8_t map[]={255,255,0,1};cmd(PC_CMD_CONFIG_GUARD,1,map,4);assert(reply==PC_STATUS_SUCCESS);
    uint8_t policy[]={13,255}; /* Hardware + confirmed + endpoint, negative search. */
    cmd(PC_CMD_CONFIG_GUARD,3,policy,2);assert(reply==PC_STATUS_SUCCESS);
}
static void home(void)
{
    uint8_t data[16];put32(data,500);put32(data+4,30000);put32(data+8,2000);put32(data+12,90000);
    cmd(PC_CMD_HOME,0x484D,data,16);
}
static void test_explicit_reference(void)
{
    setup();manager.guard[0].fixed_range=1;manager.guard[0].soft_on=1;
    manager.guard[0].minimum=-35000;manager.guard[0].maximum=35000;
    motors[0].status.position=100000;
    manager.trajectory.state=TJ_READY;manager.trajectory.session=55;manager.trajectory.count=2;
    uint8_t angle[4];put32(angle,0);
    unsigned old_triggers=triggers[0];
    cmd(PC_CMD_SET_REFERENCE,0x5246,angle,4);
    assert(reply==PC_STATUS_SUCCESS && response_size==4);
    axis_guard_t *g=&manager.guard[0];int32_t actual=0;
    assert(g->zero_valid && g->reference_angle==0 && g->origin==100000 && g->fixed_range && g->soft_on);
    assert(manager.trajectory.state==TJ_IDLE && !manager.trajectory.session && !manager.trajectory.count);
    cmd(PC_CMD_TRAJECTORY_START,55,NULL,0);assert(reply==PC_STATUS_ILLEGAL_PARAM);
    assert(triggers[0]==old_triggers); /* A reference never sends a motion trigger. */
    assert(AxisGuard_Angle(g,106945,3,10000,50,1,&actual) && actual==5000);
    assert(AxisGuard_Angle(g,120834,3,10000,50,1,&actual) && actual==15000);
    assert(AxisGuard_Angle(g,116667,3,10000,50,1,&actual) && actual==12000);
    motors[0].status.position=116667;
    uint8_t move[6]={0,20,0,0,0,0};put32(move+2,1000);
    cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);assert(reply==PC_STATUS_SUCCESS);
    assert(AxisGuard_Angle(g,motors[0].status.position,3,10000,50,1,&actual) && actual==12000);
    /* Accepted target is not yet actual travel. */
    uint8_t policy[]={0,0};cmd(PC_CMD_CONFIG_GUARD,3,policy,2);
    assert(reply==PC_STATUS_RANGE && g->soft_on); /* Zero degrees cannot disable fixture protection. */
    put32(angle,36000);cmd(PC_CMD_SET_REFERENCE,0x5246,angle,4);
    assert(reply==PC_STATUS_RANGE && g->origin==100000 && g->reference_angle==0);
    cmd(PC_CMD_SET_ZERO,0x5A52,NULL,0);
    assert(reply==PC_STATUS_SUCCESS && g->reference_angle==25000); /* Legacy call retains +25. */

    pc_command_packet_t c={0};c.device_id=2;c.command=PC_CMD_SET_REFERENCE;c.parameter=0x5246;
    c.valid_payload=1;c.data_length=4;put32(c.data,150000);
    motors[1].status.position=200000;Command_Process(&manager,&c);
    assert(reply==PC_STATUS_SUCCESS && manager.guard[1].reference_angle==150000);
    assert(!manager.guard[1].fixed_range && !manager.guard[1].soft_on);
    assert(AxisGuard_Angle(&manager.guard[1],1200000,3,10000,50,1,&actual) && actual==870000);
    put32(c.data,0);speed_on_delay=1;Command_Process(&manager,&c);
    assert(reply==PC_STATUS_BUSY && manager.guard[1].reference_angle==150000);
    motors[1].status.speed_raw=0;

    /* Sensor homing uses the fixture's physical +25 pose, even after a zero-angle capture. */
    g->reference_angle=0;g->zero_valid=0;g->home=0;g->hardware_on=1;g->phase=HOME_FINAL_STOP;
    g->home_last=motors[0].status.position;g->home_rpm=1;g->home_budget=30000;
    g->home_deadline=now+90000;g->zero_samples=0;
    manager.continuous_active[0]=1;manager.continuous_until[0]=now+5000;
    task(1);task(1);
    assert(g->phase==HOME_DONE && g->reference_angle==25000 && g->soft_on);
    Command_Manager_Init(&manager,&motors[0],&motors[1],NULL);
    assert(!manager.guard[0].zero_valid && !manager.guard[1].zero_valid);
}
int main(void)
{
    setup();
    manager.guard[0].reference_angle=25000;
    manager.guard[0].fixed_range=1;
    manager.guard[0].minimum=-35000;manager.guard[0].maximum=35000;
    uint8_t before_ref[6]={0,20,0,0,0,1};
    cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,before_ref,6);assert(reply==PC_STATUS_NOT_REFERENCED);
    cmd(PC_CMD_SET_ZERO,0x5A52,NULL,0);
    assert(reply==PC_STATUS_SUCCESS && manager.guard[0].soft_on);
    int32_t pose=0;
    assert(AxisGuard_Angle(&manager.guard[0],0,3,10000,50,1,&pose) && pose==25000);
    assert(AxisGuard_Angle(&manager.guard[0],-34722,3,10000,50,1,&pose) && pose>=0 && pose<=1);
    assert(AxisGuard_TargetAllowed(&manager.guard[0],25000,10000));
    assert(!AxisGuard_TargetAllowed(&manager.guard[0],25000,10001));
    assert(AxisGuard_TargetAllowed(&manager.guard[0],25000,-60000));
    assert(!AxisGuard_TargetAllowed(&manager.guard[0],25000,-60001));
    uint8_t wider[8];put32(wider,-36000);put32(wider+4,35000);
    cmd(PC_CMD_CONFIG_GUARD,2,wider,8);assert(reply==PC_STATUS_RANGE);
    put32(wider,-34000);put32(wider+4,35000);
    cmd(PC_CMD_CONFIG_GUARD,2,wider,8);assert(reply==PC_STATUS_RANGE);
    uint8_t disable_fixed[]={0,0};
    cmd(PC_CMD_CONFIG_GUARD,3,disable_fixed,2);
    assert(reply==PC_STATUS_RANGE && manager.guard[0].soft_on);
    setup();
    cmd(PC_CMD_READ_FEEDBACK,0,NULL,0);
    assert(reply==PC_STATUS_SUCCESS && response_size==34 && !(response[23]&1));
    motors[0].status.position=100000;
    feedback_mode[0]=0;
    cmd(PC_CMD_SET_ZERO,0x5A52,NULL,0);
    assert(reply==PC_STATUS_SUCCESS && feedback_mode[0]==3 && manager.guard[0].origin==100000);
    uint8_t range[8];put32(range,-10000);put32(range+4,10000);
    cmd(PC_CMD_CONFIG_GUARD,2,range,8);assert(reply==PC_STATUS_SUCCESS);
    uint8_t policy[]={2,0};cmd(PC_CMD_CONFIG_GUARD,3,policy,2);assert(reply==PC_STATUS_SUCCESS);
    uint8_t move[6]={0,20,0,0,0,0};put32(move+2,5000);
    cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);assert(reply==PC_STATUS_SUCCESS);
    put32(move+2,11000);cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);assert(reply==PC_STATUS_RANGE);
    uint8_t speed[4];put32(speed,5000);cmd(PC_CMD_ROTATE_CONTINUOUS,0,speed,4);assert(reply==PC_STATUS_RANGE);
    cmd(PC_CMD_MOVE_ABSOLUTE,0,move,6);assert(reply==PC_STATUS_RANGE);
    feedback_mode[0]=0;task(0);
    assert(!manager.guard[0].zero_valid && manager.guard[0].fault==GUARD_FEEDBACK);
    task(0);task(0); /* Fault stays latched, but doesn't starve re-reference. */
    cmd(PC_CMD_SET_ZERO,0x5A52,NULL,0);assert(reply==PC_STATUS_SUCCESS);
    setup();
    uint8_t map[]={0,255,255,1};cmd(PC_CMD_CONFIG_GUARD,1,map,4);
    uint8_t hard[]={1,0};cmd(PC_CMD_CONFIG_GUARD,3,hard,2);assert(reply==PC_STATUS_SUCCESS);
    tick(1,1);tick(0,30);Command_Manager_Task(&manager);
    assert(holds[0]>0 && manager.guard[0].fault==GUARD_SENSOR); /* Short pulse latched. */
    tick(1,1);Command_Manager_Task(&manager);
    put32(move+2,-5000);cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);assert(reply==PC_STATUS_LIMIT_ACTIVE);
    put32(move+2,5000);cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,move,6);assert(reply==PC_STATUS_SUCCESS); /* Away from negative limit. */
    setup();home();assert(reply==PC_STATUS_NOT_REFERENCED);
    configure_home();tick(1,1);home();assert(reply==PC_STATUS_LIMIT_ACTIVE);
    tick(0,30);home();assert(reply==PC_STATUS_SUCCESS && manager.guard[0].phase==HOME_SEEK);
    motors[0].status.position=-100;task(1);assert(manager.guard[0].phase==HOME_CONTACT_STOP);
    task(1);task(1);assert(manager.guard[0].phase==HOME_BACKOFF && last_speed[0]>0);
    motors[0].status.position=0;task(0);assert(manager.guard[0].phase==HOME_BACKOFF_STOP);
    task(0);task(0);assert(manager.guard[0].phase==HOME_LATCH && last_speed[0]<0);
    motors[0].status.position=-50;task(1);task(1);task(1);
    assert(manager.guard[0].phase==HOME_DONE && manager.guard[0].zero_valid && manager.guard[0].origin==-50);
    assert(!manager.continuous_active[0]);
    setup();configure_home();home();motors[0].status.position=200;task(0);
    assert(manager.guard[0].phase==HOME_FAULT && manager.guard[0].fault==GUARD_FEEDBACK); /* Wrong direction. */
    setup();configure_home();home();motors[0].status.position=-100;task(1);task(1);task(1);
    motors[0].status.position=3000;task(1);
    assert(manager.guard[0].phase==HOME_FAULT && !manager.guard[0].zero_valid); /* Cannot exit / broken wire. */
    setup();configure_home();home();cmd(PC_CMD_MOTOR_STOP,0,NULL,0);task(0);task(0);
    assert(manager.guard[0].phase==HOME_FAULT && manager.guard[0].fault==GUARD_CANCELLED && last_speed[0]==0);
    setup();configure_home();home();now+=5001;Command_Manager_Task(&manager);
    assert(manager.guard[0].phase==HOME_FAULT && manager.guard[0].fault==GUARD_TIMEOUT);
    setup();configure_home();home();
    for (unsigned i=0;i<31;++i) task(0);
    assert(manager.guard[0].phase==HOME_FAULT && manager.guard[0].fault==GUARD_FEEDBACK); /* Stalled feedback. */
    setup();
    uint8_t small[6]={0,20,0,0,0,100};
    cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,small,6);
    assert(reply==PC_STATUS_SUCCESS && (forced[0]&8U));
    cmd(PC_CMD_MOTOR_STOP,0,NULL,0);
    assert(reply==PC_STATUS_SUCCESS && (forced[0]&4U) && !(forced[0]&8U));
    unsigned before=triggers[0];
    motors[0].status.speed_raw=-10;
    cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,small,6);
    assert(reply==PC_STATUS_BUSY && triggers[0]==before && (forced[0]&4U));
    motors[0].status.speed_raw=0;
    cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,small,6);
    assert(reply==PC_STATUS_SUCCESS && triggers[0]==before+1 && !(forced[0]&4U));
    cmd(PC_CMD_MOTOR_STOP,0,NULL,0);
    before=triggers[0];lose_trigger_reply=1U;
    cmd(PC_CMD_MOVE_OUTPUT_RELATIVE,0,small,6);
    assert(reply==PC_STATUS_SERVO_NO_RESPONSE && triggers[0]==before+1 && (forced[0]&4U));
    /* Ambiguous trigger delivery is paused, not retried. */
    /* The mock asserts every HOLD release is atomic with a fresh CTRG edge. */
    test_explicit_reference();
    puts("position, reference capture, input expansion, soft range, homing and stopped-retrigger tests passed");
    return 0;
}
