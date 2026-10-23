#define main base_limits_main
#include "test_limits.c"
#undef main

static void setup_point(void)
{
    memset(motors,0,sizeof(motors));memset(forced,0,sizeof(forced));memset(current_mode,0,sizeof(current_mode));
    memset(last_speed,0,sizeof(last_speed));memset(speed_parameters,0,sizeof(speed_parameters));
    memset(forced_speed,0,sizeof(forced_speed));
    now=1000U;fail_reads=fail_stop=reject_speed_source=mapped_speed_input=0U;
    status_follows_setpoint=1U;
    motors[0].slave_id=1U;motors[1].slave_id=2U;motors[0].bus=motors[1].bus=&bus;
    Command_Manager_Init(&manager,&motors[0],&motors[1],NULL);
    manager.guard[1].ppr=10000U;manager.guard[1].zero_valid=1U;feedback_mode[1]=3U;
}
static void move_point(int32_t angle)
{
    pc_command_packet_t c={0};c.device_id=2U;c.command=PC_CMD_MOVE_OUTPUT_TRACKED;
    c.parameter=PC_POINT_TRACK_KEY;c.valid_payload=1U;
    c.data[1]=20U;uint32_t raw=(uint32_t)angle;
    c.data[2]=(uint8_t)(raw>>24);c.data[3]=(uint8_t)(raw>>16);c.data[4]=(uint8_t)(raw>>8);c.data[5]=(uint8_t)raw;
    Command_Process(&manager,&c);
}
static void tick_point(void)
{
    now+=50U;
    motors[1].status.position+=(int32_t)last_speed[1]*10000/1200;
    motors[1].status.speed_raw=last_speed[1];
    Command_Manager_Task(&manager);
}
int main(void)
{
    setup_point();unsigned old_triggers=triggers[1];
    move_point(5000);assert(reply==PC_STATUS_SUCCESS && manager.point_active[1]);
    tick_point();assert(last_speed[1]>0 && last_speed[1]<manager.point_max_rpm[1]); /* No speed step. */
    assert(Command_Manager_PointFlags(&manager,1U)==0x20U);
    for (unsigned n=0;n<15;++n) tick_point();
    assert(last_speed[1]>0 && motors[1].status.position>0);
    unsigned feedback_reads=status_read_count[1];
    request(1U,PC_CMD_READ_AXES);
    assert(reply==PC_STATUS_SUCCESS && response_size==40U && response[39]==1U);
    assert(status_read_count[1]==feedback_reads); /* A fresh point sample is reused. */
    request(2U,PC_CMD_READ_FEEDBACK);
    assert(reply==PC_STATUS_SUCCESS && response_size==34U && status_read_count[1]==feedback_reads);
    request(2U,PC_CMD_MOTOR_STOP);assert(!manager.point_active[1] && !last_speed[1]);
    motors[1].status.speed_raw=0;int32_t stopped=motors[1].status.position;
    move_point(-5000);assert(reply==PC_STATUS_SUCCESS);
    assert(manager.point_target[1]==stopped-6944);
    tick_point();assert(last_speed[1]<0 && manager.guard[1].fault==GUARD_OK);
    for (unsigned n=0;n<500 && manager.point_active[1];++n) tick_point();
    assert(!manager.point_active[1] && !last_speed[1] && manager.guard[1].fault==GUARD_OK);
    int64_t error=(int64_t)motors[1].status.position-manager.point_target[1];
    assert(error>=-51 && error<=51 && manager.guard[1].zero_valid);
    assert(Command_Manager_PointFlags(&manager,1U)==0x40U);
    pc_command_packet_t fresh={0};fresh.device_id=1U;fresh.command=PC_CMD_READ_AXES;
    fresh.parameter=0x4652U;fresh.valid_payload=1U;
    feedback_reads=status_read_count[1];Command_Process(&manager,&fresh);
    assert(response[39]==0U && status_read_count[1]>feedback_reads); /* Forced real drive reply. */
    request(2U,PC_CMD_MOTOR_STOP);assert(!Command_Manager_PointFlags(&manager,1U));
    assert(triggers[1]==old_triggers); /* No HOLD release / CTRG replay / P4 task. */

    /* Fast configured point under a finite drive deceleration model. */
    setup_point();status_follows_setpoint=0U;move_point(5000);
    manager.point_max_rpm[1]=253;manager.point_decel_ms[1]=1500;
    int16_t actual_rpm=0;
    for (unsigned n=0;n<1000U && manager.point_active[1];++n) {
        int16_t previous=actual_rpm;
        int32_t delta=(int32_t)last_speed[1]-actual_rpm;
        if (delta>33) delta=33;
        if (delta < -33) delta=-33;
        actual_rpm=(int16_t)(actual_rpm+delta);
        motors[1].status.position+=((int32_t)previous+actual_rpm)*10000/2400;
        motors[1].status.speed_raw=actual_rpm;now+=50U;Command_Manager_Task(&manager);
    }
    assert(!manager.point_active[1] && !manager.guard[1].fault);
    error=(int64_t)motors[1].status.position-manager.point_target[1];
    assert(error>=-51 && error<=51);

    setup_point();move_point(5000);
    for (unsigned n=0;n<10U;++n) tick_point();
    stopped=motors[1].status.position;
    move_point(-5000);assert(reply==PC_STATUS_SUCCESS && manager.point_target[1]==stopped-6944);
    tick_point();assert(last_speed[1]<0 && manager.guard[1].fault==GUARD_OK);

    /* A small overshoot is corrected after braking; no operator fault clear. */
    setup_point();move_point(5000);tick_point();
    motors[1].status.position=manager.point_target[1]+100;
    now+=50U;Command_Manager_Task(&manager);assert(!last_speed[1]);
    tick_point();assert(last_speed[1]<0 && manager.guard[1].fault==GUARD_OK);
    for (unsigned n=0;n<100U && manager.point_active[1];++n) tick_point();
    assert(!manager.point_active[1] && manager.guard[1].fault==GUARD_OK);

    /* Replacing a live task waits for stop; a never-stopping drive cannot be
       acknowledged as the new target. */
    setup_point();move_point(5000);tick_point();motors[1].status.speed_raw=10;
    status_follows_setpoint=0U;
    move_point(-5000);assert(reply==PC_STATUS_BUSY && !manager.point_active[1] && !last_speed[1]);

    setup_point();move_point(5000);tick_point();
    motors[1].status.position=-100;now+=50U;Command_Manager_Task(&manager);
    assert(manager.guard[1].fault==GUARD_EXECUTION && !manager.point_active[1] && !last_speed[1]);

    setup_point();move_point(5000);tick_point();fail_reads=1U;
    now+=50U;Command_Manager_Task(&manager);
    assert(!manager.point_active[1] && manager.guard[1].stop_pending && !manager.guard[1].zero_valid);

    setup_point();move_point(5000);tick_point();
    now+=1001U;feedback_mode[1]=4U;Command_Manager_Task(&manager);
    assert(manager.guard[1].fault==GUARD_FEEDBACK && !manager.point_active[1]);
    puts("tracked targets: stop/reverse, actual-pose origin, convergence, no-P4 and fault tests passed");
    return 0;
}
