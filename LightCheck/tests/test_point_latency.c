#define main legacy_limits_main
#include "test_limits.c"
#undef main

static void setup(void)
{
    memset(motors,0,sizeof(motors));memset(forced,0,sizeof(forced));
    memset(current_mode,0,sizeof(current_mode));memset(forced_speed,0,sizeof(forced_speed));
    memset(speed_parameters,0,sizeof(speed_parameters));memset(last_speed,0,sizeof(last_speed));
    now=1000;fail_reads=fail_stop=reject_speed_source=mapped_speed_input=range_not_supported=fail_second_armed=0;
    speed_writes=range_reads=parameter_reads=0;status_follows_setpoint=1;
    wide_range_reads=0;fail_wide_range=MODBUS_OK;
    memset(status_read_count,0,sizeof(status_read_count));
    for (unsigned i=0;i<2;++i) {motors[i].slave_id=(uint8_t)(i+1);motors[i].bus=&bus;feedback_mode[i]=3;}
    Command_Manager_Init(&manager,&motors[0],&motors[1],NULL);
    for (unsigned i=0;i<2;++i) {
        manager.guard[i].ppr=10000;manager.guard[i].zero_valid=1;
        manager.guard[i].soft_on=1;manager.guard[i].minimum=-29000;manager.guard[i].maximum=35000;
        manager.guard[i].reference_angle=0;
    }
    manager.guard[0].angle_sign=-1;
}
static void joint(int32_t first,int32_t second)
{
    pc_command_packet_t c={0};c.device_id=1;c.command=PC_CMD_MOVE_AXES_POINT;
    c.parameter=PC_AXES_POINT_KEY;c.valid_payload=1;c.data_length=10;c.data[1]=20;
    int32_t angles[2]={first,second};
    for (unsigned i=0;i<2;++i) {
        uint32_t n=(uint32_t)angles[i];unsigned p=2+4*i;
        c.data[p]=(uint8_t)(n>>24);c.data[p+1]=(uint8_t)(n>>16);c.data[p+2]=(uint8_t)(n>>8);c.data[p+3]=(uint8_t)n;
    }
    Command_Process(&manager,&c);
}
int main(void)
{
    setup();
    for (unsigned i=0;i<2;++i) {
        current_mode[i]=1;forced_speed[i]=0x80;speed_parameters[i][0]=1;
    }
    joint(5000,-5000);
    assert(reply==PC_STATUS_SUCCESS && range_reads==4 && wide_range_reads==4);
    assert(status_read_count[0]==2 && status_read_count[1]==2);
    assert(!manager.point_preflight && !last_speed[0] && !last_speed[1]);
    /* Warm dispatch: only two parameter blocks and two independent zeros per
       axis; no duplicate idle guard poll and no motion before both are armed. */

    setup();joint(5000,-5000);
    assert(reply==PC_STATUS_SUCCESS && response_size==8);
    assert(manager.point_active[0] && manager.point_active[1]);
    assert(manager.point_target[0]==-6944 && manager.point_target[1]==-6944);
    assert(!last_speed[0] && !last_speed[1]); /* Neither starts during pair preparation. */
    assert(range_reads>=4);
    unsigned reads=status_read_count[1];now+=50;Command_Manager_Task(&manager);
    assert(status_read_count[1]-reads==1); /* Guard consumes the point loop's range/alarm validation. */
    assert(last_speed[0]<0 && last_speed[1]<0);
    request(1,PC_CMD_MOTOR_STOP);request(2,PC_CMD_MOTOR_STOP);
    unsigned writes=speed_writes;joint(1000,1000);
    assert(reply==PC_STATUS_SUCCESS && speed_writes==writes); /* Verified warm mode needs no PA24 reset write. */

    setup();fail_second_armed=1;joint(5000,5000);
    assert(reply==PC_STATUS_SERVO_NO_RESPONSE && !manager.point_active[0] && !manager.point_active[1]);
    assert(!last_speed[0] && !last_speed[1]); /* Second-axis failure cannot leave first armed. */
    setup();manager.guard[1].zero_valid=0;joint(5000,5000);
    assert(reply==PC_STATUS_NOT_REFERENCED && !manager.point_active[0]);
    setup();mapped_speed_input=1;joint(5000,5000);
    assert(reply==PC_STATUS_ILLEGAL_PARAM && !manager.point_active[0] && !manager.point_active[1]);
    setup();range_not_supported=1;joint(5000,5000);
    assert(reply==PC_STATUS_SUCCESS && manager.point_active[0] && manager.point_active[1]);
    assert(manager.point_range_unsupported[0]==3 && manager.point_range_unsupported[1]==3);
    request(1,PC_CMD_MOTOR_STOP);request(2,PC_CMD_MOTOR_STOP);
    unsigned wide=wide_range_reads;joint(1000,1000);
    assert(reply==PC_STATUS_SUCCESS && wide_range_reads==wide);
    request(1,PC_CMD_MOTOR_STOP);request(2,PC_CMD_MOTOR_STOP);
    mapped_speed_input=1;joint(1000,1000);
    assert(reply==PC_STATUS_ILLEGAL_PARAM); /* Fallback rereads values, never caches them. */

    setup();fail_wide_range=MODBUS_ERROR_CRC;joint(5000,5000);
    assert(reply==PC_STATUS_CRC_ERROR && wide_range_reads==1);
    assert(!manager.point_range_unsupported[0] && !manager.point_active[0] && !manager.point_active[1]);
    setup();fail_wide_range=MODBUS_ERROR_TIMEOUT;joint(5000,5000);
    assert(reply==PC_STATUS_SERVO_NO_RESPONSE && wide_range_reads==1);
    assert(!manager.point_range_unsupported[0] && !last_speed[0] && !last_speed[1]);
    setup();motors[0].status.position=-500000;joint(5000,5000);
    assert(reply==PC_STATUS_RANGE && !manager.point_active[0] && !manager.point_active[1]);

    setup();joint(5000,5000);reads=status_read_count[1];
    now+=50;motors[1].status.position=500000;Command_Manager_Task(&manager);
    assert(manager.guard[1].fault==GUARD_RANGE && !manager.point_active[1]);
    assert(status_read_count[1]>reads);
    setup();joint(0,0);assert(reply==PC_STATUS_SUCCESS);
    for (unsigned n=0;n<4;++n) {now+=50;Command_Manager_Task(&manager);}
    assert(Command_Manager_PointFlags(&manager,0)==0x40 && Command_Manager_PointFlags(&manager,1)==0x40);
    pc_command_packet_t timing={0};timing.device_id=1;timing.command=PC_CMD_POINT_TIMING;
    timing.parameter=PC_POINT_TIMING_KEY;timing.valid_payload=1;
    Command_Process(&manager,&timing);assert(reply==PC_STATUS_SUCCESS && response_size==32);
    setup();joint(5000,5000);feedback_mode[1]=4;now+=101;Command_Manager_Task(&manager);
    assert(manager.guard[1].fault==GUARD_FEEDBACK && !manager.guard[1].zero_valid);
    puts("low-latency point: batched reads, warm mode, one poll, paired arm/failure, limits and zero targets passed");
    return 0;
}
