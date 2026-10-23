#define main legacy_limits_main
#include "test_limits.c"
#undef main

static void setup_bank(void)
{
    memset(motors,0,sizeof(motors));memset(forced,0,sizeof(forced));
    memset(current_mode,0,sizeof(current_mode));memset(forced_speed,0,sizeof(forced_speed));
    memset(speed_parameters,0,sizeof(speed_parameters));memset(last_speed,0,sizeof(last_speed));
    now=1000;fail_reads=fail_stop=reject_speed_source=mapped_speed_input=range_not_supported=fail_second_armed=0;
    status_follows_setpoint=1;
    for (unsigned i=0;i<2;++i) {motors[i].slave_id=(uint8_t)(i+1);motors[i].bus=&bus;feedback_mode[i]=3;}
    Command_Manager_Init(&manager,&motors[0],&motors[1],NULL);
    for (unsigned i=0;i<2;++i) {
        manager.guard[i].ppr=10000;manager.guard[i].zero_valid=1;manager.guard[i].reference_angle=0;
        manager.guard[i].soft_on=1;manager.guard[i].minimum=-29000;manager.guard[i].maximum=35000;
    }
    manager.guard[0].angle_sign=-1;
}
static void put(uint8_t *p,uint32_t n)
{p[0]=(uint8_t)(n>>24);p[1]=(uint8_t)(n>>16);p[2]=(uint8_t)(n>>8);p[3]=(uint8_t)n;}
static void bank(uint8_t command,uint16_t slot,uint32_t token,uint8_t index,int32_t a,int32_t b)
{
    pc_command_packet_t c={0};c.device_id=1;c.command=command;c.parameter=slot;c.valid_payload=1;
    put(c.data,token);
    if (command==PC_CMD_BANK_BEGIN) {c.data[4]=0;c.data[5]=25;c.data[6]=0;c.data[7]=200;c.data[8]=index;c.data_length=9;}
    else if (command==PC_CMD_BANK_POINT) {c.data[4]=index;put(c.data+5,(uint32_t)a);put(c.data+9,(uint32_t)b);c.data_length=13;}
    else if (command==PC_CMD_BANK_RUN) {c.data[4]=index;c.data_length=5;}
    else if (command==PC_CMD_BANK_SEAL) c.data_length=4;
    Command_Process(&manager,&c);
}
static void upload(uint16_t slot,uint32_t token)
{
    const int32_t points[6][2]={{0,0},{6900,0},{16100,-9700},{24800,0},{16100,9700},{0,0}};
    bank(PC_CMD_BANK_BEGIN,slot,token,6,0,0);assert(reply==PC_STATUS_SUCCESS);
    for (uint8_t n=0;n<6;++n) {
        bank(PC_CMD_BANK_POINT,slot,token,n,points[n][0],points[n][1]);assert(reply==PC_STATUS_SUCCESS);
    }
    bank(PC_CMD_BANK_SEAL,slot,token,0,0,0);assert(reply==PC_STATUS_SUCCESS && manager.point_banks[slot-1].ready);
}
int main(void)
{
    setup_bank();upload(1,123);upload(2,456);
    assert(!manager.point_active[0] && !manager.point_active[1] && !last_speed[0] && !last_speed[1]);
    bank(PC_CMD_BANK_STATUS,1,0,0,0,0);assert(reply==PC_STATUS_SUCCESS && response_size==11 && response[10]);
    bank(PC_CMD_BANK_RUN,1,123,1,0,0);assert(reply==PC_STATUS_SUCCESS && response_size==8);
    assert(manager.point_active[0] && manager.point_active[1] && manager.point_target[0]==-9583 && manager.point_target[1]==0);
    assert(manager.point_tolerance[0]==277 && manager.point_tolerance[1]==277);
    bank(PC_CMD_BANK_BEGIN,2,999,6,0,0);assert(reply==PC_STATUS_BUSY && manager.point_banks[1].token==456);
    request(1,PC_CMD_MOTOR_STOP);request(2,PC_CMD_MOTOR_STOP);
    assert(manager.point_banks[0].ready && manager.point_banks[1].ready);
    bank(PC_CMD_BANK_RUN,1,122,2,0,0);assert(reply==PC_STATUS_ILLEGAL_PARAM && !manager.point_active[0]);
    manager.guard[1].origin+=1;
    bank(PC_CMD_BANK_RUN,1,123,2,0,0);assert(reply==PC_STATUS_NOT_REFERENCED && !manager.point_banks[0].ready);

    setup_bank();upload(1,123);
    bank(PC_CMD_BANK_BEGIN,1,124,2,0,0);assert(reply==PC_STATUS_SUCCESS && !manager.point_banks[0].ready);
    bank(PC_CMD_BANK_POINT,1,124,1,1000,1000);assert(reply==PC_STATUS_ILLEGAL_PARAM); /* Out of order. */
    bank(PC_CMD_BANK_POINT,1,124,0,36000,0);assert(reply==PC_STATUS_SUCCESS);
    bank(PC_CMD_BANK_SEAL,1,124,0,0,0);assert(reply==PC_STATUS_ILLEGAL_PARAM); /* Incomplete. */
    bank(PC_CMD_BANK_POINT,1,124,1,0,0);assert(reply==PC_STATUS_SUCCESS);
    bank(PC_CMD_BANK_SEAL,1,124,0,0,0);assert(reply==PC_STATUS_RANGE && !manager.point_banks[0].ready);
    assert(!manager.point_active[0] && !last_speed[0] && !last_speed[1]);
    bank(PC_CMD_BANK_BEGIN,1,125,17,0,0);assert(reply==PC_STATUS_ILLEGAL_PARAM);
    bank(PC_CMD_BANK_BEGIN,7,125,6,0,0);assert(reply==PC_STATUS_ILLEGAL_PARAM);

    setup_bank();upload(1,123);request(2,PC_CMD_SET_ZERO);
    assert(!manager.point_banks[0].ready); /* Reference command invalidates banks even at the same encoder count. */
    setup_bank();upload(1,123);Command_Manager_Init(&manager,&motors[0],&motors[1],NULL);
    bank(PC_CMD_BANK_RUN,1,123,1,0,0);assert(reply!=PC_STATUS_SUCCESS && !manager.point_active[0]);
    setup_bank();upload(1,123);bank(PC_CMD_BANK_RUN,1,123,1,0,0);
    for (unsigned i=0;i<2;++i) motors[i].status.position=manager.point_target[i]+250; /* 0.18 degree. */
    now+=50;Command_Manager_Task(&manager);
    assert(manager.point_active[0] && !(Command_Manager_PointFlags(&manager,0)&0x40)); /* One zero cannot complete. */
    now+=50;Command_Manager_Task(&manager);
    assert(Command_Manager_PointFlags(&manager,0)==0x40 && Command_Manager_PointFlags(&manager,1)==0x40);
    setup_bank();upload(1,123);bank(PC_CMD_BANK_RUN,1,123,1,0,0);
    for (unsigned i=0;i<2;++i) motors[i].status.position=manager.point_target[i]+300; /* 0.216 degree. */
    now+=50;Command_Manager_Task(&manager);
    assert(!(Command_Manager_PointFlags(&manager,0)&0x40) && manager.point_active[0]);
    setup_bank();upload(1,123);bank(PC_CMD_BANK_RUN,1,123,1,0,0);
    for (unsigned i=0;i<2;++i) motors[i].status.position=manager.point_target[i]+250; /* 0.18 degree. */
    now+=50;Command_Manager_Task(&manager);
    assert(manager.point_active[0] && !(Command_Manager_PointFlags(&manager,0)&0x40)); /* One zero cannot complete. */
    now+=50;Command_Manager_Task(&manager);
    assert(Command_Manager_PointFlags(&manager,0)==0x40 && Command_Manager_PointFlags(&manager,1)==0x40);
    setup_bank();upload(1,123);bank(PC_CMD_BANK_RUN,1,123,1,0,0);
    for (unsigned i=0;i<2;++i) motors[i].status.position=manager.point_target[i]+300; /* 0.216 degree. */
    now+=50;Command_Manager_Task(&manager);
    assert(!(Command_Manager_PointFlags(&manager,0)&0x40) && manager.point_active[0]);
    setup_bank();
    for (uint16_t slot=1;slot<=6;++slot) upload(slot,1000U+slot);
    for (uint16_t slot=1;slot<=6;++slot) {
        assert(manager.point_banks[slot-1].token==1000U+slot && manager.point_banks[slot-1].ready);
        bank(PC_CMD_BANK_RUN,slot,1000U+slot,1,0,0);
        assert(reply==PC_STATUS_SUCCESS && manager.point_active[0]);
        request(1,PC_CMD_MOTOR_STOP);request(2,PC_CMD_MOTOR_STOP);
    }
    setup_bank();
    for (uint8_t count=6;count<=16;count+=2) {
        uint32_t token=100U+count;
        bank(PC_CMD_BANK_BEGIN,6,token,count,0,0);assert(reply==PC_STATUS_SUCCESS);
        for (uint8_t n=0;n<count;++n) {
            bank(PC_CMD_BANK_POINT,6,token,n,(n%3)*1000,0);assert(reply==PC_STATUS_SUCCESS);
        }
        bank(PC_CMD_BANK_SEAL,6,token,0,0,0);assert(reply==PC_STATUS_SUCCESS);
        bank(PC_CMD_BANK_RUN,6,token,(uint8_t)(count-1),0,0);assert(reply==PC_STATUS_SUCCESS);
        request(1,PC_CMD_MOTOR_STOP);request(2,PC_CMD_MOTOR_STOP);
    }
    puts("point banks: six independent slots, 6/8/10/12/14/16 points, indexed run and invalidation passed");
    return 0;
}
