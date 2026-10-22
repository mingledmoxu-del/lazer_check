/* Shared point-motion path used by GUI and SDK; no hardware connection. */
#define main base_limits_main
#include "test_limits.c"
#undef main

static void setup_position(void)
{
    memset(motors,0,sizeof(motors));memset(forced,0,sizeof(forced));
    memset(current_mode,0,sizeof(current_mode));memset(position_parameters,0,sizeof(position_parameters));
    fail_stop=fail_reads=reject_position_parameter=lose_trigger_reply=0U;now=1000U;
    motors[0].slave_id=1U;motors[1].slave_id=2U;motors[0].bus=motors[1].bus=&bus;
    Command_Manager_Init(&manager,&motors[0],&motors[1],NULL);
    manager.guard[1].zero_valid=1U;manager.guard[1].origin=123;
    feedback_mode[1]=3U;manager.guard[1].ppr=10000U;
}
int main(void)
{
    setup_position();
    request(2U,PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply==PC_STATUS_SUCCESS && manager.position_watch[1]);
    assert(position_parameters[1][1]==20U && position_parameters[1][0]==1U);
    motors[1].status.position=100;now+=100U;Command_Manager_Task(&manager);
    assert(manager.guard[1].fault==GUARD_OK);
    request(2U,PC_CMD_MOTOR_STOP);
    assert(!manager.position_watch[1]);
    pc_command_packet_t c={0};c.device_id=2;c.command=PC_CMD_MOVE_OUTPUT_RELATIVE;c.valid_payload=1;
    c.data[1]=20U;c.data[2]=255U;c.data[3]=255U;c.data[4]=0xECU;c.data[5]=0x78U;
    Command_Process(&manager,&c); /* Correctly accepted negative new target. */
    assert(reply==PC_STATUS_SUCCESS && manager.position_direction[1]==-1);
    unsigned before=triggers[1];
    motors[1].status.position=200;motors[1].status.speed_raw=9;
    now+=100U;Command_Manager_Task(&manager); /* Drive incorrectly resumes old positive travel. */
    assert(manager.guard[1].fault==GUARD_EXECUTION && (forced[1]&4U));
    assert(manager.guard[1].zero_valid && manager.guard[1].origin==123);
    assert(!manager.position_watch[1] && triggers[1]==before);
    motors[1].status.speed_raw=0;now+=300U;Command_Manager_Task(&manager);
    now+=300U;Command_Manager_Task(&manager);
    request(2U,PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply!=PC_STATUS_SUCCESS && triggers[1]==before); /* Fault cannot auto-restart. */
    c.command=PC_CMD_CONFIG_GUARD;c.parameter=4U;
    Command_Process(&manager,&c);
    assert(reply==PC_STATUS_SUCCESS && manager.guard[1].fault==GUARD_OK);
    assert(manager.guard[1].zero_valid && manager.guard[1].origin==123 && triggers[1]==before);

    setup_position();request(2U,PC_CMD_MOVE_OUTPUT_RELATIVE);
    before=triggers[1];now+=3001U;Command_Manager_Task(&manager);
    assert(manager.guard[1].fault==GUARD_EXECUTION && triggers[1]==before && (forced[1]&4U));

    setup_position();reject_position_parameter=1U;before=triggers[1];
    request(2U,PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply!=PC_STATUS_SUCCESS && triggers[1]==before);

    setup_position();lose_trigger_reply=1U;before=triggers[1];
    request(2U,PC_CMD_MOVE_OUTPUT_RELATIVE);
    assert(reply!=PC_STATUS_SUCCESS && triggers[1]==before+1U && (forced[1]&4U));
    puts("position execution direction, no-start, readback and ambiguous-trigger tests passed");
    return 0;
}
