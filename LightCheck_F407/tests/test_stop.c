/* Reuse transport stubs; never connects to a drive. */
#define main legacy_limits_test_main
#include "test_limits.c"
#undef main

int main(void)
{
    motors[0].slave_id = 1U; motors[1].slave_id = 2U;
    motors[0].bus = motors[1].bus = &bus;
    Command_Manager_Init(&manager, &motors[0], &motors[1], NULL);
    for (uint16_t source = 0U; source <= 5U; ++source) {
        current_mode[1] = 1U;
        speed_parameters[1][0] = source;
        for (unsigned bank = 2U; bank < 6U; ++bank)
            speed_parameters[1][bank] = (uint16_t)(100U * bank);
        speed_parameters[0][2] = 123U;
        request(2U, PC_CMD_MOTOR_STOP);
        assert(reply == PC_STATUS_SUCCESS);
        assert(speed_parameters[1][0] == 1U);
        for (unsigned bank = 2U; bank < 6U; ++bank)
            assert(speed_parameters[1][bank] == 0U);
        assert(speed_parameters[0][2] == 123U); /* Do not stop another axis. */
        assert(holds[1] == 0U && triggers[1] == 0U);
    }
    speed_parameters[1][0] = 0U;
    speed_parameters[1][2] = 200U;
    fail_stop = 1U;
    request(2U, PC_CMD_MOTOR_STOP);
    assert(reply == PC_STATUS_SERVO_NO_RESPONSE);
    assert(speed_parameters[1][0] == 0U); /* Do not switch to uncleared banks. */
    fail_stop = 0U;
    reject_speed_source = 1U;
    request(2U, PC_CMD_MOTOR_STOP);
    assert(reply == PC_STATUS_ILLEGAL_PARAM); /* No false ACK on readback mismatch. */
    reject_speed_source = 0U;
    request(2U, PC_CMD_MOTOR_STOP);
    assert(reply == PC_STATUS_SUCCESS && speed_parameters[1][0] == 1U);

    current_mode[1] = 0U;
    forced[1] = 8U;
    request(2U, PC_CMD_MOTOR_STOP);
    assert(reply == PC_STATUS_SUCCESS && forced[1] == 4U);
    puts("speed-source stop regression tests passed");
    return 0;
}
