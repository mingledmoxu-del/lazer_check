#include "axis_motion.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

int main(void)
{
    axis_motion_command_t command;

    assert(AxisMotion_ConvertRelative(20U, 180000, 50U, 1U, 10000U, &command));
    assert(command.motor_speed_rpm == 10);
    assert(command.motor_pulses == 250000);

    assert(AxisMotion_ConvertRelative(100U, -10000, 50U, 1U, 10000U, &command));
    assert(command.motor_speed_rpm == 50);
    assert(command.motor_pulses == -13889);

    assert(!AxisMotion_ConvertRelative(0U, 1000, 50U, 1U, 10000U, &command));
    assert(!AxisMotion_ConvertRelative(20U, 0, 50U, 1U, 10000U, &command));
    assert(!AxisMotion_ConvertRelative(20000U, 1000, 50U, 1U, 10000U, &command));
    assert(!AxisMotion_ConvertRelative(20U, INT32_MAX, 10000U, 1U, 65535U, &command));

    puts("axis motion conversion tests passed");
    int16_t rpm = 0;
    assert(AxisMotion_ConvertVelocity(5000, 50, 1, &rpm) && rpm == 42);
    assert(AxisMotion_ConvertVelocity(-5000, 50, 1, &rpm) && rpm == -42);
    assert(AxisMotion_ConvertVelocity(600000, 50, 1, &rpm) && rpm == 5000);
    assert(!AxisMotion_ConvertVelocity(0, 50, 1, &rpm));
    assert(!AxisMotion_ConvertVelocity(1, 50, 1, &rpm));
    assert(!AxisMotion_ConvertVelocity(INT32_MIN, 50, 1, &rpm));
    assert(!AxisMotion_ConvertVelocity(5000, 0, 1, &rpm));
    assert(!AxisMotion_ConvertVelocity(5000, 50, 0, &rpm));
    return 0;
}
