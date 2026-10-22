#include "axis_motion.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

int main(void)
{
    assert(AxisMotion_PointSpeed(51U,51U,10000U,253U,100U,0U,100U)==0U);
    assert(AxisMotion_PointSpeed(100000U,51U,10000U,253U,100U,0U,100U)==253U);
    uint16_t fast=AxisMotion_PointSpeed(3000U,51U,10000U,253U,100U,0U,100U);
    assert(fast>3000U*120U/10000U); /* No half-second proportional tail. */
    assert(AxisMotion_PointSpeed(3000U,51U,10000U,253U,1500U,0U,100U)<fast);
    assert(AxisMotion_PointSpeed(3000U,51U,10000U,253U,100U,300U,100U)<fast);
    assert(AxisMotion_PointSpeed(3000U,51U,10000U,253U,100U,0U,300U)<fast);
    for (uint32_t distance=52U;distance<10000U;distance+=37U) {
        uint16_t rpm=AxisMotion_PointSpeed(distance,51U,10000U,253U,100U,0U,100U);
        assert(rpm>=1U && rpm<=253U);
        if (rpm>1U) {
            uint64_t coast=((uint64_t)rpm*10000U*100U+59999U)/60000U;
            uint64_t brake=((uint64_t)rpm*rpm*10000U*100U+119999999U)/120000000U;
            assert(coast+brake<=distance-51U);
        }
    }
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
