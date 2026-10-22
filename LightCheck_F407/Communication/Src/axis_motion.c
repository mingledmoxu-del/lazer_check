#include "axis_motion.h"
#include <limits.h>
#include <stddef.h>

static int64_t divide_round_nearest(int64_t numerator, int64_t denominator)
{
    if (numerator >= 0) return (numerator + denominator / 2) / denominator;
    return -((-numerator + denominator / 2) / denominator);
}

int AxisMotion_ConvertVelocity(int32_t millideg_per_sec, uint16_t ratio_num,
                               uint16_t ratio_den, int16_t *motor_rpm)
{
    if (motor_rpm == NULL || millideg_per_sec == 0 || ratio_num == 0U || ratio_den == 0U)
        return 0;
    int64_t rpm = divide_round_nearest((int64_t)millideg_per_sec * ratio_num,
                                      (int64_t)6000 * ratio_den);
    if (rpm == 0 || rpm < -5000 || rpm > 5000) return 0;
    *motor_rpm = (int16_t)rpm;
    return 1;
}

int AxisMotion_ConvertRelative(uint16_t output_speed_centi_rpm,
                               int32_t output_angle_millideg,
                               uint16_t ratio_num,
                               uint16_t ratio_den,
                               uint16_t pulses_per_motor_rev,
                               axis_motion_command_t *command)
{
    if (command == NULL || output_speed_centi_rpm == 0U || output_angle_millideg == 0 ||
        ratio_num == 0U || ratio_den == 0U || pulses_per_motor_rev == 0U)
        return 0;

    int64_t speed_numerator = (int64_t)output_speed_centi_rpm * ratio_num;
    int64_t speed_denominator = (int64_t)100 * ratio_den;
    int64_t motor_speed = divide_round_nearest(speed_numerator, speed_denominator);
    if (motor_speed < 1 || motor_speed > 5000) return 0;

    int64_t pulse_numerator = (int64_t)output_angle_millideg * ratio_num * pulses_per_motor_rev;
    int64_t pulse_denominator = (int64_t)360000 * ratio_den;
    int64_t pulses = divide_round_nearest(pulse_numerator, pulse_denominator);
    if (pulses == 0 || pulses < INT32_MIN || pulses > INT32_MAX) return 0;

    command->motor_speed_rpm = (int16_t)motor_speed;
    command->motor_pulses = (int32_t)pulses;
    return 1;
}
