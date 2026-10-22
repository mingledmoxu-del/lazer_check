#ifndef AXIS_MOTION_H
#define AXIS_MOTION_H

#include <stdint.h>

typedef struct {
    int16_t motor_speed_rpm;
    int32_t motor_pulses;
} axis_motion_command_t;

/* Signed output angular speed in 0.001 degrees/second, to signed motor rpm. */
int AxisMotion_ConvertVelocity(int32_t millideg_per_sec, uint16_t ratio_num,
                               uint16_t ratio_den, int16_t *motor_rpm);

/*
 * output_speed_centi_rpm: 输出轴速度，单位 0.01 rpm，必须大于 0。
 * output_angle_millideg: 输出轴相对角度，单位 0.001°，正负表示方向。
 * ratio_num / ratio_den: 电机转数 : 输出轴转数。
 */
int AxisMotion_ConvertRelative(uint16_t output_speed_centi_rpm,
                               int32_t output_angle_millideg,
                               uint16_t ratio_num,
                               uint16_t ratio_den,
                               uint16_t pulses_per_motor_rev,
                               axis_motion_command_t *command);

#endif
