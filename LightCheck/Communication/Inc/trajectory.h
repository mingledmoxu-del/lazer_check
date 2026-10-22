#ifndef TRAJECTORY_H
#define TRAJECTORY_H
#include <stdint.h>
#include "pc_protocol.h"
#define TRAJECTORY_CAPACITY 128U
#define TRAJECTORY_LEASE_MS 3000U
enum { TJ_IDLE, TJ_UPLOADING, TJ_READY, TJ_RUNNING, TJ_SETTLING, TJ_STOPPING,
       TJ_DONE, TJ_FAULT, TJ_CANCELLED };
enum { TJ_OK, TJ_USER_STOP, TJ_REFERENCE, TJ_RANGE, TJ_LIMIT, TJ_COMM,
       TJ_ALARM, TJ_TRACKING, TJ_DEADLINE, TJ_HEARTBEAT, TJ_START_POSITION,
       TJ_BUFFER, TJ_SETTLE_TIMEOUT, TJ_PROGRESS_TIMEOUT };
typedef struct { uint32_t time; int32_t angle[2]; } trajectory_point_t;
typedef struct {
    trajectory_point_t points[TRAJECTORY_CAPACITY];
    uint16_t session, count, loaded, cycle_ms;
    uint8_t state, fault, zero_samples;
    int32_t maximum_speed, maximum_accel, maximum_error, tolerance;
    uint32_t started, last_tick, next_tick, lease_until, retry_at, units_at;
    uint32_t elapsed, max_gap, last_cycle;
    uint32_t sampled_at[2], settling_at;
    uint8_t sample_valid[2], adaptive;
    int32_t actual[2], error[2], velocity[2];
    uint32_t trips[2][3];
} trajectory_t;
struct command_manager;
uint8_t Trajectory_Active(const trajectory_t *t);
void Trajectory_Cancel(struct command_manager *m, uint8_t reason);
void Trajectory_Task(struct command_manager *m);
void Trajectory_Command(struct command_manager *m, const pc_command_packet_t *command);
#endif
