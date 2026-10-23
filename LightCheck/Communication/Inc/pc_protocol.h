#ifndef PC_PROTOCOL_H
#define PC_PROTOCOL_H
#include <stdint.h>

#define PC_PROTOCOL_MAX_FRAME 64U
#define PC_PROTOCOL_BUFFER_SIZE 512U
#define PC_CMD_WRITE_PA 0x01U
#define PC_CMD_READ_PA  0x02U
#define PC_CMD_SERVO_ON       0x03U
#define PC_CMD_SERVO_OFF      0x04U
#define PC_CMD_MOTOR_START    0x05U
#define PC_CMD_MOTOR_STOP     0x06U
#define PC_CMD_READ_SPEED     0x07U
#define PC_CMD_READ_POSITION  0x08U
#define PC_CMD_READ_ALARM     0x09U
#define PC_CMD_RESET_ALARM    0x0AU
#define PC_CMD_EMERGENCY_STOP 0x0BU
#define PC_CMD_SET_SPEED      0x10U
#define PC_CMD_MOVE_RELATIVE  0x11U
#define PC_CMD_MOVE_ABSOLUTE  0x12U
#define PC_CMD_JOG            0x13U
#define PC_CMD_MOVE_OUTPUT_RELATIVE 0x14U
#define PC_POINT_TRACK_KEY          0x5054U
#define PC_CMD_MOVE_OUTPUT_TRACKED  0x19U
#define PC_CMD_MOVE_AXES_POINT      0x1AU
#define PC_AXES_POINT_KEY          0x4A50U
#define PC_CMD_POINT_TIMING        0x1BU
#define PC_POINT_TIMING_KEY        0x544DU
#define PC_CMD_SET_MOTION_PROFILE   0x15U
#define PC_CMD_LIMIT_DEBUG          0x16U
#define PC_CMD_ROTATE_CONTINUOUS    0x17U
#define PC_CMD_ROTATE_KEEPALIVE     0x18U
#define PC_ROTATE_LEASE_MS          5000U
#define PC_ROTATE_KEEPALIVE_KEY     0x4352U
#define PC_LIMIT_DEBUG_KEY          0x4C44U
#define PC_LIMIT_DEBUG_LEASE_MS     5000U
#define PC_CMD_READ_FEEDBACK 0x20U
#define PC_CMD_CONFIG_GUARD  0x21U
#define PC_CMD_SET_ZERO      0x22U
#define PC_CMD_HOME          0x23U
#define PC_CMD_READ_GUARD    0x24U
#define PC_CMD_SET_REFERENCE 0x25U
#define PC_CMD_CONFIG_DUAL_LIMIT 0x26U
#define PC_CMD_READ_DUAL_LIMIT   0x27U
#define PC_CMD_HOME_DUAL_LIMIT   0x28U
#define PC_CMD_TRAJECTORY_BEGIN 0x30U
#define PC_CMD_TRAJECTORY_POINT 0x31U
#define PC_CMD_TRAJECTORY_SEAL  0x32U
#define PC_CMD_TRAJECTORY_START 0x33U
#define PC_CMD_TRAJECTORY_STATUS 0x34U
#define PC_CMD_TRAJECTORY_STOP 0x35U
#define PC_CMD_TRAJECTORY_KEEPALIVE 0x36U
#define PC_CMD_READ_AXES 0x37U
#define PC_CMD_TRAJECTORY_CLEAR 0x38U
#define PC_CMD_BANK_BEGIN 0x40U
#define PC_CMD_BANK_POINT 0x41U
#define PC_CMD_BANK_SEAL  0x42U
#define PC_CMD_BANK_RUN   0x43U
#define PC_CMD_BANK_STATUS 0x44U
#define PC_POINT_BANK_SLOTS 6U
#define PC_PROTOCOL_MAX_DATA  40U

typedef enum {
    PC_STATUS_SUCCESS = 0U, PC_STATUS_TIMEOUT = 1U, PC_STATUS_CRC_ERROR = 2U,
    PC_STATUS_SERVO_NO_RESPONSE = 3U, PC_STATUS_ILLEGAL_PARAM = 4U,
    PC_STATUS_RS485_ERROR = 5U, PC_STATUS_BUSY = 6U, PC_STATUS_UNKNOWN_CMD = 7U,
    PC_STATUS_LIMIT_ACTIVE = 8U, PC_STATUS_RANGE = 9U, PC_STATUS_NOT_REFERENCED = 10U
} pc_status_t;

typedef struct {
    uint8_t device_id;
    uint8_t command;
    uint16_t parameter;
    uint8_t save_mode;
    int16_t value;
    uint8_t data[PC_PROTOCOL_MAX_DATA];
    uint8_t data_length;
    uint8_t valid_payload;
} pc_command_packet_t;

typedef void (*pc_protocol_command_fn)(void *context, const pc_command_packet_t *command);
typedef struct {
    uint8_t buffer[PC_PROTOCOL_BUFFER_SIZE];
    uint16_t size;
    pc_protocol_command_fn command_fn;
    void *context;
} pc_protocol_t;

void PC_Protocol_Init(pc_protocol_t *protocol, pc_protocol_command_fn command_fn, void *context);
void PC_Protocol_Input(pc_protocol_t *protocol, const uint8_t *data, uint16_t length);
uint16_t PC_Protocol_BuildResponse(uint8_t *out, uint16_t capacity, uint8_t device_id,
                                   uint8_t command, pc_status_t status, uint16_t parameter, int16_t value);
uint16_t PC_Protocol_BuildResponseData(uint8_t *out, uint16_t capacity, uint8_t device_id,
                                      uint8_t command, pc_status_t status, uint16_t parameter,
                                      const uint8_t *data, uint8_t data_length);

#endif
