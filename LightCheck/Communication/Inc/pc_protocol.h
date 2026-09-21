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
#define PC_PROTOCOL_MAX_DATA  7U

typedef enum {
    PC_STATUS_SUCCESS = 0U, PC_STATUS_TIMEOUT = 1U, PC_STATUS_CRC_ERROR = 2U,
    PC_STATUS_SERVO_NO_RESPONSE = 3U, PC_STATUS_ILLEGAL_PARAM = 4U,
    PC_STATUS_RS485_ERROR = 5U, PC_STATUS_BUSY = 6U, PC_STATUS_UNKNOWN_CMD = 7U
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
