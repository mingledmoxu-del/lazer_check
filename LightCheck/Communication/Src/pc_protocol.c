#include "pc_protocol.h"
#include "crc16.h"
#include <string.h>

static void discard_prefix(pc_protocol_t *p, uint16_t count)
{
    if (count >= p->size) p->size = 0U;
    else { memmove(p->buffer, p->buffer + count, p->size - count); p->size -= count; }
}

static uint8_t has_complete_frame_after_prefix(const pc_protocol_t *p, uint16_t *offset)
{
    for (uint16_t i = 1U; i + 2U < p->size; ++i) {
        if (p->buffer[i] != 0xAAU || p->buffer[i + 1U] != 0x55U) continue;
        uint8_t nested_length = p->buffer[i + 2U];
        uint16_t nested_total = (uint16_t)nested_length + 3U;
        if (nested_length < 6U || nested_total > PC_PROTOCOL_MAX_FRAME ||
            (uint32_t)i + nested_total > p->size) continue;
        uint16_t nested_crc = CRC16_Modbus(p->buffer + i, nested_total - 2U);
        uint16_t nested_received = (uint16_t)p->buffer[i + nested_total - 2U] |
                                   ((uint16_t)p->buffer[i + nested_total - 1U] << 8);
        if (nested_crc == nested_received) {
            *offset = i;
            return 1U;
        }
    }
    return 0U;
}

void PC_Protocol_Init(pc_protocol_t *protocol, pc_protocol_command_fn command_fn, void *context)
{
    if (protocol == NULL) return;
    memset(protocol, 0, sizeof(*protocol)); protocol->command_fn = command_fn; protocol->context = context;
}

static void parse(pc_protocol_t *p)
{
    while (p->size >= 3U) {
        uint16_t start = 0U;
        while (start + 1U < p->size && (p->buffer[start] != 0xAAU || p->buffer[start + 1U] != 0x55U)) ++start;
        if (start != 0U) { discard_prefix(p, start); continue; }
        uint8_t length = p->buffer[2];
        uint16_t total = (uint16_t)length + 3U;
        if (length < 6U || total > PC_PROTOCOL_MAX_FRAME) { discard_prefix(p, 1U); continue; }
        if (p->size < total) {
            uint16_t nested_offset;
            /* 错误但“看似合法”的 Length 不得永久挡住后续完整帧。 */
            if (has_complete_frame_after_prefix(p, &nested_offset)) {
                discard_prefix(p, nested_offset);
                continue;
            }
            return;
        }
        uint16_t crc = CRC16_Modbus(p->buffer, total - 2U);
        uint16_t received = (uint16_t)p->buffer[total - 2U] | ((uint16_t)p->buffer[total - 1U] << 8);
        if (crc == received) {
            pc_command_packet_t c = {0}; c.device_id = p->buffer[3]; c.command = p->buffer[4];
            c.parameter = ((uint16_t)p->buffer[5] << 8) | p->buffer[6];
            uint16_t data_len = (uint16_t)length - 6U;
            c.data_length = (uint8_t)data_len;
            if (data_len <= PC_PROTOCOL_MAX_DATA) memcpy(c.data, p->buffer + 7U, data_len);
            switch (c.command) {
            case PC_CMD_TRAJECTORY_BEGIN: c.valid_payload=(uint8_t)(data_len==20U); break;
            case PC_CMD_TRAJECTORY_POINT: c.valid_payload=(uint8_t)(data_len==14U); break;
            case PC_CMD_TRAJECTORY_SEAL:
            case PC_CMD_TRAJECTORY_START:
            case PC_CMD_TRAJECTORY_STATUS:
            case PC_CMD_TRAJECTORY_STOP:
            case PC_CMD_TRAJECTORY_KEEPALIVE: c.valid_payload=(uint8_t)(data_len==0U); break;
            case PC_CMD_TRAJECTORY_CLEAR: c.valid_payload=(uint8_t)(data_len==0U); break;
            case PC_CMD_READ_AXES: c.valid_payload=(uint8_t)(data_len==0U && (c.parameter==0U || c.parameter==0x4652U)); break;
            case PC_CMD_WRITE_PA:       c.valid_payload = (uint8_t)(data_len == 3U); break;
            case PC_CMD_READ_PA:
            case PC_CMD_SERVO_ON:
            case PC_CMD_SERVO_OFF:
            case PC_CMD_MOTOR_START:
            case PC_CMD_MOTOR_STOP:
            case PC_CMD_READ_SPEED:
            case PC_CMD_READ_POSITION:
            case PC_CMD_READ_ALARM:
            case PC_CMD_RESET_ALARM:
            case PC_CMD_EMERGENCY_STOP: c.valid_payload = (uint8_t)(data_len == 0U); break;
            case PC_CMD_SET_SPEED:      c.valid_payload = (uint8_t)(data_len == 2U); break;
            case PC_CMD_LIMIT_DEBUG:    c.valid_payload = (uint8_t)(data_len == 1U); break;
            case PC_CMD_ROTATE_CONTINUOUS: c.valid_payload = (uint8_t)(data_len == 4U); break;
            case PC_CMD_ROTATE_KEEPALIVE: c.valid_payload = (uint8_t)(data_len == 0U); break;
            case PC_CMD_READ_FEEDBACK:
            case PC_CMD_READ_GUARD:
            case PC_CMD_SET_ZERO: c.valid_payload = (uint8_t)(data_len == 0U); break;
            case PC_CMD_SET_REFERENCE: c.valid_payload=(uint8_t)(data_len==4U); break;
            case PC_CMD_CONFIG_DUAL_LIMIT: c.valid_payload=(uint8_t)(data_len==14U && c.parameter==0x444CU); break;
            case PC_CMD_READ_DUAL_LIMIT: c.valid_payload=(uint8_t)(data_len==0U && c.parameter==0U); break;
            case PC_CMD_HOME_DUAL_LIMIT: c.valid_payload=(uint8_t)(data_len==13U && (c.parameter==0x4448U || c.parameter==0x4449U)); break;
            case PC_CMD_HOME: c.valid_payload = (uint8_t)(data_len == 16U); break;
            case PC_CMD_CONFIG_GUARD:
                c.valid_payload = (uint8_t)((c.parameter == 1U && data_len == 4U) ||
                    (c.parameter == 2U && data_len == 8U) || (c.parameter == 3U && data_len == 2U) ||
                    (c.parameter == 4U && data_len == 0U));
                break;
            case PC_CMD_MOVE_RELATIVE:
            case PC_CMD_MOVE_ABSOLUTE:
            case PC_CMD_MOVE_OUTPUT_RELATIVE:
            case PC_CMD_SET_MOTION_PROFILE:
                                        c.valid_payload = (uint8_t)(data_len == 6U); break;
            case PC_CMD_MOVE_OUTPUT_TRACKED:
                c.valid_payload=(uint8_t)(data_len==6U && c.parameter==PC_POINT_TRACK_KEY);break;
            case PC_CMD_JOG:            c.valid_payload = (uint8_t)(data_len == 7U); break;
            default:                    c.valid_payload = 0U; break;
            }
            if (c.command == PC_CMD_WRITE_PA && data_len >= 3U) {
                c.save_mode = p->buffer[7];
                c.value = (int16_t)(((uint16_t)p->buffer[8] << 8) | p->buffer[9]);
            }
            /* Unknown commands are reported by the manager; malformed known payloads are illegal. */
            if (p->command_fn != NULL) p->command_fn(p->context, &c);
        }
        if (crc == received) {
            discard_prefix(p, total);
        } else {
            /* 只滑过一个字节并重新搜索帧头，避免错误长度/CRC 吞掉后续合法帧。 */
            discard_prefix(p, 1U);
        }
    }
}

void PC_Protocol_Input(pc_protocol_t *protocol, const uint8_t *data, uint16_t length)
{
    if (protocol == NULL || data == NULL || length == 0U) return;
    if (length >= PC_PROTOCOL_BUFFER_SIZE) { data += length - PC_PROTOCOL_BUFFER_SIZE + 1U; length = PC_PROTOCOL_BUFFER_SIZE - 1U; }
    if ((uint32_t)protocol->size + length > PC_PROTOCOL_BUFFER_SIZE) {
        uint16_t drop = (uint16_t)((uint32_t)protocol->size + length - PC_PROTOCOL_BUFFER_SIZE);
        discard_prefix(protocol, drop);
    }
    memcpy(protocol->buffer + protocol->size, data, length); protocol->size += length; parse(protocol);
}

uint16_t PC_Protocol_BuildResponse(uint8_t *out, uint16_t capacity, uint8_t device_id,
                                   uint8_t command, pc_status_t status, uint16_t parameter, int16_t value)
{
    uint8_t data[2] = {(uint8_t)((uint16_t)value >> 8), (uint8_t)value};
    return PC_Protocol_BuildResponseData(out, capacity, device_id, command, status, parameter, data, 2U);
}

uint16_t PC_Protocol_BuildResponseData(uint8_t *out, uint16_t capacity, uint8_t device_id,
                                      uint8_t command, pc_status_t status, uint16_t parameter,
                                      const uint8_t *data, uint8_t data_length)
{
    uint16_t total = (uint16_t)(10U + data_length);
    if (out == NULL || data_length > PC_PROTOCOL_MAX_DATA || capacity < total ||
        (data_length != 0U && data == NULL)) return 0U;
    out[0]=0xAAU; out[1]=0x55U; out[2]=(uint8_t)(7U + data_length); out[3]=device_id;
    out[4]=command; out[5]=(uint8_t)status; out[6]=(uint8_t)(parameter >> 8); out[7]=(uint8_t)parameter;
    if (data_length != 0U) memcpy(out + 8U, data, data_length);
    uint16_t crc=CRC16_Modbus(out,total - 2U); out[total - 2U]=(uint8_t)crc; out[total - 1U]=(uint8_t)(crc >> 8);
    return total;
}
