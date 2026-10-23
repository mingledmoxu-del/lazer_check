#include "p100s.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct mock_port_s {
    uint8_t         response[MODBUS_MAX_FRAME_SIZE];
    uint16_t        response_size;
    uint8_t         request[8];
    unsigned        calls;
    modbus_result_t result;
} mock_port_t;

static modbus_result_t mock_exchange(void          *context,
                                     const uint8_t *tx_data,
                                     uint16_t       tx_size,
                                     uint8_t       *rx_data,
                                     uint16_t       rx_capacity,
                                     uint16_t      *rx_size,
                                     uint32_t       timeout_ms) {
    mock_port_t *port = context;
    assert(tx_size == 8U && timeout_ms == 400U);
    assert(rx_capacity >= port->response_size);
    memcpy(port->request, tx_data, tx_size);
    memcpy(rx_data, port->response, port->response_size);
    *rx_size = port->response_size;
    ++port->calls;
    return port->result;
}

static void mock_seal(mock_port_t *port, uint16_t size) {
    uint16_t crc              = modbus_crc16(port->response, size);
    port->response[size]      = (uint8_t)crc;
    port->response[size + 1U] = (uint8_t)(crc >> 8);
    port->response_size       = size + 2U;
}

int main(void) {
    mock_port_t     port = {0};
    modbus_handle_t bus;
    p100s_handle_t  motor;
    uint16_t        values[100]     = {0};
    const uint8_t   read_request[]  = {1, 3, 0, 5, 0, 2, 0xD4, 0x0A};
    const uint8_t   write_request[] = {1, 6, 0, 5, 0, 100, 0x98, 0x20};
    assert(modbus_crc16(read_request, 6) == 0x0AD4U);
    assert(modbus_crc16(write_request, 6) == 0x2098U);
    assert(modbus_init(&bus, &port, mock_exchange, 400) == MODBUS_OK);
    assert(p100s_init(&motor, &bus, 1) == MODBUS_OK);

    const uint8_t reply[] = {1, 3, 4, 0, 150, 0, 75, 0x5A, 0x28};
    memcpy(port.response, reply, sizeof(reply));
    port.response_size = sizeof(reply);
    assert(modbus_read_holding(&bus, 1, 5, 2, values) == MODBUS_OK);
    assert(values[0] == 150 && values[1] == 75);
    assert(memcmp(port.request, read_request, 8) == 0);

    port.response[8] ^= 1;
    values[0]         = 1234;
    assert(modbus_read_holding(&bus, 1, 5, 2, values) == MODBUS_ERROR_CRC);
    assert(values[0] == 1234 && bus.busy == 0);
    port.response[0] = 2;
    mock_seal(&port, 7);
    assert(modbus_read_holding(&bus, 1, 5, 2, values) == MODBUS_ERROR_FRAME);
    port.response[0] = 1;
    port.response[1] = 4;
    mock_seal(&port, 7);
    assert(modbus_read_holding(&bus, 1, 5, 2, values) == MODBUS_ERROR_FRAME);
    port.response[1] = 3;
    port.response[2] = 2;
    mock_seal(&port, 7);
    assert(modbus_read_holding(&bus, 1, 5, 2, values) == MODBUS_ERROR_FRAME);
    port.response_size = 4;
    assert(modbus_read_holding(&bus, 1, 5, 2, values) == MODBUS_ERROR_FRAME);

    port.response[1] = 0x83;
    port.response[2] = 2;
    mock_seal(&port, 3);
    assert(p100s_read_alarm(&motor, values) == MODBUS_ERROR_EXCEPTION);
    assert(motor.exception_code == 2 && bus.busy == 0);
    port.result = MODBUS_ERROR_TIMEOUT;
    assert(p100s_read_alarm(&motor, values) == MODBUS_ERROR_TIMEOUT);
    assert(motor.exception_code == 0 && bus.busy == 0);
    port.result = MODBUS_OK;

    unsigned calls = port.calls;
    assert(modbus_read_holding(&bus, 1, 0, 0, values) == MODBUS_ERROR_ARGUMENT);
    assert(modbus_read_holding(&bus, 1, 0, 101, values) == MODBUS_ERROR_ARGUMENT);
    assert(modbus_read_holding(&bus, 1, 65535, 2, values) == MODBUS_ERROR_ARGUMENT);
    assert(modbus_read_holding(&bus, 0, 0, 1, values) == MODBUS_ERROR_ARGUMENT);
    bus.busy = 1;
    assert(modbus_read_holding(&bus, 1, 0, 1, values) == MODBUS_ERROR_BUSY);
    bus.busy = 0;
    assert(port.calls == calls);

    memcpy(port.response, write_request, 8);
    port.response_size = 8;
    assert(modbus_write_single(&bus, 1, 5, 100) == MODBUS_OK);
    port.response[5] = 101;
    mock_seal(&port, 6);
    assert(modbus_write_single(&bus, 1, 5, 100) == MODBUS_ERROR_FRAME);
    port.response[1] = 0x86;
    port.response[2] = 3;
    mock_seal(&port, 3);
    assert(modbus_write_single(&bus, 1, 5, 100) == MODBUS_ERROR_EXCEPTION);

    port.response[1] = 6;
    port.response[2] = 0;
    port.response[3] = 0xA2;
    port.response[4] = 0;
    port.response[5] = 100;
    mock_seal(&port, 6);
    assert(p100s_write_parameter(&motor, P100S_GROUP_PA, 34, 100, P100S_STORAGE_TEMPORARY) == MODBUS_OK);
    assert(port.request[2] == 0 && port.request[3] == 0xA2);
    calls = port.calls;
    assert(p100s_write_parameter(&motor, P100S_GROUP_P4, 2, 1, P100S_STORAGE_TEMPORARY) == MODBUS_ERROR_ARGUMENT);
    assert(p100s_read_parameter(&motor, P100S_GROUP_PA, 128, values) == MODBUS_ERROR_ARGUMENT);
    assert(port.calls == calls);

    memset(port.response, 0, sizeof(port.response));
    port.response[0]  = 1;
    port.response[1]  = 3;
    port.response[2]  = 40;
    port.response[3]  = 0xFF;
    port.response[4]  = 0x9C; /* -100 */
    port.response[5]  = 0xFF;
    port.response[6]  = 0xFF;
    port.response[7]  = 0xFF;
    port.response[8]  = 0xFF; /* -1 */
    port.response[11] = 0x80; /* INT32_MIN */
    port.response[13] = 0xFF;
    port.response[14] = 0xFF;
    port.response[15] = 0x7F;
    port.response[16] = 0xFF; /* INT32_MAX */
    port.response[22] = 1; /* Live control mode, status word 9. */
    port.response[42] = 7;
    mock_seal(&port, 43);
    assert(p100s_read_status(&motor) == MODBUS_OK);
    assert(motor.status_valid && motor.status.speed_raw == -100);
    assert(motor.status.position == -1);
    assert(motor.status.command_position == INT32_MIN);
    assert(motor.status.position_error == INT32_MAX);
    assert(motor.status.alarm_code == 7);
    assert(motor.status.control_mode == 1);
    port.result = MODBUS_ERROR_TIMEOUT;
    assert(p100s_read_status(&motor) == MODBUS_ERROR_TIMEOUT);
    assert(!motor.status_valid && motor.status.position == -1);
    port.result = MODBUS_OK;

    memset(port.response, 0, sizeof(port.response));
    port.response[0] = 1;
    port.response[1] = 3;
    port.response[2] = 200;
    mock_seal(&port, 203);
    assert(modbus_read_holding(&bus, 1, 0, 100, values) == MODBUS_OK);
    puts("PASS: CRC vectors, read/write, malformed frames, exceptions, timeout, bounds, temporary addressing, signed "
         "positions, stale status, maximum frame");
    return 0;
}
