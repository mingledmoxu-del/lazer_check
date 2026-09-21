#include "modbus_rtu.h"
#include "crc16.h"
#include <string.h>

uint16_t modbus_crc16(const uint8_t *data, size_t size) { return CRC16_Modbus(data, size); }

modbus_result_t modbus_init(modbus_handle_t *bus, void *context, modbus_exchange_fn exchange, uint32_t timeout_ms) {
    if (bus == NULL)
        return MODBUS_ERROR_ARGUMENT;
    memset(bus, 0, sizeof(*bus));
    if (exchange == NULL || timeout_ms == 0U)
        return MODBUS_ERROR_ARGUMENT;
    bus->context    = context;
    bus->exchange   = exchange;
    bus->timeout_ms = timeout_ms;
    return MODBUS_OK;
}

static modbus_result_t modbus_request(
    modbus_handle_t *bus, uint8_t slave_id, uint8_t function, uint16_t address, uint16_t argument, uint16_t *values) {
    uint8_t  tx_data[8] = {slave_id,
                           function,
                           (uint8_t)(address >> 8),
                           (uint8_t)address,
                           (uint8_t)(argument >> 8),
                           (uint8_t)argument,
                           0U,
                           0U};
    uint16_t rx_size    = 0U;
    if (bus == NULL || bus->exchange == NULL || bus->timeout_ms == 0U || slave_id == 0U || slave_id > 254U)
        return MODBUS_ERROR_ARGUMENT;
    if (bus->busy)
        return MODBUS_ERROR_BUSY;
    bus->busy           = 1U;
    bus->exception_code = 0U;
    uint16_t crc        = modbus_crc16(tx_data, 6U);
    tx_data[6]          = (uint8_t)crc;
    tx_data[7]          = (uint8_t)(crc >> 8);
    modbus_result_t result =
        bus->exchange(bus->context, tx_data, 8U, bus->rx_data, sizeof(bus->rx_data), &rx_size, bus->timeout_ms);
    if (result != MODBUS_OK)
        goto finish;
    if (rx_size < 5U || rx_size > sizeof(bus->rx_data)) {
        result = MODBUS_ERROR_FRAME;
        goto finish;
    }
    crc = modbus_crc16(bus->rx_data, rx_size - 2U);
    if (bus->rx_data[rx_size - 2U] != (uint8_t)crc || bus->rx_data[rx_size - 1U] != (uint8_t)(crc >> 8)) {
        result = MODBUS_ERROR_CRC;
        goto finish;
    }
    result = MODBUS_ERROR_FRAME;
    if (bus->rx_data[0] != slave_id)
        goto finish;
    if (bus->rx_data[1] == (uint8_t)(function | 0x80U)) {
        if (rx_size == 5U) {
            bus->exception_code = bus->rx_data[2];
            result              = MODBUS_ERROR_EXCEPTION;
        }
        goto finish;
    }
    if (bus->rx_data[1] != function)
        goto finish;
    if (function == 0x03U) {
        if (rx_size != 5U + 2U * argument || bus->rx_data[2] != 2U * argument)
            goto finish;
        for (uint16_t index = 0U; index < argument; ++index) {
            values[index] = (uint16_t)((uint16_t)bus->rx_data[3U + 2U * index] << 8) | bus->rx_data[4U + 2U * index];
        }
    } else if (rx_size != 8U || memcmp(tx_data, bus->rx_data, 8U) != 0) {
        goto finish;
    }
    result = MODBUS_OK;
finish:
    bus->busy = 0U;
    return result;
}

modbus_result_t
modbus_read_holding(modbus_handle_t *bus, uint8_t slave_id, uint16_t address, uint16_t count, uint16_t *values) {
    if (values == NULL || count == 0U || count > MODBUS_MAX_REGISTERS || (uint32_t)address + count > 65536U)
        return MODBUS_ERROR_ARGUMENT;
    return modbus_request(bus, slave_id, 0x03U, address, count, values);
}

modbus_result_t modbus_write_single(modbus_handle_t *bus, uint8_t slave_id, uint16_t address, uint16_t value) {
    return modbus_request(bus, slave_id, 0x06U, address, value, NULL);
}
