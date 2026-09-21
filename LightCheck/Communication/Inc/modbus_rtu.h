#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#include <stddef.h>
#include <stdint.h>

#define MODBUS_MAX_REGISTERS  100U
#define MODBUS_MAX_FRAME_SIZE 205U

typedef enum modbus_result_e {
    MODBUS_OK = 0,
    MODBUS_ERROR_ARGUMENT,
    MODBUS_ERROR_BUSY,
    MODBUS_ERROR_TIMEOUT,
    MODBUS_ERROR_IO,
    MODBUS_ERROR_FRAME,
    MODBUS_ERROR_CRC,
    MODBUS_ERROR_EXCEPTION
} modbus_result_t;

/* exchange 必须返回完整帧；失败后必须恢复接收方向。 */
typedef modbus_result_t (*modbus_exchange_fn)(void          *context,
                                              const uint8_t *tx_data,
                                              uint16_t       tx_size,
                                              uint8_t       *rx_data,
                                              uint16_t       rx_capacity,
                                              uint16_t      *rx_size,
                                              uint32_t       timeout_ms);

typedef struct modbus_handle_s {
    void              *context;
    modbus_exchange_fn exchange;
    uint32_t           timeout_ms;
    uint8_t            exception_code;
    uint8_t            busy; /* 仅防止重入；RTOS 下调用者另加互斥锁。 */
    uint8_t            rx_data[MODBUS_MAX_FRAME_SIZE];
} modbus_handle_t;

modbus_result_t modbus_init(modbus_handle_t *bus, void *context, modbus_exchange_fn exchange, uint32_t timeout_ms);
uint16_t        modbus_crc16(const uint8_t *data, size_t size);
modbus_result_t
modbus_read_holding(modbus_handle_t *bus, uint8_t slave_id, uint16_t address, uint16_t count, uint16_t *values);
modbus_result_t modbus_write_single(modbus_handle_t *bus, uint8_t slave_id, uint16_t address, uint16_t value);

#endif
