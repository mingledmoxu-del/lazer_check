#ifndef RS485_H
#define RS485_H

#include "stm32f4xx_hal.h"
#include "modbus_rtu.h"

typedef struct rs485_handle_s {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef       *dir_port;
    uint16_t            dir_pin;
    GPIO_PinState       tx_level;
    uint32_t            frame_gap_ms;
} rs485_handle_t;

/* GPIO/UART须先初始化。波特率由motor_config.h配置，8N2，帧边界按速率计算。 */
modbus_result_t rs485_init(
    rs485_handle_t *port, UART_HandleTypeDef *huart, GPIO_TypeDef *dir_port, uint16_t dir_pin, GPIO_PinState tx_level);
/* 阻塞方法：只能在主循环/线程调用，UART 必须独占。 */
modbus_result_t rs485_exchange(void          *context,
                               const uint8_t *tx_data,
                               uint16_t       tx_size,
                               uint8_t       *rx_data,
                               uint16_t       rx_capacity,
                               uint16_t      *rx_size,
                               uint32_t       timeout_ms);

#endif
