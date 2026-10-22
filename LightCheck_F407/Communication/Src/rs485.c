#include "rs485.h"
#include "motor_config.h"
#include <string.h>

static void rs485_set_receive(rs485_handle_t *port) {
    HAL_GPIO_WritePin(port->dir_port, port->dir_pin, port->tx_level == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

modbus_result_t rs485_init(
    rs485_handle_t *port, UART_HandleTypeDef *huart, GPIO_TypeDef *dir_port, uint16_t dir_pin, GPIO_PinState tx_level) {
    if (port == NULL)
        return MODBUS_ERROR_ARGUMENT;
    memset(port, 0, sizeof(*port));
    if (huart == NULL || dir_port == NULL || dir_pin == 0U || (dir_pin & (dir_pin - 1U)) != 0U ||
        (tx_level != GPIO_PIN_RESET && tx_level != GPIO_PIN_SET) || huart->Init.BaudRate != MOTOR_RS485_BAUD ||
        huart->Init.WordLength != UART_WORDLENGTH_8B || huart->Init.StopBits != UART_STOPBITS_2 ||
        huart->Init.Parity != UART_PARITY_NONE)
        return MODBUS_ERROR_ARGUMENT;
    port->huart        = huart;
    port->dir_port     = dir_port;
    port->dir_pin      = dir_pin;
    port->tx_level     = tx_level;
    /* 8N2: 11 bits/character; >=3.5 characters, and >=1.75ms above 19200.
       Millisecond HAL timers require rounding up. */
    port->frame_gap_ms = (38500U + MOTOR_RS485_BAUD - 1U) / MOTOR_RS485_BAUD;
    if (port->frame_gap_ms < 2U) port->frame_gap_ms = 2U;
    rs485_set_receive(port);
    return MODBUS_OK;
}

modbus_result_t rs485_exchange(void          *context,
                               const uint8_t *tx_data,
                               uint16_t       tx_size,
                               uint8_t       *rx_data,
                               uint16_t       rx_capacity,
                               uint16_t      *rx_size,
                               uint32_t       timeout_ms) {
    rs485_handle_t *port = context;
    if (port == NULL || port->huart == NULL || tx_data == NULL || tx_size == 0U || rx_data == NULL ||
        rx_capacity == 0U || rx_size == NULL || timeout_ms <= port->frame_gap_ms)
        return MODBUS_ERROR_ARGUMENT;
    *rx_size                 = 0U;
    uint32_t        start_ms = HAL_GetTick();
    uint8_t         byte;
    modbus_result_t result = MODBUS_ERROR_TIMEOUT;
    rs485_set_receive(port);
    /* F4 clears PE/FE/NE/ORE by reading SR then DR. */
    __HAL_UART_CLEAR_OREFLAG(port->huart);
    /* 丢弃残留帧，直到实际观察到静默；持续噪声也受总超时限制。 */
    uint32_t elapsed_ms;
    while ((elapsed_ms = HAL_GetTick() - start_ms) < timeout_ms) {
        uint32_t remaining_ms = timeout_ms - elapsed_ms;
        if (remaining_ms <= port->frame_gap_ms)
            goto finish;
        HAL_StatusTypeDef status = HAL_UART_Receive(port->huart, &byte, 1U, port->frame_gap_ms);
        if (status == HAL_TIMEOUT)
            break;
        if (status != HAL_OK) {
            result = MODBUS_ERROR_IO;
            goto finish;
        }
    }
    elapsed_ms = HAL_GetTick() - start_ms;
    if (elapsed_ms >= timeout_ms)
        goto finish;
    HAL_GPIO_WritePin(port->dir_port, port->dir_pin, port->tx_level);
    /* 本项目阻塞 HAL_UART_Transmit 在返回 HAL_OK 前等待 TC。 */
    HAL_StatusTypeDef status = HAL_UART_Transmit(port->huart, (uint8_t *)tx_data, tx_size, timeout_ms - elapsed_ms);
    if (status == HAL_OK) {
        while (__HAL_UART_GET_FLAG(port->huart, UART_FLAG_TC) == RESET &&
               HAL_GetTick() - start_ms < timeout_ms) {}
        if (__HAL_UART_GET_FLAG(port->huart, UART_FLAG_TC) == RESET) status = HAL_TIMEOUT;
    }
    rs485_set_receive(port);
    if (status != HAL_OK) {
        result = status == HAL_TIMEOUT ? MODBUS_ERROR_TIMEOUT : MODBUS_ERROR_IO;
        goto finish;
    }
    while ((elapsed_ms = HAL_GetTick() - start_ms) < timeout_ms) {
        uint32_t remaining_ms = timeout_ms - elapsed_ms;
        uint32_t wait_ms      = *rx_size == 0U ? remaining_ms : port->frame_gap_ms;
        if (wait_ms > remaining_ms)
            wait_ms = remaining_ms;
        status = HAL_UART_Receive(port->huart, &byte, 1U, wait_ms);
        if (status == HAL_TIMEOUT) {
            if (*rx_size != 0U && wait_ms == port->frame_gap_ms)
                result = MODBUS_OK;
            goto finish;
        }
        if (status != HAL_OK) {
            result = MODBUS_ERROR_IO;
            goto finish;
        }
        if (*rx_size >= rx_capacity) {
            result = MODBUS_ERROR_FRAME;
            goto finish;
        }
        rx_data[(*rx_size)++] = byte;
    }
finish:
    rs485_set_receive(port);
    return result;
}
