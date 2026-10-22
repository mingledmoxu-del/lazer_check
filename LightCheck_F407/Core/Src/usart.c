#include "usart.h"
#include "pc_comm.h"
#include "motor_config.h"
UART_HandleTypeDef huart1, huart2;
static void init_uart(UART_HandleTypeDef *uart, USART_TypeDef *instance,
                      uint32_t baud, uint32_t stop_bits)
{
    uart->Instance = instance;
    uart->Init.BaudRate = baud;
    uart->Init.WordLength = UART_WORDLENGTH_8B;
    uart->Init.StopBits = stop_bits;
    uart->Init.Parity = UART_PARITY_NONE;
    uart->Init.Mode = UART_MODE_TX_RX;
    uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart->Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(uart) != HAL_OK) Error_Handler();
}
void MX_USART1_UART_Init(void) { init_uart(&huart1, USART1, 115200U, UART_STOPBITS_1); }
void MX_USART2_UART_Init(void) { init_uart(&huart2, USART2, MOTOR_RS485_BAUD, UART_STOPBITS_2); }
void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    if (uart->Instance == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
        gpio.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &gpio);
        if (PC_Comm_MspInit(uart) != HAL_OK) Error_Handler();
    } else if (uart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        gpio.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}
void HAL_UART_MspDeInit(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART1) {
        PC_Comm_MspDeInit(uart);
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
    } else if (uart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    }
}
