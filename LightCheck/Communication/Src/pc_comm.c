#include "network_config.h"
#if !PC_USE_ETHERNET
#include "pc_comm.h"
#include <string.h>

#define PC_DMA_SIZE 256U
static UART_HandleTypeDef *g_uart; static pc_protocol_t *g_protocol;
static DMA_HandleTypeDef g_hdma_usart1_rx;
/* DMA1 无法访问 DTCM；显式放入 D2 SRAM，并按缓存行对齐便于以后启用 D-Cache。 */
static uint8_t g_dma[PC_DMA_SIZE] __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t g_ring[PC_PROTOCOL_BUFFER_SIZE];
static volatile uint16_t g_head, g_tail;
static volatile uint8_t g_restart_pending;

HAL_StatusTypeDef PC_Comm_MspInit(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart->Instance != USART1) return HAL_ERROR;
    __HAL_RCC_DMA1_CLK_ENABLE();
    g_hdma_usart1_rx.Instance = DMA1_Stream0;
    g_hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    g_hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    g_hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    g_hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    g_hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    g_hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    g_hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    g_hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    g_hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&g_hdma_usart1_rx) != HAL_OK) return HAL_ERROR;
    __HAL_LINKDMA(uart, hdmarx, g_hdma_usart1_rx);
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    return HAL_OK;
}

void PC_Comm_MspDeInit(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart->Instance != USART1) return;
    HAL_DMA_DeInit(uart->hdmarx);
    HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(USART1_IRQn);
}

void PC_Comm_DMA_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_hdma_usart1_rx);
}

static HAL_StatusTypeDef start_receive(void)
{
    HAL_StatusTypeDef status;
    if (g_uart == NULL || g_uart->hdmarx == NULL) return HAL_ERROR;
    status = HAL_UARTEx_ReceiveToIdle_DMA(g_uart, g_dma, sizeof(g_dma));
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(g_uart->hdmarx, DMA_IT_HT);
        g_restart_pending = 0U;
    } else {
        g_restart_pending = 1U;
    }
    return status;
}

HAL_StatusTypeDef PC_Comm_Init(UART_HandleTypeDef *uart, pc_protocol_t *protocol)
{
    if (uart == NULL || protocol == NULL) return HAL_ERROR;
    g_uart=uart; g_protocol=protocol; g_head=0U; g_tail=0U; g_restart_pending=0U;
    return start_receive();
}

void PC_Comm_RxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
    if (uart != g_uart || size > PC_DMA_SIZE) return;
    for (uint16_t i=0U; i<size; ++i) {
        uint16_t next=(uint16_t)((g_head+1U)%PC_PROTOCOL_BUFFER_SIZE);
        if (next == g_tail) break;
        g_ring[g_head]=g_dma[i]; g_head=next;
    }
    (void)start_receive();
}

void PC_Comm_Error(UART_HandleTypeDef *uart)
{
    if (uart == g_uart) (void)start_receive();
}

void PC_Comm_Task(void)
{
    uint8_t local[64]; uint16_t n=0U;
    if (g_restart_pending != 0U) (void)start_receive();
    while (g_tail != g_head && n < sizeof(local)) { local[n++]=g_ring[g_tail]; g_tail=(uint16_t)((g_tail+1U)%PC_PROTOCOL_BUFFER_SIZE); }
    if (n != 0U) PC_Protocol_Input(g_protocol, local, n);
}

HAL_StatusTypeDef PC_Comm_SendResponse(uint8_t device_id, uint8_t command, pc_status_t status, uint16_t parameter, int16_t value)
{
    uint8_t frame[PC_PROTOCOL_MAX_FRAME]; uint16_t size=PC_Protocol_BuildResponse(frame,sizeof(frame),device_id,command,status,parameter,value);
    return size == 0U ? HAL_ERROR : HAL_UART_Transmit(g_uart,frame,size,100U);
}

HAL_StatusTypeDef PC_Comm_SendResponseData(uint8_t device_id, uint8_t command, pc_status_t status,
                                           uint16_t parameter, const uint8_t *data, uint8_t data_length)
{
    uint8_t frame[PC_PROTOCOL_MAX_FRAME];
    uint16_t size=PC_Protocol_BuildResponseData(frame,sizeof(frame),device_id,command,status,parameter,data,data_length);
    return size == 0U ? HAL_ERROR : HAL_UART_Transmit(g_uart,frame,size,100U);
}
#endif
