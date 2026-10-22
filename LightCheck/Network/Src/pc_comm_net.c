#include "network_config.h"
#if PC_USE_ETHERNET
#include "pc_comm.h"
#include "network.h"
/* Network_Init is bound to the controller explicitly in main. */
HAL_StatusTypeDef PC_Comm_Init(UART_HandleTypeDef *uart,pc_protocol_t *protocol)
{(void)uart;(void)protocol;return HAL_OK;}
HAL_StatusTypeDef PC_Comm_MspInit(UART_HandleTypeDef *uart) {(void)uart;return HAL_OK;}
void PC_Comm_MspDeInit(UART_HandleTypeDef *uart) {(void)uart;}
void PC_Comm_DMA_IRQHandler(void) {}
void PC_Comm_RxEvent(UART_HandleTypeDef *uart,uint16_t size) {(void)uart;(void)size;}
void PC_Comm_Error(UART_HandleTypeDef *uart) {(void)uart;}
void PC_Comm_Task(void) {Network_Task();}
HAL_StatusTypeDef PC_Comm_SendResponseData(uint8_t id,uint8_t command,pc_status_t status,
    uint16_t parameter,const uint8_t *data,uint8_t length)
{
    uint8_t frame[PC_PROTOCOL_MAX_FRAME];
    uint16_t size=PC_Protocol_BuildResponseData(frame,sizeof(frame),id,command,status,parameter,data,length);
    return size ? Network_Send(frame,size) : HAL_ERROR;
}
HAL_StatusTypeDef PC_Comm_SendResponse(uint8_t id,uint8_t command,pc_status_t status,uint16_t parameter,int16_t value)
{
    uint8_t data[2]={(uint8_t)((uint16_t)value>>8),(uint8_t)value};
    return PC_Comm_SendResponseData(id,command,status,parameter,data,2);
}
#endif
