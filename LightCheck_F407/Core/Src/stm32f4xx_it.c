#include "main.h"
#include "usart.h"
#include "pc_comm.h"
extern void Motor_Limits_Tick(void);
void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}
void SysTick_Handler(void) { HAL_IncTick(); Motor_Limits_Tick(); }
void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void DMA2_Stream2_IRQHandler(void) { PC_Comm_DMA_IRQHandler(); }
