#ifndef MAIN_H
#define MAIN_H
#include "stm32f4xx_hal.h"
/* External half-duplex transceiver: DE and /RE tied together. */
#define RS485_RE_GPIO_Port GPIOB
#define RS485_RE_Pin GPIO_PIN_1
#define MOTOR_LIMIT_GPIO_Port GPIOC
#define MOTOR1_LIMIT_Pin GPIO_PIN_0
#define MOTOR2_LIMIT_Pin GPIO_PIN_1
void Error_Handler(void);
#endif
