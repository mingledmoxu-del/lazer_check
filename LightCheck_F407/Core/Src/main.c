/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rs485.h"
#include "p100s.h"
#include "pc_protocol.h"
#include "pc_comm.h"
#include "command_manager.h"
#include "limit_debug.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 按原理图确认 PC12 高电平发送、低电平接收。 */
#define P100S_RS485_TX_LEVEL GPIO_PIN_SET
/* Temporary ASCII diagnostics on the PC UART; set to 0 for normal binary use. */
#define LIMIT_SERIAL_DEBUG 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static rs485_handle_t servo_port;
static modbus_handle_t servo_bus;
static p100s_handle_t servo_motors[2];
static pc_protocol_t pc_protocol;
static command_manager_t command_manager;
static volatile uint8_t limits_ready;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  
  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  if (rs485_init(&servo_port, &huart2, RS485_RE_GPIO_Port, RS485_RE_Pin,
                 P100S_RS485_TX_LEVEL) != MODBUS_OK ||
      modbus_init(&servo_bus, &servo_port, rs485_exchange, 100U) != MODBUS_OK ||
      p100s_init(&servo_motors[0], &servo_bus, 1U) != MODBUS_OK ||
      p100s_init(&servo_motors[1], &servo_bus, 2U) != MODBUS_OK) {
    Error_Handler();
  }
  Command_Manager_Init(&command_manager, &servo_motors[0], &servo_motors[1], &huart1);
  PC_Protocol_Init(&pc_protocol, Command_Process, &command_manager);
  if (PC_Comm_Init(&huart1, &pc_protocol) != HAL_OK) {
    Error_Handler();
  }
#if MOTOR_LIMITS_ENABLED && LIMIT_SERIAL_DEBUG
  LimitDebug_Init(&huart1);
#endif
  limits_ready = 1U;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    PC_Comm_Task();
    Command_Manager_Task(&command_manager);
#if MOTOR_LIMITS_ENABLED && LIMIT_SERIAL_DEBUG
    LimitDebug_Task();
#endif
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
/* HSI avoids assuming a particular board crystal. 16MHz /16 *336 /2 =168MHz. */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = 16;
    osc.PLL.PLLN = 336;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

/* USER CODE BEGIN 4 */
void Motor_Limits_Tick(void)
{
  if (limits_ready != 0U) {
    uint8_t raw=0;
    for (unsigned ch=0;ch<6;++ch)
      if (HAL_GPIO_ReadPin(MOTOR_LIMIT_GPIO_Port,(uint16_t)(GPIO_PIN_0<<ch))==GPIO_PIN_SET) raw|=(uint8_t)(1U<<ch);
    Command_Manager_SampleInputs(&command_manager,raw);
#if MOTOR_LIMITS_ENABLED
    uint8_t raw1 = (uint8_t)(HAL_GPIO_ReadPin(MOTOR_LIMIT_GPIO_Port, MOTOR1_LIMIT_Pin) == GPIO_PIN_SET);
    uint8_t raw2 = (uint8_t)(HAL_GPIO_ReadPin(MOTOR_LIMIT_GPIO_Port, MOTOR2_LIMIT_Pin) == GPIO_PIN_SET);
    Command_Manager_SampleLimits(&command_manager, raw1, raw2);
#if LIMIT_SERIAL_DEBUG
    LimitDebug_Sample(0U, raw1, &command_manager.limits[0]);
    LimitDebug_Sample(1U, raw2, &command_manager.limits[1]);
#endif
#endif
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  PC_Comm_RxEvent(huart, Size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  PC_Comm_Error(huart);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
