/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"
#include "crc.h"
#include "dma.h"
#include "fmpi2c.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CRs232OutputHandlerWrapper.h"
#include "AmsAssert.h"
#include "GetMicroseconds.h"
#include "SyncPulseHandlerCApi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern osSemaphoreId_t asm330lhhInt1SemHandle;

extern osMessageQueueId_t QueueTaskAsm330TimeStampHandle;
extern osMessageQueueId_t QueueTaskBmi270TimeStampHandle;
extern osMessageQueueId_t QueueTaskBmp384TimeStampHandle;
extern osMessageQueueId_t QueueTaskLps22TimeStampHandle;
extern osMessageQueueId_t QueueTaskIcp20100TimeStampHandle;
extern osMessageQueueId_t QueueTaskMmc5983TimestampHandle;
extern osMessageQueueId_t QueueTaskLis3TimestampHandle;
extern osMessageQueueId_t QueueTaskBmm350TimestampHandle;

extern DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

#ifdef ENABLE_PRINTF

int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

#endif // ENABLE_PRINTF

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

  // Ensure that TIM6 driving tick for HAL has priority higher than interrupts calling HAL functions.
  // The preceeding auto-generated call to HAL_Init() sets the tick priority to 15. With this call
  // we set it to 1. The subsequent call to SystemClock_Config() will call HAL_InitTick() again, but
  // it won't change the tick priority stored in the global variable uwTickPrio.
  HAL_InitTick(1);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_SPI4_Init();
  MX_CRC_Init();
  MX_SPI1_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_SPI2_Init();
  MX_FMPI2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim5);

  HAL_GPIO_WritePin(RS232_SHDN_GPIO_Port, RS232_SHDN_Pin, GPIO_PIN_SET);
  // Enable power (Power sequence)
  // Enable Sensor Group 1A power (ST IMU & Barometer)
  HAL_GPIO_WritePin(PWR_EN_G1A_GPIO_Port, PWR_EN_G1A_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_SET);

  // Enable Sensor Group 2 power (Bosch IMU & Barometer)
  HAL_GPIO_WritePin(PWR_EN_G2_GPIO_Port, PWR_EN_G2_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_SET);

  // Enable Sensor Group 1A power (Murata IMU & TDK Barometer & Memsic Magnetometer)
  HAL_GPIO_WritePin(PWR_EN_G3_GPIO_Port, PWR_EN_G3_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(SPI2_CS0_GPIO_Port, SPI2_CS0_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(SPI2_CS1_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_SET);

  // Enable Sensor Group 1B power (ST & Bosch Magnetometers)
  HAL_GPIO_WritePin(PWR_EN_G1B_GPIO_Port, PWR_EN_G1B_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);

  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    Rs232TxInterruptHandler(huart->TxXferSize);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == BMI270_INT1_Pin)
  {
    uint64_t uTimestamp = GetMicroseconds();
    osMessageQueuePut(QueueTaskBmi270TimeStampHandle, &uTimestamp, 0U, 0U);
  }
  if (GPIO_Pin == ASM330_INT1_Pin) // Accelerometer interrupt
  {
    osSemaphoreRelease(asm330lhhInt1SemHandle);
  }
  if (GPIO_Pin == ASM330_INT2_Pin) // Gyroscope interrupt
  {
    uint64_t uTimestamp = GetMicroseconds();
    osMessageQueuePut(QueueTaskAsm330TimeStampHandle, &uTimestamp, 0U, 0U);
  }
  if (GPIO_Pin == BMP384_INT_Pin)
  {
    uint64_t uTimestamp = GetMicroseconds();
    osMessageQueuePut(QueueTaskBmp384TimeStampHandle, &uTimestamp, 0U, 0U);
  }
  if (GPIO_Pin == LPS22_INT_Pin)
  {
    uint64_t uTimestamp = GetMicroseconds();
    osMessageQueuePut(QueueTaskLps22TimeStampHandle, &uTimestamp, 0U, 0U);
  }
  if (GPIO_Pin == ICP20100_INT_Pin)
  {
    uint64_t uTimestamp = GetMicroseconds();
    osMessageQueuePut(QueueTaskIcp20100TimeStampHandle, &uTimestamp, 0U, 0U);
  }
  if (GPIO_Pin == MMC5983_INT_Pin)
  {
    uint64_t uTimestamp = GetMicroseconds();
    osMessageQueuePut(QueueTaskMmc5983TimestampHandle, &uTimestamp, 0U, 0U);
  }
  if (GPIO_Pin == LIS3_DRDY_Pin)
  {
    uint64_t uTimestamp = GetMicroseconds();
    osMessageQueuePut(QueueTaskLis3TimestampHandle, &uTimestamp, 0U, 0U);
  }
  if (GPIO_Pin == BMM350_INT_Pin)
  {
    uint64_t uTimestamp = GetMicroseconds();
    osMessageQueuePut(QueueTaskBmm350TimestampHandle, &uTimestamp, 0U, 0U);
  }
  if (GPIO_Pin == SYNC_PULSE_IN_Pin)
  {
    SyncPulseCallback();
  }
}


/**
 * The function shall redirect assertions from std to custom assertion handler.
 */
void __assert_func(const char* kcpFile, int iLine, const char* kcpFunction, const char* kcpExpression)
{
  AmsHardAssertFunc(kcpFunction, iLine);
}

/**
 * The function shall redirect assertions from std to custom assertion handler.
 */
void __assert(const char* kcpFile, int iLine, const char* kcpFunction)
{
  AmsHardAssertFunc(kcpFunction, iLine);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
