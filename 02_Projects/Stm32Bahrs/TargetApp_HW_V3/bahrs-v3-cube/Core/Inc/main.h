/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MMC5983_INT_Pin GPIO_PIN_13
#define MMC5983_INT_GPIO_Port GPIOC
#define MMC5983_INT_EXTI_IRQn EXTI15_10_IRQn
#define RS232_SHDN_Pin GPIO_PIN_3
#define RS232_SHDN_GPIO_Port GPIOE
#define SPI2_CS2_Pin GPIO_PIN_2
#define SPI2_CS2_GPIO_Port GPIOE
#define SPI2_CS1_Pin GPIO_PIN_1
#define SPI2_CS1_GPIO_Port GPIOE
#define DBG_LED_Pin GPIO_PIN_0
#define DBG_LED_GPIO_Port GPIOE
#define RS232_RX_Pin GPIO_PIN_6
#define RS232_RX_GPIO_Port GPIOD
#define SYNC_EN_Pin GPIO_PIN_7
#define SYNC_EN_GPIO_Port GPIOD
#define SPI3_CS0_Pin GPIO_PIN_15
#define SPI3_CS0_GPIO_Port GPIOA
#define MURATA_RESET_DUE_Pin GPIO_PIN_14
#define MURATA_RESET_DUE_GPIO_Port GPIOC
#define MURATA_RESET_UNO_Pin GPIO_PIN_4
#define MURATA_RESET_UNO_GPIO_Port GPIOE
#define ICP20100_INT_Pin GPIO_PIN_5
#define ICP20100_INT_GPIO_Port GPIOE
#define ICP20100_INT_EXTI_IRQn EXTI9_5_IRQn
#define SPI2_CS0_Pin GPIO_PIN_9
#define SPI2_CS0_GPIO_Port GPIOB
#define RS232_TX_Pin GPIO_PIN_5
#define RS232_TX_GPIO_Port GPIOD
#define SYNC_PULSE_IN_Pin GPIO_PIN_11
#define SYNC_PULSE_IN_GPIO_Port GPIOC
#define SYNC_PULSE_IN_EXTI_IRQn EXTI15_10_IRQn
#define BMM350_INT_Pin GPIO_PIN_15
#define BMM350_INT_GPIO_Port GPIOC
#define BMM350_INT_EXTI_IRQn EXTI15_10_IRQn
#define CAN_SHDN_Pin GPIO_PIN_9
#define CAN_SHDN_GPIO_Port GPIOA
#define DBG_PF3_Pin GPIO_PIN_3
#define DBG_PF3_GPIO_Port GPIOF
#define LIS3_DRDY_Pin GPIO_PIN_9
#define LIS3_DRDY_GPIO_Port GPIOG
#define LIS3_DRDY_EXTI_IRQn EXTI9_5_IRQn
#define LIS3_INT_Pin GPIO_PIN_9
#define LIS3_INT_GPIO_Port GPIOF
#define PWR_EN_G3_Pin GPIO_PIN_0
#define PWR_EN_G3_GPIO_Port GPIOC
#define SPI4_CS0_Pin GPIO_PIN_11
#define SPI4_CS0_GPIO_Port GPIOE
#define PWR_EN_G1A_Pin GPIO_PIN_0
#define PWR_EN_G1A_GPIO_Port GPIOA
#define SPI1_CS0_Pin GPIO_PIN_4
#define SPI1_CS0_GPIO_Port GPIOA
#define ASM330_INT1_Pin GPIO_PIN_4
#define ASM330_INT1_GPIO_Port GPIOC
#define ASM330_INT1_EXTI_IRQn EXTI4_IRQn
#define PWR_EN_G2_Pin GPIO_PIN_1
#define PWR_EN_G2_GPIO_Port GPIOG
#define BMP384_INT_Pin GPIO_PIN_10
#define BMP384_INT_GPIO_Port GPIOE
#define BMP384_INT_EXTI_IRQn EXTI15_10_IRQn
#define BMI270_INT2_Pin GPIO_PIN_10
#define BMI270_INT2_GPIO_Port GPIOD
#define ASM330_INT2_Pin GPIO_PIN_1
#define ASM330_INT2_GPIO_Port GPIOA
#define ASM330_INT2_EXTI_IRQn EXTI1_IRQn
#define SPI4_CS1_Pin GPIO_PIN_13
#define SPI4_CS1_GPIO_Port GPIOF
#define BMI270_INT1_Pin GPIO_PIN_0
#define BMI270_INT1_GPIO_Port GPIOG
#define BMI270_INT1_EXTI_IRQn EXTI0_IRQn
#define PWR_EN_G1B_Pin GPIO_PIN_2
#define PWR_EN_G1B_GPIO_Port GPIOA
#define SPI1_CS1_Pin GPIO_PIN_12
#define SPI1_CS1_GPIO_Port GPIOF
#define NVM_WRITE_CONTROL_Pin GPIO_PIN_15
#define NVM_WRITE_CONTROL_GPIO_Port GPIOB
#define LPS22_INT_Pin GPIO_PIN_3
#define LPS22_INT_GPIO_Port GPIOA
#define LPS22_INT_EXTI_IRQn EXTI3_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
