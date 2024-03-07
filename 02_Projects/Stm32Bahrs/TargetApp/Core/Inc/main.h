/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#define BMP384_INT_Pin GPIO_PIN_14
#define BMP384_INT_GPIO_Port GPIOC
#define BMP384_CSB_Pin GPIO_PIN_0
#define BMP384_CSB_GPIO_Port GPIOC
#define MMC_MOSI_Pin GPIO_PIN_1
#define MMC_MOSI_GPIO_Port GPIOC
#define BMP384_MISO_Pin GPIO_PIN_2
#define BMP384_MISO_GPIO_Port GPIOC
#define BMP384_MOSI_Pin GPIO_PIN_3
#define BMP384_MOSI_GPIO_Port GPIOC
#define MMC_INT_Pin GPIO_PIN_0
#define MMC_INT_GPIO_Port GPIOA
#define BMP384_SHDN_Pin GPIO_PIN_1
#define BMP384_SHDN_GPIO_Port GPIOA
#define MURATA_RESET_DUE_Pin GPIO_PIN_4
#define MURATA_RESET_DUE_GPIO_Port GPIOA
#define MURATA_SCK_Pin GPIO_PIN_5
#define MURATA_SCK_GPIO_Port GPIOA
#define MURATA_MISO_Pin GPIO_PIN_6
#define MURATA_MISO_GPIO_Port GPIOA
#define MURATA_MOSI_Pin GPIO_PIN_7
#define MURATA_MOSI_GPIO_Port GPIOA
#define MURATA_CSB_UNO_Pin GPIO_PIN_4
#define MURATA_CSB_UNO_GPIO_Port GPIOC
#define MURATA_RESET_UNO_Pin GPIO_PIN_5
#define MURATA_RESET_UNO_GPIO_Port GPIOC
#define MURATA_CSB_DUE_Pin GPIO_PIN_0
#define MURATA_CSB_DUE_GPIO_Port GPIOB
#define MURATA_SHDN_Pin GPIO_PIN_1
#define MURATA_SHDN_GPIO_Port GPIOB
#define BMM2_SCL_Pin GPIO_PIN_10
#define BMM2_SCL_GPIO_Port GPIOB
#define RS232_SHDN_Pin GPIO_PIN_12
#define RS232_SHDN_GPIO_Port GPIOB
#define BMP384_SCK_Pin GPIO_PIN_13
#define BMP384_SCK_GPIO_Port GPIOB
#define BMM1_SCL_Pin GPIO_PIN_6
#define BMM1_SCL_GPIO_Port GPIOC
#define BMM1_SDA_Pin GPIO_PIN_7
#define BMM1_SDA_GPIO_Port GPIOC
#define NVM_WRITE_CONTROL_Pin GPIO_PIN_8
#define NVM_WRITE_CONTROL_GPIO_Port GPIOC
#define RS232_TX_Pin GPIO_PIN_9
#define RS232_TX_GPIO_Port GPIOA
#define RS232_RX_Pin GPIO_PIN_10
#define RS232_RX_GPIO_Port GPIOA
#define MMC_SHDN_Pin GPIO_PIN_12
#define MMC_SHDN_GPIO_Port GPIOA
#define MMC_CSB_Pin GPIO_PIN_15
#define MMC_CSB_GPIO_Port GPIOA
#define MMC_SCK_Pin GPIO_PIN_10
#define MMC_SCK_GPIO_Port GPIOC
#define MMC_MISO_Pin GPIO_PIN_11
#define MMC_MISO_GPIO_Port GPIOC
#define BMM2_SDA_Pin GPIO_PIN_12
#define BMM2_SDA_GPIO_Port GPIOC
#define SYNC_INPUT_Pin GPIO_PIN_2
#define SYNC_INPUT_GPIO_Port GPIOD
#define SYNC_INPUT_EXTI_IRQn EXTI2_IRQn
#define SYNC_ENABLE_Pin GPIO_PIN_4
#define SYNC_ENABLE_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_5
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_6
#define CAN_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
