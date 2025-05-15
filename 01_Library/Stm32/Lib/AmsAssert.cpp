/**
 * @file AmsAssert.cpp
 * @brief Implementattion of project-specific assert function.
 * @author Fedor Baklanov
 * @date 2 August 2024
 * @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#include "stm32f4xx_hal.h"
#include <string>

extern UART_HandleTypeDef huart1;

static void UartSendString(UART_HandleTypeDef *huart, const char* kcpStr);

extern "C" void AmsHardAssertFunc(const char* kcpFunction, int32_t iLine)
{
  __disable_irq();

  HAL_UART_Abort(&huart1);

  UartSendString(&huart1, "\n\nASSERTION FAILED: function ");
  UartSendString(&huart1, kcpFunction);
  UartSendString(&huart1, ", line ");
  UartSendString(&huart1, std::to_string(iLine).c_str());
  UartSendString(&huart1, ".\n");

  while (true)
  {
    // endless loop
  }
}

void UartSendString(UART_HandleTypeDef *huart, const char* kcpStr)
{
    while ((*kcpStr) != '\0')
    {
        HAL_UART_Transmit(huart, reinterpret_cast<const uint8_t*>(kcpStr), 1, 100U);
        ++kcpStr;
    }
}
