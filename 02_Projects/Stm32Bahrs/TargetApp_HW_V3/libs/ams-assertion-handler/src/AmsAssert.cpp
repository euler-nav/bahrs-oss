/**
 * @file AmsAssert.cpp
 * @brief Implementattion of project-specific assert function.
 * @author Fedor Baklanov
 * @date 2 August 2024
 * @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#include "stm32f4xx_hal.h"
#include <string>

#ifdef BAHRS_HW_V2

extern UART_HandleTypeDef huart1;
static UART_HandleTypeDef& orUartForAssertions{huart1};

#elif defined(BAHRS_HW_V3)

extern UART_HandleTypeDef huart2;
static UART_HandleTypeDef& orUartForAssertions{huart2};

#else
#error Unsupported hardware!
#endif

static void UartSendString(UART_HandleTypeDef *huart, const char* kcpStr);

extern "C" void AmsHardAssertFunc(const char* kcpFunction, int32_t iLine)
{
  __disable_irq();

  HAL_UART_Abort(&orUartForAssertions);

  UartSendString(&orUartForAssertions, "\n\nASSERTION FAILED: function ");
  UartSendString(&orUartForAssertions, kcpFunction);
  UartSendString(&orUartForAssertions, ", line ");
  UartSendString(&orUartForAssertions, std::to_string(iLine).c_str());
  UartSendString(&orUartForAssertions, ".\n");

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
