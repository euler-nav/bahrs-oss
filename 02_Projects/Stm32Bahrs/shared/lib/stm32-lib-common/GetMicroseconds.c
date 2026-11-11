/// @file GetMicroseconds.c
/// @brief Implementation of a microsecond timer.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "stm32f446xx.h"

uint64_t GetMicroseconds()
{
  uint32_t uHigh1, uHigh2, uLow;

  do
  {
    uHigh1 = TIM5->CNT;
    uLow   = TIM2->CNT;
    uHigh2 = TIM5->CNT;
  } while (uHigh1 != uHigh2);

  return ((uint64_t)uHigh1 << 32) | uLow;
}

