/**
 * @file GetMicroseconds.c
 * @brief Implementation of a microsecond timer.
 * @author Fedor Baklanov
 * @date 01 June 2022
 */

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

