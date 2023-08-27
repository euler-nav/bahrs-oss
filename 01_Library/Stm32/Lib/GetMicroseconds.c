/**
 * @file GetMicroseconds.c
 * @brief Implementation of a microsecond timer.
 * @author Fedor Baklanov
 * @date 01 June 2022
 */

#include "stm32f446xx.h"

uint64_t GetMicroseconds()
{
  uint64_t uRetVal = 0;
  uint64_t uTim5Count = TIM5->CNT;
  uRetVal = (uTim5Count << 32) | TIM2->CNT;

  return uRetVal;
}

