/// @file CStartupTaskBarrier.cpp
/// @brief Implementation of the startup task barrier.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CStartupTaskBarrier.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <array>
#include "AmsAssert.h"
#include <stdint.h>

static constexpr uint32_t skuCapacity_{32U};
using CArrayOfThreadIds = std::array<osThreadId_t, skuCapacity_>;

namespace
{

CArrayOfThreadIds soThreads_{};

} // namespace


bool CStartupTaskBarrier::suspend()
{
  // Per contract the init task has the highest priority, while suspend() should be called
  // in the beginning of the init. Therefore we do not need to lock kernel here.

  const osThreadId_t kopInitTask{osThreadGetId()};
  const osThreadId_t kopIdleTask{xTaskGetIdleTaskHandle()};
  const uint32_t kuThreadCount{osThreadGetCount()};

  AMS_HARD_ASSERT(skuCapacity_ >= kuThreadCount);

  bool bStatus{(nullptr != kopInitTask) && (nullptr != kopIdleTask) &&
               (3U <= kuThreadCount) && (skuCapacity_ >= kuThreadCount)};

  // Get all the thread handles
  if (bStatus)
  {
    // The bundled CMSIS adapter can return a nonzero count on allocation failure.
    soThreads_.fill(nullptr);
    bStatus = (kuThreadCount == osThreadEnumerate(soThreads_.data(), skuCapacity_));
  }

  // Check all the thread handles are valid
  if (bStatus)
  {
    for (uint32_t uIndex{0U}; (uIndex < kuThreadCount) && bStatus; ++uIndex)
    {
      bStatus = (nullptr != soThreads_[uIndex]);
    }
  }

  if (bStatus)
  {
    for (osThreadId_t opThread : soThreads_)
    {
      if ((nullptr != opThread) && (kopInitTask != opThread) && (kopIdleTask != opThread))
      {
        bStatus &= (osOK == osThreadSuspend(opThread));
      }
    }
  }

  return bStatus;
}

bool CStartupTaskBarrier::resume()
{
  // We need to lock kernel since initialization of software components may have changed task priorities
  // by the time resume() is called.

  bool bStatus{(osKernelLock() >= 0)};

  if (bStatus)
  {
    for (osThreadId_t opThread : soThreads_)
    {
      if (nullptr != opThread)
      {
        bStatus &= (osOK == osThreadResume(opThread));
      }
    }
  }

  bStatus &= (osKernelUnlock() >= 0);

  return bStatus;
}
