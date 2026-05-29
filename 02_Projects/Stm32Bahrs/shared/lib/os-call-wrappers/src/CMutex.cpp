/// @file CMutex.cpp
/// @brief Implementation of a mutex wrapper class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CMutex.h"

bool CMutex::Create()
{
  if (false == bIsInitialized_)
  {
    pMutexHandle_ = osMutexNew(&oMutexAttributes_);

    if (nullptr != pMutexHandle_)
    {
      bIsInitialized_ = true;
    }
  }

  return bIsInitialized_;
}

bool CMutex::Acquire(uint32_t uTimeoutInOsTicks)
{
  bool bStatus{false};

  if (bIsInitialized_)
  {
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, uTimeoutInOsTicks);

    if (osOK == eStatus)
    {
      bStatus = true;
    }
  }

  return bStatus;
}

void CMutex::Release()
{
  if (bIsInitialized_)
  {
    osMutexRelease(pMutexHandle_);
  }
}

CMutex::~CMutex()
{
  if (bIsInitialized_)
  {
    // Destruction after kernel stop may return error, ignore it.
    osMutexDelete(pMutexHandle_);
    pMutexHandle_ = nullptr;
    bIsInitialized_ = false;
  }
}
