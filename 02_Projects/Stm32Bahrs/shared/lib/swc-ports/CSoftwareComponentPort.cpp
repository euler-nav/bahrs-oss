/// @file CSoftwareComponentPort.cpp
/// @brief Implementation of software component port functions.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CSoftwareComponentPort.h"

#ifndef _MSC_VER

CSoftwareComponentPortBase::CSoftwareComponentPortBase()
{
  pMutexHandle_ = osMutexNew(&sMutexAttributes_);

  if (NULL == pMutexHandle_)
  {
    AMS_HARD_ASSERT(false);
  }
}

CSoftwareComponentPortBase::~CSoftwareComponentPortBase()
{
  osMutexDelete(pMutexHandle_);
}

bool CSoftwareComponentPortBase::Lock()
{
  bool bRetVal{false};

  osStatus_t eStatus = osMutexAcquire(pMutexHandle_, skuPortLockTimeout);

  if (osOK == eStatus)
  {
    bRetVal = true;
  }

  return bRetVal;
}

void CSoftwareComponentPortBase::Unlock()
{
  osMutexRelease(pMutexHandle_);
}

#else

CSoftwareComponentPortBase::CSoftwareComponentPortBase()
{
}

CSoftwareComponentPortBase::~CSoftwareComponentPortBase()
{
}

bool CSoftwareComponentPortBase::Lock()
{
  return true;
}

void CSoftwareComponentPortBase::Unlock()
{
}

#endif // _MSC_VER
