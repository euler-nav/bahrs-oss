/// @file CI2CHandler.cpp
/// @brief Implementation of the I2C handler software component class.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CI2CHandler.h"
#include "AmsAssert.h"
#include "FreeRTOS.h"
#include "i2c.h"

CI2CHandler& CI2CHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  static CI2CHandler soInstance{};
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

void CI2CHandler::Init()
{
  bool bStatus{true};

  for (SBusState& orBusState : aoBusStates_)
  {
    bStatus = bStatus && orBusState.oMutex.Create();

    if (false == bStatus)
    {
      break;
    }
  }

  bIsInitialized_ = bStatus;
}

bool CI2CHandler::IsInitialized()
{
  return bIsInitialized_;
}

bool CI2CHandler::MemRead(EBus eBus, uint16_t uDevAddr, uint16_t uMemAddr, uint16_t uMemAddrSize,
                          uint8_t* upData, uint16_t uSize, uint32_t uTimeout)
{
  AMS_HARD_ASSERT(0 == xPortIsInsideInterrupt());
  bool bStatus{false};

  if ((nullptr != upData) && bIsInitialized_)
  {
    bStatus = true;
  }

  if (bStatus)
  {
    HAL_StatusTypeDef eStatus{HAL_BUSY};

    bStatus = false; // Assume failure until the read operation succeeds.

    if (mutexAcquire(eBus))
    {
      I2C_HandleTypeDef* opHandle{getI2CHandle(eBus)};

      if (nullptr != opHandle)
      {
        if (i2cWaitOnFlag(opHandle, I2C_FLAG_BUSY, SET))
        {
          // HAL performs its own BUSY check; we already yielded while waiting.
          eStatus = HAL_I2C_Mem_Read(opHandle, uDevAddr, uMemAddr, uMemAddrSize, upData, uSize, uTimeout);
        }

        bStatus = (HAL_OK == eStatus);
      }

      updateFailureState(eBus, eStatus);
      mutexRelease(eBus);
    }
  }

  return bStatus;
}

bool CI2CHandler::MemWrite(EBus eBus, uint16_t uDevAddr, uint16_t uMemAddr, uint16_t uMemAddrSize,
                           const uint8_t* upData, uint16_t uSize, uint32_t uTimeout)
{
  AMS_HARD_ASSERT(0 == xPortIsInsideInterrupt());
  bool bStatus{false};

  if ((nullptr != upData) && bIsInitialized_)
  {
    bStatus = true;
  }

  if (bStatus)
  {
    HAL_StatusTypeDef eStatus{HAL_BUSY};

    bStatus = false; // Assume failure until the write operation succeeds.

    if (mutexAcquire(eBus))
    {
      I2C_HandleTypeDef* opHandle{getI2CHandle(eBus)};

      if (nullptr != opHandle)
      {
        if (i2cWaitOnFlag(opHandle, I2C_FLAG_BUSY, SET))
        {
          // HAL performs its own BUSY check; we already yielded while waiting.
          eStatus = HAL_I2C_Mem_Write(opHandle, uDevAddr, uMemAddr, uMemAddrSize, const_cast<uint8_t*>(upData), uSize, uTimeout);
        }

        bStatus = (HAL_OK == eStatus);
      }

      updateFailureState(eBus, eStatus);
      mutexRelease(eBus);
    }
  }

  return bStatus;
}

bool CI2CHandler::mutexAcquire(EBus eBus)
{
  return getBusState(eBus).oMutex.Acquire(skuMutexAcquisitionTimeout_);
}

void CI2CHandler::mutexRelease(EBus eBus)
{
  getBusState(eBus).oMutex.Release();
}

bool CI2CHandler::i2cWaitOnFlag(I2C_HandleTypeDef* opHandle,
                                uint32_t uFlag,
                                FlagStatus eStatus)
{
  bool bResult{true};
  const uint32_t kuTickStart{HAL_GetTick()};

  while (__HAL_I2C_GET_FLAG(opHandle, uFlag) == eStatus)
  {
    if (HAL_MAX_DELAY != skuWaitOnFlagTimeout_)
    {
      if (((HAL_GetTick() - kuTickStart) > skuWaitOnFlagTimeout_) || (0U == skuWaitOnFlagTimeout_))
      {
        bResult = false;
        break;
      }
    }

    osDelay(1U);
  }

  return bResult;
}

void CI2CHandler::updateFailureState(EBus eBus, HAL_StatusTypeDef eStatus)
{
  SBusState& orBusState{getBusState(eBus)};

  if (HAL_OK == eStatus)
  {
    orBusState.uConsecutiveFailures_ = 0U;
  }
  else if ((HAL_BUSY == eStatus) || (HAL_ERROR == eStatus))
  {
    if (orBusState.uConsecutiveFailures_ < skuFailureThreshold_)
    {
      ++orBusState.uConsecutiveFailures_;
    }
  }

  if ((orBusState.uConsecutiveFailures_ >= skuFailureThreshold_) && (false == orBusState.bDemoted_))
  {
    demoteTasks();
    orBusState.bDemoted_ = true;
  }
}

void CI2CHandler::demoteTasks()
{
  const osThreadId_t pTaskHandle{osThreadGetId()};

  if (nullptr != pTaskHandle)
  {
    (void)osThreadSetPriority(pTaskHandle, skeDemotedPriority_);
  }
}

CI2CHandler::SBusState& CI2CHandler::getBusState(EBus eBus)
{
  const uint8_t kuIndex{static_cast<uint8_t>(eBus)};
  AMS_HARD_ASSERT(kuIndex < aoBusStates_.size());
  return aoBusStates_[kuIndex];
}

const CI2CHandler::SBusState& CI2CHandler::getBusState(EBus eBus) const
{
  const uint8_t kuIndex{static_cast<uint8_t>(eBus)};
  AMS_HARD_ASSERT(kuIndex < aoBusStates_.size());
  return aoBusStates_[kuIndex];
}

I2C_HandleTypeDef* CI2CHandler::getI2CHandle(EBus eBus)
{
  I2C_HandleTypeDef* opHandle{nullptr};

  switch (eBus)
  {
    case EBus::eI2c1:
      opHandle = &hi2c1;
      break;
    case EBus::eI2c2:
      opHandle = &hi2c2;
      break;
    case EBus::eI2c3:
      opHandle = &hi2c3;
      break;
    default:
      break;
  }

  return opHandle;
}
