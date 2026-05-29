/// @file CSpiHandler.cpp
/// @brief Implementation of the SPI handler software component class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CSpiHandler.h"
#include "AmsAssert.h"

CSpiHandler& CSpiHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  static CSpiHandler soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

void CSpiHandler::Init()
{
  if (false == bIsInitialized_)
  {
    bIsInitialized_ = oMutexSpi4_.Create();
  }
  if (true == bIsInitialized_)
  {
	  bIsInitialized_ = oMutexSpi1_.Create();
  }
}

bool CSpiHandler::IsInitialized()
{
  return bIsInitialized_;
}

bool CSpiHandler::Transmit(EPeripheral ePeripheral, const uint8_t* uRegAddr, uint8_t uRegAddrLen, const uint8_t* upData, uint16_t uSize, uint32_t uTimeout)
{
  bool bStatus{false};

  switch (ePeripheral)
  {
    case EPeripheral::eBmi270:
      bStatus = transmitInternal(&hspi4, SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, uRegAddr, uRegAddrLen, upData, uSize, uTimeout);
      break;
    case EPeripheral::eBmp384:
      bStatus = transmitInternal(&hspi4, SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, uRegAddr, uRegAddrLen, upData, uSize, uTimeout);
      break;
    case EPeripheral::eAsm330:
      bStatus = transmitInternal(&hspi1, SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, uRegAddr, uRegAddrLen, upData, uSize, uTimeout);
      break;
    case EPeripheral::eLps22:
      bStatus = transmitInternal(&hspi1, SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, uRegAddr, uRegAddrLen, upData, uSize, uTimeout);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }

  return bStatus;
}

bool CSpiHandler::Receive(EPeripheral ePeripheral, const uint8_t* uRegAddr, uint8_t uRegAddrLen, uint8_t* upData, uint16_t uSize, uint32_t uTimeout)
{
  bool bStatus{false};

  switch (ePeripheral)
  {
    case EPeripheral::eBmi270:
      bStatus = receiveInternal(&hspi4, SPI4_CS0_GPIO_Port, SPI4_CS0_Pin, uRegAddr, uRegAddrLen, upData, uSize, uTimeout);
      break;
    case EPeripheral::eBmp384:
      bStatus = receiveInternal(&hspi4, SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, uRegAddr, uRegAddrLen, upData, uSize, uTimeout);
      break;
    case EPeripheral::eAsm330:
      bStatus = receiveInternal(&hspi1, SPI1_CS0_GPIO_Port, SPI1_CS0_Pin, uRegAddr, uRegAddrLen, upData, uSize, uTimeout);
      break;
    case EPeripheral::eLps22:
      bStatus = receiveInternal(&hspi1, SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, uRegAddr, uRegAddrLen, upData, uSize, uTimeout);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }

  return bStatus;
}

bool CSpiHandler::transmitInternal(SPI_HandleTypeDef* opSpiHandle, GPIO_TypeDef* opGpioPort, uint16_t uChipSelectPin, const uint8_t* uRegAddr, uint8_t uRegAddrLen, const uint8_t* upData, uint16_t uSize, uint32_t uTimeout)
{
  bool bStatus{false};

  if (mutexAcquire(opSpiHandle))
  {
    HAL_GPIO_WritePin(opGpioPort, uChipSelectPin, GPIO_PIN_RESET);

    HAL_StatusTypeDef eStatus{HAL_SPI_Transmit(opSpiHandle, uRegAddr, uRegAddrLen, uTimeout)};
    if (HAL_OK == eStatus)
    {
      eStatus = HAL_SPI_Transmit(opSpiHandle, upData, uSize, uTimeout);
    }
    HAL_GPIO_WritePin(opGpioPort, uChipSelectPin, GPIO_PIN_SET);

    mutexRelease(opSpiHandle);
    bStatus = (HAL_OK == eStatus);
  }

  return bStatus;
}

bool CSpiHandler::receiveInternal(SPI_HandleTypeDef* opSpiHandle, GPIO_TypeDef* opGpioPort, uint16_t uChipSelectPin, const uint8_t* uRegAddr, uint8_t uRegAddrLen, uint8_t* upData, uint16_t uSize, uint32_t uTimeout)
{
  bool bStatus{false};

  if (mutexAcquire(opSpiHandle))
  {
    HAL_GPIO_WritePin(opGpioPort, uChipSelectPin, GPIO_PIN_RESET);

    HAL_StatusTypeDef eStatus{HAL_SPI_Transmit(opSpiHandle, uRegAddr, uRegAddrLen, uTimeout)};
    if (HAL_OK == eStatus)
    {
      eStatus = HAL_SPI_Receive(opSpiHandle, upData, uSize, uTimeout);
    }
    HAL_GPIO_WritePin(opGpioPort, uChipSelectPin, GPIO_PIN_SET);

    mutexRelease(opSpiHandle);
    bStatus = (HAL_OK == eStatus);
  }

  return bStatus;
}

bool CSpiHandler::mutexAcquire(const SPI_HandleTypeDef* opSpiHandle)
{
  bool bStatus{false};

  if (opSpiHandle == &hspi4)
  {
    bStatus = oMutexSpi4_.Acquire(skuMutexAcquisitionTimeout_);
  }
  else if (opSpiHandle == &hspi1)
  {
    bStatus = oMutexSpi1_.Acquire(skuMutexAcquisitionTimeout_);
  }
  else
  {
    // do nothing
  }

  return bStatus;
}

void CSpiHandler::mutexRelease(const SPI_HandleTypeDef* opSpiHandle)
{
  if (opSpiHandle == &hspi4)
  {
    oMutexSpi4_.Release();
  }
  else if (opSpiHandle == &hspi1)
  {
    oMutexSpi1_.Release();
  }
  else
  {
    // do nothing
  }
}

