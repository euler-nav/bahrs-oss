/// @file CIcp20100Driver.cpp
/// @brief Implementation of the ICP20100 driver software component class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifdef ENABLE_PRINTF
#include <stdio.h>
#endif // ENABLE_PRINTF

#include "CRte.h"
#include "CIcp20100Driver.h"
#include "AmsAssert.h"

CIcp20100Driver& CIcp20100Driver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CIcp20100Driver soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

bool CIcp20100Driver::IsInitialized()
{
  return bIsInitialized_;
}

void CIcp20100Driver::PollSensor(uint64_t uTimestampUs)
{
  bool bStatus{false};
  float fPressure{};
  float fTemperature{};
  SBarometerMeasurement oOutput{};
  uint8_t uFifoFillRegValue{};

  if (IsInitialized())
  {
    taskENTER_CRITICAL();
    NVIC_DisableIRQ(TIM7_IRQn);

    if (isWatermarkHighInterruptTriggered())
    {
      bStatus = readPressureAndTemperature(fPressure, fTemperature);

      // Check if the FIFO level is at 0
      if (readReg(skuIcp20100RegFifoFill_, &uFifoFillRegValue, 1))
      {
        if ((uFifoFillRegValue & skuIcp20100FifoEmpty_) != skuIcp20100FifoEmpty_)
        {
          bStatus = false;
          uFifoFillRegValue |= skuIcp20100FifoFlush_;
          writeReg(skuIcp20100RegFifoFill_, &uFifoFillRegValue, 1);
        }
      }

      watermarkHighInterruptClear();

      // Discard first 14 samples: ICP20100 FIR filter group delay causes invalid output at startup.
      if (uIgnoredSamplesCount_ < skuSamplesToIgnore_)
      {
        ++uIgnoredSamplesCount_;
        bStatus = false;
      }

      if(true == bStatus)
      {
        oOutput.fPressure_ = fPressure;
        oOutput.fTemperature_ = fTemperature;
        oOutput.uTimestampUs_ = uTimestampUs;
        oOutput.uValid_ = BoolToUint(true);
      }
    }

    NVIC_EnableIRQ(TIM7_IRQn);
    taskEXIT_CRITICAL();
  }

  CRte::GetInstance().oPortPressureInput1_.Write(oOutput);
}

bool CIcp20100Driver::getDeviceId(uint8_t& urChipId)
{
  return readReg(skuIcp20100RegDeviceId_, &urChipId, 1);
}

bool CIcp20100Driver::getVersion(uint8_t& urVersion)
{
  return readReg(skuIcp20100RegVersion_, &urVersion, 1);
}

void CIcp20100Driver::Init()
{
  bool bStatus{false};
  bool bIcp20100ModeSyncStatus{false};
  uint8_t uIcp20100ModeSyncWaitCycles{2};
  uint8_t uChipId{0};
  uint8_t uVersion{0};
  uint8_t uRxValue{0};

  uIgnoredSamplesCount_ = 0U;

  bStatus = getDeviceId(uChipId);
  if (true == bStatus)
  {
    if (skuIcp20100DeviceId_ != uChipId)
    {
      bStatus = false;
#ifdef ENABLE_PRINTF
      printf("ICP-20100: Get Chip ID failed\r\n");
#endif // ENABLE_PRINTF
    }
  }

  if (true == bStatus)
  {
    bStatus = getVersion(uVersion);

    if (skuIcp20100VersionB2_ != uVersion)
    {
      bStatus = false;
#ifdef ENABLE_PRINTF
      printf("ICP-20100: Unsupported version (%.2Xh)\r\n", uVersion);
#endif // ENABLE_PRINTF
    }
  }

  // Check synchronization of the selected mode to the internal clock domain
  if (true == bStatus)
  {
    while((uIcp20100ModeSyncWaitCycles > 0) && (false == bIcp20100ModeSyncStatus))
    {
      bStatus = readReg(skuIcp20100RegDeviceStatus_, &uRxValue, 1);
      if (true == bStatus)
      {
        // 1: Synchronization of the selected mode to the internal clock domain is finished. (P. 53)
        if((uRxValue & skuIcp20100ModeSyncStatus_) == skuIcp20100ModeSyncStatus_)
        {
          bIcp20100ModeSyncStatus = true;
          break;
        }
      }

      HAL_Delay(1);
      uIcp20100ModeSyncWaitCycles--;
    }

    bStatus = bIcp20100ModeSyncStatus;
  }

#ifdef ENABLE_PRINTF
  if (false == bStatus)
  {
    printf("ICP-20100: Synchronization ERROR!\r\n");
  }
#endif // ENABLE_PRINTF

  if (true == bStatus)
  {
    bStatus = writeReg(skuIcp20100RegInterruptMask_, &skuIcp20100FifoWmkHighUnMask_, 1);
  }

  if (true == bStatus)
  {
    bStatus = writeReg(skuIcp20100RegFifoConfig_, &skuIcp20100FifoWmHigh_, 1);
  }

  if (true == bStatus)
  {
    fifoFlush();
  }

  if (true == bStatus)
  {
    bStatus = writeReg(skuIcp20100RegModeSelect_, &skuIcp20100MeasModeContinuous_, 1);
  }

  if (true == bStatus)
  {
    bIsInitialized_ = true;
  }

#ifdef ENABLE_PRINTF
  if (bIsInitialized_)
  {
    printf("ICP-20100: Initialization finished\r\n");
  }
  else
  {
    printf("ICP-20100: Initialization failed\r\n");
  }
#endif // ENABLE_PRINTF
}

bool CIcp20100Driver::isWatermarkHighInterruptTriggered()
{
  bool bStatus = false;
  uint8_t uRxValue{};

  bStatus = readReg(skuIcp20100RegInterruptStatus_, &uRxValue, 1);
  if (true == bStatus)
  {
    bStatus = ((uRxValue & skuIcp20100InterruptStatusMask_) == skuIcp20100InterruptStatusMask_) ? true : false;
  }

  return bStatus;
}

bool CIcp20100Driver::watermarkHighInterruptClear()
{
  // Write policy is W1C, p. 50
  return writeReg(skuIcp20100RegInterruptStatus_, &skuIcp20100InterruptStatusMask_, 1);
}

bool CIcp20100Driver::fifoFlush()
{
  bool bStatus = false;
  uint8_t uRegValue{};

  bStatus = readReg(skuIcp20100RegFifoFill_, &uRegValue, 1);
  if (true == bStatus)
  {
    uRegValue |= skuIcp20100FifoFlush_;
    bStatus = writeReg(skuIcp20100RegFifoFill_, &uRegValue, 1);
  }

  return bStatus;
}

bool CIcp20100Driver::readPressureAndTemperature(float& frPressure, float& frTemperature)
{
  bool bStatus{false};
  uint8_t auValue[6]{};

  bStatus = readReg(skuIcp20100RegPressData0_, auValue, 6);

  if (true == bStatus)
  {
    // Calculate pressure
    const int32_t kiPressureRaw = static_cast<int32_t>(((auValue[2] & 0x0F) << 28) | (auValue[1] << 20) | (auValue[0] << 12)) >> 12;

    // 6.7.1 Pressure conversion formula (P. 34)
    frPressure = (static_cast<float>(kiPressureRaw) / 131072.0F) * 40000.0F + 70000.0F;

    // Calculate temperature
    const int32_t kiTemperatureRaw = static_cast<int32_t>(((auValue[5] & 0x0F) << 28) | (auValue[4] << 20) | (auValue[3] << 12)) >> 12;

    // 6.7.2 Temperature conversion formula (P. 35)
    frTemperature = (static_cast<float>(kiTemperatureRaw) / 262144.0F) * 65.0F + 25.0F;
  }

  return bStatus;
}

bool CIcp20100Driver::readReg(uint8_t uRegAddr, uint8_t* upBuffer, uint16_t uLen)
{
  const uint8_t auTxData[2] = {skuIcp20100CmdReadReg_, uRegAddr}; // Read from register

  HAL_StatusTypeDef eHalStatus = HAL_ERROR;
  HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_RESET);
  eHalStatus = HAL_SPI_Transmit(&hspi2, auTxData, 2, 2);
  if (HAL_OK == eHalStatus)
  {
    eHalStatus = HAL_SPI_Receive(&hspi2, upBuffer, uLen, 2);
  }
  HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_SET);

  return (HAL_OK == eHalStatus) ? true : false;
}

bool CIcp20100Driver::writeReg(uint8_t uRegAddr, const uint8_t* upBuffer, uint16_t uLen)
{
  const uint8_t auTxData[2] = {skuIcp20100CmdWriteReg_, uRegAddr}; // Write to register

  HAL_StatusTypeDef eHalStatus = HAL_ERROR;
  HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_RESET);
  eHalStatus = HAL_SPI_Transmit(&hspi2, auTxData, 2, 2);
  if (HAL_OK == eHalStatus)
  {
    eHalStatus = HAL_SPI_Transmit(&hspi2, upBuffer, uLen, 2);
  }
  HAL_GPIO_WritePin(SPI2_CS2_GPIO_Port, SPI2_CS2_Pin, GPIO_PIN_SET);

  return (HAL_OK == eHalStatus) ? true : false;
}
