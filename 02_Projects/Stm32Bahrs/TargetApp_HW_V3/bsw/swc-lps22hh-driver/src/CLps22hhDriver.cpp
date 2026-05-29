/// @file CLps22hhDriver.cpp
/// @brief Implementation of the BMI270 driver software component class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifdef ENABLE_PRINTF
#include <stdio.h>
#endif // ENABLE_PRINTF

#include "CRte.h"
#include "CLps22hhDriver.h"
#include "AmsAssert.h"

CLps22hhDriver& CLps22hhDriver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CLps22hhDriver soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

void CLps22hhDriver::PollSensor(uint64_t uTimestampUs)
{
  bool bStatus = false;
  float fTemperature{};
  float fPressure{};
  SBarometerMeasurement oOutput{};

  if (IsInitialized())
  {
    bStatus = isBaroDataReady();

    if(true == bStatus)
    {
      bStatus = readPressureAndTemperature(fPressure, fTemperature);

      if(true == bStatus)
      {
        oOutput.fPressure_ = fPressure;
        oOutput.fTemperature_ = fTemperature;
        oOutput.uTimestampUs_ = uTimestampUs;
        oOutput.uValid_ = BoolToUint(true);
      }
    }
  }

  CRte::GetInstance().oPortPressureInput3_.Write(oOutput);
}

bool CLps22hhDriver::IsInitialized()
{
  return bIsInitialized_;
}

void CLps22hhDriver::Init()
{
  bool bStatus = false;
  uint8_t uChipId{};

  bStatus = getChipId(uChipId);

  if (true == bStatus)
  {
    if (uChipId != skuLps22hhChipId_)
    {
      bStatus = false;
    }
  }

  /// Perform reset of the volatile registers to default values
  if (true == bStatus)
  {
    bStatus = performSoftwareReset();
  }

  if (true == bStatus)
  {
    bStatus = writeReg(skuLps22hhRegIfCtrl_, &skuLps22hhRegIfCtrlValue_, 1);
  }

  if (true == bStatus)
  {
    /// LOW_NOISE_EN is disabled by default and must be changed when the device is in power-down
    /// mode (ODR bits in CtrlReg1 are set to '000').
    bStatus = writeReg(skuLps22hhRegCtrlReg2_, &skuLps22hhRegCtrlReg2Value_, 1);
  }

  if (true == bStatus)
  {
    bStatus = writeReg(skuLps22hhRegCtrlReg1_, &skuLps22hhOdr25_, 1);
  }

  if (true == bStatus)
  {
    bStatus = writeReg(skuLps22hhRegCtrlReg3_, &skuLps22hhDrdy_, 1);
  }

  if (true == bStatus)
  {
    bIsInitialized_ = true;
  }

  //DEBUG
#ifdef ENABLE_PRINTF
  if (bStatus)
  {
    printf("LPS22HH: Initialization finished\r\n");
  }
  else
  {
    printf("LPS22HH: Initialization failed\r\n");
  }
#endif // ENABLE_PRINTF

}

bool CLps22hhDriver::getChipId(uint8_t& urChipId)
{
  return readReg(skuLps22hhRegWhoAmI_, &urChipId, 1);
}

bool CLps22hhDriver::performSoftwareReset()
{
  bool bStatus = false;
  uint8_t uRxValue{};

  /// The SWRESET bit resets the volatile registers to default value.
  bStatus = writeReg(skuLps22hhRegCtrlReg2_, &skuLps22hhSwReset_, 1);
  if (true == bStatus)
  {
    /// Wait 50 μs (Chapter 6 AN5209)
    HAL_Delay(1);
    bStatus = readReg(skuLps22hhRegCtrlReg2_, &uRxValue, 1);
    if (true == bStatus)
    {
      /// The bit is self-cleared when the reset is completed.
      bStatus = ((uRxValue & skuLps22hhSwReset_) != skuLps22hhSwReset_) ? true : false;
    }
  }

  return bStatus;
}

bool CLps22hhDriver::isBaroDataReady()
{
  bool bStatus = false;
  uint8_t uRxValue{};

  bStatus = readReg(skuLps22hhRegStatus_, &uRxValue, 1);
  if (true == bStatus)
  {
    bStatus = ((uRxValue & skuLps22hhPressTempDataAvailable_) == skuLps22hhPressTempDataAvailable_) ? true : false;
  }

  return bStatus;
}

bool CLps22hhDriver::readPressureAndTemperature(float& fPressure, float& fTemperature)
{
  bool bStatus = false;
  uint8_t auValue[5]{};

  bStatus = readReg(skuLps22hhRegPressOutXL_, auValue, 5);

  if (true == bStatus)
  {
    const int32_t kiPressureRaw = static_cast<int32_t>((auValue[2] << 24) | (auValue[1] << 16) | (auValue[0] << 8)) >> 8;
    fPressure = static_cast<float>(kiPressureRaw) * skfLps22hhPressSensitivityPa_;

    const int16_t kiTemperatureRaw = static_cast<int16_t>((auValue[4] << 8) | auValue[3]);
    fTemperature = static_cast<float>(kiTemperatureRaw) * skfLps22hhTempSensitivity_;
  }

  return bStatus;
}

bool CLps22hhDriver::readReg(uint8_t uRegAddr, uint8_t *upBuffer, uint16_t uLen)
{
  uRegAddr = uRegAddr | 0x80;

  return CSpiHandler::GetInstance().Receive(CSpiHandler::EPeripheral::eLps22, &uRegAddr, 1, upBuffer, uLen, 2);
}

bool CLps22hhDriver::writeReg(uint8_t uRegAddr, const uint8_t *upBuffer, uint16_t uLen)
{
  return CSpiHandler::GetInstance().Transmit(CSpiHandler::EPeripheral::eLps22, &uRegAddr, 1, upBuffer, uLen, 2);
}
