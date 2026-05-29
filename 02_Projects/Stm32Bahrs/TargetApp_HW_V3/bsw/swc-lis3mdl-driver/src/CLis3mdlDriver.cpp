/// @file CLis3mdlDriver.cpp
/// @brief Implementation of the LIS3MDL driver software component class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifdef ENABLE_PRINTF
#include <stdio.h>
#endif // ENABLE_PRINTF

#include "CRte.h"
#include "CLis3mdlDriver.h"
#include "AmsAssert.h"
#include "GetMicroseconds.h"

#include "CI2CHandler.h"

CLis3mdlDriver& CLis3mdlDriver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CLis3mdlDriver soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

bool CLis3mdlDriver::IsInitialized()
{
  return bIsInitialized_;
}

void CLis3mdlDriver::PollSensor(uint64_t uTimestampUs)
{
  bool bStatus = false;
  SMagneticMeasurement oOutput{};
  float fValueX{}, fValueY{}, fValueZ{}, fTemperature{};

  if (true == isDataReady())
  {
    bStatus = IsInitialized();

    if (true == bStatus)
    {
      bStatus = readMagnetometer(fValueX, fValueY, fValueZ);

      if (true == bStatus)
      {
        bStatus = readTemperature(fTemperature);
      }

      if(true == bStatus)
      {
        oOutput.fVectorX_ = fValueX;
        oOutput.fVectorY_ = fValueY;
        oOutput.fVectorZ_ = fValueZ;
        oOutput.fTemperature_ = fTemperature;
        oOutput.uTimestampUs_ = uTimestampUs;
        oOutput.uValid_ = BoolToUint(true);
      }
    }

    CRte::GetInstance().oPortMagnetometerInput3_.Write(oOutput);
  }
}

void CLis3mdlDriver::PollSensor()
{
  PollSensor(GetMicroseconds());
}

bool CLis3mdlDriver::getDeviceId(uint8_t& urChipId)
{
  return readReg(skuLis3mdlRegWhoAmI_, &urChipId, 1);
}

void CLis3mdlDriver::Init()
{
  bool bStatus = false;
  uint8_t uChipId;

  bStatus = getDeviceId(uChipId);
  if (true == bStatus)
  {
    if (skuLis3mdlWhoAmI_ != uChipId)
    {
      bStatus = false;
#ifdef ENABLE_PRINTF
      printf("LIS3MDL: Get Chip ID failed\r\n");
#endif // ENABLE_PRINTF
    }
  }

  if (true == bStatus)
  {
    bStatus = writeReg(skuLis3mdlRegCtrlReg1_, &skuLis3mdlCtrlReg1Value_, 1);
  }

  if (true == bStatus)
  {
    bStatus = writeReg(skuLis3mdlRegCtrlReg3_, &skuLis3mdlCtrlReg3Value_, 1);
  }

  if (true == bStatus)
  {
    bStatus = writeReg(skuLis3mdlRegCtrlReg4_, &skuLis3mdlCtrlReg4OpModeZ_, 1);
  }

  if (true == bStatus)
  {
    bIsInitialized_ = true;
  }

#ifdef ENABLE_PRINTF
  if (bIsInitialized_)
  {
    printf("LIS3MDL: Initialization finished\r\n");
  }
  else
  {
    printf("LIS3MDL: Initialization failed\r\n");
  }
#endif // ENABLE_PRINTF

}

bool CLis3mdlDriver::readMagnetometer(float& frValueX, float& frValueY, float& frValueZ)
{
  bool bStatus = false;
  uint8_t auValue[6]{};

  bStatus = readReg((skuLis3mdlRegOutXL_), auValue, 6);

  if (true == bStatus)
  {
    const int16_t kiMagX = static_cast<int16_t>((auValue[1] << 8) | auValue[0]);
    const int16_t kiMagY = static_cast<int16_t>((auValue[3] << 8) | auValue[2]);
    const int16_t kiMagZ = static_cast<int16_t>((auValue[5] << 8) | auValue[4]);

    frValueX = static_cast<float>(-kiMagY) * skfLis3mdlMagneticSensitivity_;
    frValueY = static_cast<float>(kiMagX) * skfLis3mdlMagneticSensitivity_;
    frValueZ = static_cast<float>(kiMagZ) * skfLis3mdlMagneticSensitivity_;
  }

  return bStatus;
}

bool CLis3mdlDriver::readTemperature(float& frTemperature)
{
  bool bStatus = false;
  uint8_t auValue[2]{};

  bStatus = readReg(skuLis3mdlRegTempOutL_, auValue, 2);
  if (true == bStatus)
  {
    const int16_t kiTemp = static_cast<int16_t>((auValue[1] << 8) | auValue[0]);
    frTemperature = static_cast<float>(kiTemp) * skfLis3mdlTempSensitivity_ + 25.0F;
  }

  return bStatus;
}

bool CLis3mdlDriver::isDataReady()
{
  bool bStatus = false;
  uint8_t uValue{};

  bStatus = readReg(skuLis3mdlRegStatReg_, &uValue, 1);
  if (true == bStatus)
  {
    if (skuLis3mdlStatRegXYZDataReady_ != (uValue & skuLis3mdlStatRegXYZDataReady_))
    {
      bStatus = false;
    }
  }

  return bStatus;
}

bool CLis3mdlDriver::readReg(uint8_t uRegAddr, uint8_t* upBuffer, uint16_t uLen)
{
  return CI2CHandler::GetInstance().MemRead(CI2CHandler::EBus::eI2c3,
                                            skuLis3mdlI2CAddress_,
                                            uRegAddr,
                                            1U,
                                            upBuffer,
                                            uLen,
                                            skuLis3mdlI2cTimeout_);
}

bool CLis3mdlDriver::writeReg(uint8_t uRegAddr, const uint8_t* upBuffer, uint16_t uLen)
{
  return CI2CHandler::GetInstance().MemWrite(CI2CHandler::EBus::eI2c3,
                                             skuLis3mdlI2CAddress_,
                                             uRegAddr,
                                             1U,
                                             upBuffer,
                                             uLen,
                                             skuLis3mdlI2cTimeout_);
}
