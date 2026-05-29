/**
 * @file CBmi270Driver.cpp
 * @brief Implementation of the BMI270 driver software component class.
 * @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#include "CRte.h"
#include "CBmi270Driver.h"
#include "GetMicroseconds.h"
#include "AmsAssert.h"

#ifdef ENABLE_PRINTF

#include <stdio.h>

#endif // ENABLE_PRINTF

CBmi270Driver& CBmi270Driver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CBmi270Driver soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

bool CBmi270Driver::IsInitialized()
{
  return bIsInitialized_;
}

void CBmi270Driver::PollSensor(uint64_t uTimestamp)
{
  bool bStatus = false;
  float fTemperature = 0.0F;
  SImuMeasurement oOutput{};

  bStatus = IsInitialized();

  if (true == bStatus) // Sensor is initialized
  {
    bStatus = isAccelGyroDataReady();
  }

  if(true == bStatus) // Accelerometer and Gyroscope data ready
  {
    CSpecificForce oSpecificForce{};
    CAngularRate oAngularRate{};

    bStatus = readAccelAndGyro(oSpecificForce, oAngularRate);

    if(true == bStatus) // Accelerometer and Gyroscope data read successful
    {
      oOutput.fSpecificForceX_ = -oSpecificForce[0];
      oOutput.fSpecificForceY_ = -oSpecificForce[1];
      oOutput.fSpecificForceZ_ = oSpecificForce[2];

      oOutput.fAngularRateX_ = -oAngularRate[0];
      oOutput.fAngularRateY_ = -oAngularRate[1];
      oOutput.fAngularRateZ_ = oAngularRate[2];

      oOutput.uTimestampUs_ = uTimestamp;
      oOutput.uImuValid_ = BoolToUint(true);
    }
  }

  if(true == bStatus) // Accelerometer and Gyroscope data read successful
  {
    bStatus = readTemp(fTemperature);

    if(true == bStatus) // Temperature data read successful
    {
      oOutput.fTemperature_ = fTemperature;
      oOutput.uTemperatureValid_ = BoolToUint(true);
    }
  }

  CRte::GetInstance().oPortImuInput2_.Write(oOutput);
}

bool CBmi270Driver::getChipID(uint8_t& urChipId)
{
  return readReg(skuBmi270RegChipId_, &urChipId, 1);
}

void CBmi270Driver::Init()
{
  bool bStatus = false;
  uint8_t uRegValue{};
  uint8_t uBmiInitStatus{};
  uint8_t uChipId{};
  uint16_t uChunkCounter{};
  uint16_t uAddress{};
  uint8_t uRegInitAddr1Value{};

#ifdef ENABLE_PRINTF
  printf("Starting BMI270 initialization.\n");
#endif // ENABLE_PRINTF

  bStatus = getChipID(uChipId);
  if (true == bStatus)
  {
    if (skuBmi270ChipId_ != uChipId )
    {
      bStatus = false;
    }
  }

  // Disable advanced power save mode
  if (true == bStatus)
  {
    // Read - modify - write
    bStatus = readReg(skuBmi270RegPwrConf_, &uRegValue, 1);
    if (true == bStatus)
    {
      uRegValue &= ~skuBmi270AdvPwrSaveMsk_; // clear bit 0
      bStatus = writeReg(skuBmi270RegPwrConf_, &uRegValue, 1);
    }
  }

  // Wait at least 450 us
  HAL_Delay(1);

  if (true == bStatus)
  {
    bStatus = readReg(skuBmi270RegPwrConf_, &uRegValue, 1);
    if (true == bStatus)
    {
      // Check if bit 0 is cleared
      bStatus = ((uRegValue & skuBmi270AdvPwrSaveMsk_) == 0x00) ? true : false;
    }
  }

  // Prepare configuration load
  if (true == bStatus)
  {
    bStatus = writeReg(skuBmi270RegInitCtrl_, &skuBmi270PrepareConfLoad_, 1);
  }

  // Upload configuration file

  static_assert(sizeof(skauBmi270ConfigFile) == skuBmi270ConfigFileSize_);

  if (true == bStatus)
  {
    for (uChunkCounter = 0U; uChunkCounter < skuBmi270PacketNum_; uChunkCounter++)
    {
      bStatus = writeReg(skuBmi270RegInitData_, ((uint8_t*) skauBmi270ConfigFile + skuBmi270ChunkSize_ * uChunkCounter), skuBmi270ChunkSize_);
      if (false == bStatus)
      {
        break;
      }

      uAddress += skuBmi270ChunkSize_ / 2; // Address needs to be incremented by the length of the chunk in bytes/2
      uRegInitAddr1Value = (uAddress >> skuBmi270InitAddr1Pos_);
      bStatus = writeReg(skuBmi270RegInitAddr1_, &uRegInitAddr1Value, 1);
      if (false == bStatus)
      {
        break;
      }
    }
  }

  // Complete configuration load
  if (true == bStatus)
  {
    bStatus = writeReg(skuBmi270RegInitCtrl_, &skuBmi270CompleteConfLoad_, 1);
  }

  // Wait at most 20 msec
  HAL_Delay(20);

  if (true == bStatus)
  {
    bStatus = getIntStatusReg(uBmiInitStatus);
    if ((uBmiInitStatus & skuBmi270AsicInitDoneMsk_) != skuBmi270AsicInitDone_)
    {
      bStatus = false;
    }
  }

  if (true == bStatus)
  {
    bStatus = enableGyroAccelTemp();
  }
  if (true == bStatus)
  {
    bStatus = configAccel();
  }
  if (true == bStatus)
  {
    bStatus = configGyro();
  }
  if (true == bStatus)
  {
    bStatus = configIntPin();
  }
  if (true == bStatus)
  {
    bIsInitialized_ = true;
  }

#ifdef ENABLE_PRINTF
  if (bIsInitialized_)
  {
    printf("BMI270 initialization SUCCEEDED.\n");
  }
  else
  {
    printf("BMI270 initialization FAILED.\n");
  }
#endif // ENABLE_PRINTF
}

bool CBmi270Driver::getIntStatusReg(uint8_t& uBmiInitStatus)
{
  bool bStatus = false;

  bStatus = readReg(skuBmi270RegIntStat_, &uBmiInitStatus, 1);

  return bStatus;
}

bool CBmi270Driver::enableGyroAccelTemp()
{
  bool bStatus = false;
  uint8_t uRxValue{};

  bStatus = writeReg(skuBmi270RegPwrCtrl_, &skuBmi270EnableGyroAccelTemp_, 1);
  if (true == bStatus)
  {
    bStatus = readReg(skuBmi270RegPwrCtrl_, &uRxValue, 1);

    if (true == bStatus)
    {
      if ((uRxValue & skuBmi270EnableGyroAccelTemp_) != skuBmi270EnableGyroAccelTemp_)
      {
        bStatus = false;
      }
    }
  }

  return bStatus;
}

bool CBmi270Driver::configAccel()
{
  bool bStatus = false;

  bStatus = writeReg(skuBmi270RegAccelConf_, &skuBmi270AccelConfVal_, 1);

  if (true == bStatus)
  {
  bStatus = writeReg(skuBmi270RegAccelRange_, &skuBmi270AccelRangeVal_, 1);
  }

  return bStatus;
}

bool CBmi270Driver::configGyro()
{
  bool bStatus = false;

  bStatus = writeReg(skuBmi270RegGyroConf_, &skuBmi270GyroConfVal_, 1);

  if (true == bStatus)
  {
    bStatus = writeReg(skuBmi270RegGyroRange_, &skuBmi270GyroRangeVal_, 1);
  }

  return bStatus;
}

bool CBmi270Driver::configIntPin()
{
  bool bStatus = false;

  bStatus = writeReg(skuBmi270RegInt1IoCtrl_, &skuBmi270Int1IoCtrlValue_, 1);

  if (true == bStatus)
  {
    bStatus = writeReg(skuBmi270RegIntMapData_, &skuBmi270DrdyInt1_, 1);
  }

  return bStatus;
}

bool CBmi270Driver::isAccelGyroDataReady()
{
  bool bStatus = false;
  uint8_t uRxValue{};

  bStatus = readReg(skuBmi270RegIntStatus1_, &uRxValue, 1);
  if (true == bStatus)
  {
    bStatus = ((uRxValue & skuBmi270GyroAccelDatareadyInt_) == skuBmi270GyroAccelDatareadyInt_) ? true : false;
  }

  return bStatus;
}

bool CBmi270Driver::readAccelAndGyro(CSpecificForce& orSpecificForce, CAngularRate& orAngularRate)
{
  bool bStatus = false;
  uint8_t auValue[12]{};
  int16_t iAccelX{}, iAccelY{}, iAccelZ{};
  int16_t iGyroX{}, iGyroY{}, iGyroZ{};

  bStatus = readReg(skuBmi270RegAccXLsb_, auValue, 12);

  if (true == bStatus)
  {
    iAccelX = static_cast<int16_t>((auValue[1] << 8) | auValue[0]);
    iAccelY = static_cast<int16_t>((auValue[3] << 8) | auValue[2]);
    iAccelZ = static_cast<int16_t>((auValue[5] << 8) | auValue[4]);

    iGyroX = static_cast<int16_t>((auValue[7] << 8) | auValue[6]);
    iGyroY = static_cast<int16_t>((auValue[9] << 8) | auValue[8]);
    iGyroZ = static_cast<int16_t>((auValue[11] << 8) | auValue[10]);

    orSpecificForce[0] = static_cast<float>(iAccelX) * (skfBmi270AccelSensitivity_ * skfStandardGravity_);
    orSpecificForce[1] = static_cast<float>(iAccelY) * (skfBmi270AccelSensitivity_ * skfStandardGravity_);
    orSpecificForce[2] = static_cast<float>(iAccelZ) * (skfBmi270AccelSensitivity_ * skfStandardGravity_);

    orAngularRate[0] = static_cast<float>(iGyroX) * (skfBmi270GyroSensitivity_ * skfDegToRad_);
    orAngularRate[1] = static_cast<float>(iGyroY) * (skfBmi270GyroSensitivity_ * skfDegToRad_);
    orAngularRate[2] = static_cast<float>(iGyroZ) * (skfBmi270GyroSensitivity_ * skfDegToRad_);
  }

  return bStatus;
}

bool CBmi270Driver::readTemp(float& fTemperature)
{
  bool bStatus = false;
  uint8_t auValue[2]{};
  int16_t iTemp{};

  bStatus = readReg(skuBmi270RegTemp0_, auValue, 2);

  if (bStatus)
  {
    iTemp = static_cast<int16_t>((auValue[1] << 8) | auValue[0]);
    fTemperature = static_cast<float>(iTemp) * skfBmi270TempSensitivity_ + 23.0F;
  }

  return bStatus;
}

bool CBmi270Driver::readReg(uint8_t uRegAddr, uint8_t* upBuffer, uint16_t uLen)
{
  uint8_t uTxData[2]{};
  uTxData[0] = uRegAddr | 0x80; // SPI read
  uTxData[1] = 0x00; // dummy byte rejection (p. 125, Rev. 1.5)

  return CSpiHandler::GetInstance().Receive(CSpiHandler::EPeripheral::eBmi270, uTxData, 2, upBuffer, uLen, 2);
}

bool CBmi270Driver::writeReg(uint8_t uRegAddr, const uint8_t* upBuffer, uint16_t uLen)
{
  return CSpiHandler::GetInstance().Transmit(CSpiHandler::EPeripheral::eBmi270, &uRegAddr, 1, upBuffer, uLen, 2);
}
