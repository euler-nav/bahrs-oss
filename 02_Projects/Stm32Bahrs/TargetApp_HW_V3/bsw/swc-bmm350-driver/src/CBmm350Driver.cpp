/// @file CBmm350Driver.cpp
/// @brief Implementation of the Bmm350 driver software component class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifdef ENABLE_PRINTF
#include <stdio.h>
#endif // ENABLE_PRINTF

#include "CRte.h"
#include "CBmm350Driver.h"
#include "AmsAssert.h"

#include "CI2CHandler.h"

CBmm350Driver& CBmm350Driver::getInstanceImpl(unsigned uInstanceIndex)
{
  static CBmm350Driver soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

bool CBmm350Driver::IsInitialized()
{
  return bIsInitialized_;
}

void CBmm350Driver::PollSensor(uint64_t uTimestampUs)
{
  bool bStatus = false;
  SMagneticMeasurement oOutput{};
  CVector3dAndTemperature oSensorData{};
  uint8_t uIntStatusRegVal{};

  bStatus = IsInitialized();

  if (true == bStatus)
  {
    // Get data ready interrupt status
    bStatus = readReg(skuBmm350RegIntStatus_, &uIntStatusRegVal, 1);

    // Check if data ready interrupt occurred
    if (true == bStatus)
    {
      if ((skuBmm350DrDyDataReg_ & uIntStatusRegVal) == skuBmm350DrDyDataReg_)
      {
        bStatus = readCompensatedMagAndTempData(oSensorData);
      }
      else
      {
        bStatus = false;
      }
    }

    if (true == bStatus)
    {
      oOutput.fVectorX_ = oSensorData.fMagneticVectorX_;
      oOutput.fVectorY_ = -oSensorData.fMagneticVectorY_;
      oOutput.fVectorZ_ = -oSensorData.fMagneticVectorZ_;
      oOutput.fTemperature_ = oSensorData.fTemperatureInDegreesCelsius_;
      oOutput.uTimestampUs_ = uTimestampUs;
      oOutput.uValid_ = BoolToUint(true);
    }
  }

  CRte::GetInstance().oPortMagnetometerInput2_.Write(oOutput);
}

void CBmm350Driver::Init()
{
  bool bStatus = false;
  uint8_t uChipId{};

  bStatus = getDeviceId(uChipId);
  if (true == bStatus)
  {
    if (skuBmm350ChipId_ != uChipId)
    {
      bStatus = false;
#ifdef ENABLE_PRINTF
      printf("BMM350: Get Chip ID failed\r\n");
#endif // ENABLE_PRINTF
    }
  }

  if (true == bStatus)
  {
    bStatus = readOtp();
  }

  if (true == bStatus)
  {
    bStatus = configureInterrupt();
  }

  if (true == bStatus)
  {
    bStatus = configureOdrAndAveraging();
  }

  if (true == bStatus)
  {
    bStatus = enableXYZAxis();
  }

  if (true == bStatus)
  {
    bStatus = enableNormalMode();
  }

  if (true == bStatus)
  {
    bIsInitialized_ = true;
  }

#ifdef ENABLE_PRINTF
  if (bIsInitialized_)
  {
    printf("BMM350: Initialization finished\r\n");
  }
  else
  {
    printf("BMM350: Initialization failed\r\n");
  }
#endif // ENABLE_PRINTF
}

bool CBmm350Driver::getDeviceId(uint8_t& urChipId)
{
  return readReg(skuBmm350RegDeviceId_, &urChipId, 1);
}

bool CBmm350Driver::readOtp()
{
  bool bStatus = false;
  uint8_t uWaitAttempts{};
  uint8_t uOtpRegAddr{}, uOtpCmd{}, uOtpStatus{}, uOtpErr{};
  std::array<uint8_t, 2U> auOtpBytes{};
  COtpDataVector auOtpData{};

#ifdef ENABLE_PRINTF
  printf("BMM350: reading OTP data:\r\n");
#endif

  for (uOtpRegAddr = 0; uOtpRegAddr < auOtpData.size(); uOtpRegAddr++)
  {
    uOtpCmd = skuBmm350OtpCmdDirRead_ | (uOtpRegAddr & skuBmm350OtpWordAddrMsk_);
    bStatus = writeReg(skuBmm350RegOtpCmd_, &uOtpCmd, 1);

    if (true == bStatus)
    {
      do // wait until OTP command done flag set
      {
        // Delay before reading the status register (API delay is 300us)
        HAL_Delay(1);

        // Get OTP status
        bStatus = readReg(skuBmm350RegOtpStatus_, &uOtpStatus, 1);

        if (true == bStatus)
        {
          uOtpErr = uOtpStatus & skuBmm350OtpStatusMsk_;
          if (skuBmm350OtpStatusNoError_ != uOtpErr)
          {
            bStatus = false;
#ifdef ENABLE_PRINTF
            printf("BMM350: OTP error: 0x%.2X\r\n", uOtpErr);
#endif
          }
        }

        uWaitAttempts++;

        if (uWaitAttempts > 2U)
        {
          bStatus = false;
        }
      }
      while ((skuBmm350OtpStatusCmdDone_ != (uOtpStatus & skuBmm350OtpStatusCmdDone_)) && (true == bStatus));

      uWaitAttempts = 0;

      if ((skuBmm350OtpStatusNoError_ == uOtpErr) && (true == bStatus))
      {
        bStatus = readReg(skuBmm350RegOtpMsb_, auOtpBytes.data(), auOtpBytes.size());
        if (true == bStatus)
        {
          auOtpData[uOtpRegAddr] = static_cast<uint16_t>((auOtpBytes[0] << 8) | auOtpBytes[1]);

#ifdef ENABLE_PRINTF
          printf("0x%.4X ", auOtpData[uOtpRegAddr]);
          if ((uOtpRegAddr + 1) % 8 == 0)
          {
            printf("\r\n");
          }
#endif
        }
      }

    }

    if (false == bStatus) // Leave for-loop if error occurred
    {
#ifdef ENABLE_PRINTF
      printf("BMM350: OTP read failed!\r\n");
#endif
      break;
    }
  }

  // Power off OTP
  if (true == bStatus)
  {
    bStatus = writeReg(skuBmm350RegOtpCmd_, &skuBmm350OtpCmdPwrOffOtp_, 1);
  }

  if (true == bStatus)
  {
    updateCompensationParameters(auOtpData);
  }

  return bStatus;
}

void CBmm350Driver::updateCompensationParameters(COtpDataVector& auOtpData)
{
  int16_t iOffsetX{}, iOffsetY{}, iOffsetZ{}, iTemperatureOffset{}, iTemperature0{};
  int8_t iScaleX{}, iScaleY{}, iScaleZ{}, iScaleT{};
  int8_t iTempOffsetCoefX{}, iTempOffsetCoefY{}, iTempOffsetCoefZ{};
  int8_t iTempScaleCoefX{}, iTempScaleCoefY{}, iTempScaleCoefZ{};
  int8_t iCrossAxisSensXY{}, iCrossAxisSensYX{}, iCrossAxisSensZX{}, iCrossAxisSensZY{};

  uint16_t uTmp = (auOtpData[skuBmm350OtpWordMagOffsetXY_] & 0x0FFF);
  iOffsetX = static_cast<int16_t>(uTmp << 4) >> 4;

  uTmp = (auOtpData[skuBmm350OtpWordMagOffsetXY_] & 0xF000) | ((auOtpData[skuBmm350OtpWordMagOffsetYZ_] & 0x00FF) << 4);
  iOffsetY = static_cast<int16_t>(uTmp) >> 4;

  uTmp = (auOtpData[skuBmm350OtpWordMagOffsetYZ_] & 0x0F00) | (auOtpData[skuBmm350OtpWordMagOffsetZScaleX_] & 0x00FF);
  iOffsetZ = static_cast<int16_t>(uTmp << 4) >> 4;

  iTemperatureOffset = static_cast<int8_t>(auOtpData[skuBmm350OtpWordTempOffsetScale_] & 0x00FF);

  oCompensationParameters_.fOffsetX_ = static_cast<float>(iOffsetX);
  oCompensationParameters_.fOffsetY_ = static_cast<float>(iOffsetY);
  oCompensationParameters_.fOffsetZ_ = static_cast<float>(iOffsetZ);
  oCompensationParameters_.fTemperatureOffset_ = static_cast<float>(iTemperatureOffset) / 5.0F;

  iScaleX = static_cast<int8_t>((auOtpData[skuBmm350OtpWordMagOffsetZScaleX_] & 0xFF00) >> 8);
  iScaleY = static_cast<int8_t>(auOtpData[skuBmm350OtpWordMagScaleYZ_] & 0x00FF);
  iScaleZ = static_cast<int8_t>((auOtpData[skuBmm350OtpWordMagScaleYZ_] & 0xFF00) >> 8);
  iScaleT = static_cast<int8_t>((auOtpData[skuBmm350OtpWordTempOffsetScale_] & 0xFF00) >> 8);

  oCompensationParameters_.fScaleX_ = static_cast<float>(iScaleX) / 256.0F;
  oCompensationParameters_.fScaleY_ = static_cast<float>(iScaleY) / 256.0F;
  oCompensationParameters_.fScaleZ_ = static_cast<float>(iScaleZ) / 256.0F;
  oCompensationParameters_.fTemperatureScale_ = static_cast<float>(iScaleT) / 512.0F;

  iTempOffsetCoefX = static_cast<int8_t>(auOtpData[skuBmm350OtpWordTempCoefOffsetXScaleX_] & 0x00FF);
  iTempOffsetCoefY = static_cast<int8_t>(auOtpData[skuBmm350OtpWordTempCoefOffsetYScaleY_] & 0x00FF);
  iTempOffsetCoefZ = static_cast<int8_t>(auOtpData[skuBmm350OtpWordTempCoefOffsetZScaleZ_] & 0x00FF);

  oCompensationParameters_.fOffsetTemperatureCoefX_ = static_cast<float>(iTempOffsetCoefX) / 32.0F;
  oCompensationParameters_.fOffsetTemperatureCoefY_ = static_cast<float>(iTempOffsetCoefY) / 32.0F;
  oCompensationParameters_.fOffsetTemperatureCoefZ_ = static_cast<float>(iTempOffsetCoefZ) / 32.0F;

  iTempScaleCoefX = static_cast<int8_t>((auOtpData[skuBmm350OtpWordTempCoefOffsetXScaleX_] & 0xFF00) >> 8);
  iTempScaleCoefY = static_cast<int8_t>((auOtpData[skuBmm350OtpWordTempCoefOffsetYScaleY_] & 0xFF00) >> 8);
  iTempScaleCoefZ = static_cast<int8_t>((auOtpData[skuBmm350OtpWordTempCoefOffsetZScaleZ_] & 0xFF00) >> 8);

  oCompensationParameters_.fScaleTemperatureCoefX_ = static_cast<float>(iTempScaleCoefX) / 16384.0F;
  oCompensationParameters_.fScaleTemperatureCoefY_ = static_cast<float>(iTempScaleCoefY) / 16384.0F;
  oCompensationParameters_.fScaleTemperatureCoefZ_ = static_cast<float>(iTempScaleCoefZ) / 16384.0F;

  iTemperature0 = static_cast<int16_t>(auOtpData[skuBmm350OtpWordReferenceTemp_]);
  oCompensationParameters_.fTemperature0_ = static_cast<float>(iTemperature0) / 512.0F + 23.0F;

  iCrossAxisSensXY = static_cast<int8_t>(auOtpData[skuBmm350OtpWordCrossAxisSensXYYX_] & 0x00FF);
  iCrossAxisSensYX = static_cast<int8_t>((auOtpData[skuBmm350OtpWordCrossAxisSensXYYX_] & 0xFF00) >> 8);
  iCrossAxisSensZX = static_cast<int8_t>(auOtpData[skuBmm350OtpWordCrossAxisSensZXZY_] & 0x00FF);
  iCrossAxisSensZY = static_cast<int8_t>((auOtpData[skuBmm350OtpWordCrossAxisSensZXZY_] & 0xFF00) >> 8);

  oCompensationParameters_.fCrossAxisSensitivityXY_ = static_cast<float>(iCrossAxisSensXY) / 800.0F;
  oCompensationParameters_.fCrossAxisSensitivityYX_ = static_cast<float>(iCrossAxisSensYX) / 800.0F;
  oCompensationParameters_.fCrossAxisSensitivityZX_ = static_cast<float>(iCrossAxisSensZX) / 800.0F;
  oCompensationParameters_.fCrossAxisSensitivityZY_ = static_cast<float>(iCrossAxisSensZY) / 800.0F;
}

bool CBmm350Driver::readUncompensatedMagAndTempData(CVector3dAndTemperature& orRawSensorData)
{
  bool bStatus = false;
  std::array<uint8_t, 12U> auRegData{};

  bStatus = readReg(skuBmm350RegMagXXlsb_, auRegData.data(), auRegData.size());

  if (true == bStatus)
  {
    const int32_t kiMagXAxisRaw = static_cast<int32_t>((static_cast<uint32_t>(auRegData[0]) << 8) | (static_cast<uint32_t>(auRegData[1]) << 16) | (static_cast<uint32_t>(auRegData[2]) << 24)) >> 8;
    const int32_t kiMagYAxisRaw = static_cast<int32_t>((static_cast<uint32_t>(auRegData[3]) << 8) | (static_cast<uint32_t>(auRegData[4]) << 16) | (static_cast<uint32_t>(auRegData[5]) << 24)) >> 8;
    const int32_t kiMagZAxisRaw = static_cast<int32_t>((static_cast<uint32_t>(auRegData[6]) << 8) | (static_cast<uint32_t>(auRegData[7]) << 16) | (static_cast<uint32_t>(auRegData[8]) << 24)) >> 8;

    const int32_t kiTempRaw = static_cast<int32_t>((static_cast<uint32_t>(auRegData[9]) << 8) | (static_cast<uint32_t>(auRegData[10]) << 16) | (static_cast<uint32_t>(auRegData[11]) << 24)) >> 8;

    orRawSensorData.fMagneticVectorX_ = static_cast<float>(kiMagXAxisRaw) * skfBmm350LsbToMicroTeslaXy_;
    orRawSensorData.fMagneticVectorY_ = static_cast<float>(kiMagYAxisRaw) * skfBmm350LsbToMicroTeslaXy_;
    orRawSensorData.fMagneticVectorZ_ = static_cast<float>(kiMagZAxisRaw) * skfBmm350LsbToMicroTeslaZ_;
    orRawSensorData.fTemperatureInDegreesCelsius_ = static_cast<float>(kiTempRaw) * skfBmm350LsbToDegC_ - skfBmm350TemperatureShiftDegC_;
  }

  return bStatus;
}

bool CBmm350Driver::readCompensatedMagAndTempData(CVector3dAndTemperature& orSensorData)
{
  bool bStatus = false;
  CVector3dAndTemperature oRawData{};

  bStatus = readUncompensatedMagAndTempData(oRawData);

  if (true == bStatus)
  {
    orSensorData = compensateMeasurement(oRawData, oCompensationParameters_);
  }

  return bStatus;
}

CBmm350Driver::CVector3dAndTemperature CBmm350Driver::compensateMeasurement(const CVector3dAndTemperature& korRawData, const SCompensationParameters& oCompensationParameters)
{
  CVector3dAndTemperature orSensorData{};

  // Step 1: Temperature compensation
  const float fTemperature = (1 + oCompensationParameters.fTemperatureScale_) * korRawData.fTemperatureInDegreesCelsius_ + oCompensationParameters.fTemperatureOffset_;

  // Step 2: Offset and scale compensation
  const float kfMagneticDataXTmp = 1.0F / (1.0F + oCompensationParameters.fScaleTemperatureCoefX_ * (fTemperature - oCompensationParameters.fTemperature0_)) * \
      ((1.0F + oCompensationParameters.fScaleX_) * korRawData.fMagneticVectorX_ + oCompensationParameters.fOffsetX_ + \
          oCompensationParameters.fOffsetTemperatureCoefX_ * (fTemperature - oCompensationParameters.fTemperature0_));

  const float kfMagneticDataYTmp = 1.0F / (1.0F + oCompensationParameters.fScaleTemperatureCoefY_ * (fTemperature - oCompensationParameters.fTemperature0_)) * \
      ((1.0F + oCompensationParameters.fScaleY_) * korRawData.fMagneticVectorY_ + oCompensationParameters.fOffsetY_ + \
          oCompensationParameters.fOffsetTemperatureCoefY_ * (fTemperature - oCompensationParameters.fTemperature0_));

  const float kfMagneticDataZTmp = 1.0F / (1.0F + oCompensationParameters.fScaleTemperatureCoefZ_ * (fTemperature - oCompensationParameters.fTemperature0_)) * \
      ((1.0F + oCompensationParameters.fScaleZ_) * korRawData.fMagneticVectorZ_ + oCompensationParameters.fOffsetZ_ + \
          oCompensationParameters.fOffsetTemperatureCoefZ_ * (fTemperature - oCompensationParameters.fTemperature0_));

  // Step 3: Cross-axis compensation
  orSensorData.fMagneticVectorX_ = (kfMagneticDataXTmp - oCompensationParameters.fCrossAxisSensitivityXY_ * kfMagneticDataYTmp) / \
      (1.0F - oCompensationParameters.fCrossAxisSensitivityYX_ * oCompensationParameters.fCrossAxisSensitivityXY_);

  orSensorData.fMagneticVectorY_ = (kfMagneticDataYTmp - oCompensationParameters.fCrossAxisSensitivityYX_ * kfMagneticDataXTmp) / \
      (1.0F - oCompensationParameters.fCrossAxisSensitivityYX_ * oCompensationParameters.fCrossAxisSensitivityXY_);

  orSensorData.fMagneticVectorZ_ = kfMagneticDataZTmp + \
      (kfMagneticDataXTmp * (oCompensationParameters.fCrossAxisSensitivityYX_ * oCompensationParameters.fCrossAxisSensitivityZY_ - oCompensationParameters.fCrossAxisSensitivityZX_) - \
          kfMagneticDataYTmp * (oCompensationParameters.fCrossAxisSensitivityZY_ - oCompensationParameters.fCrossAxisSensitivityXY_ * oCompensationParameters.fCrossAxisSensitivityZX_)) / \
          (1.0F - oCompensationParameters.fCrossAxisSensitivityYX_ * oCompensationParameters.fCrossAxisSensitivityXY_);

  // Step 4. Convert uTesla to Gauss
  orSensorData.fMagneticVectorX_ *= skfBmm350MicroTeslaToGauss_;
  orSensorData.fMagneticVectorY_ *= skfBmm350MicroTeslaToGauss_;
  orSensorData.fMagneticVectorZ_ *= skfBmm350MicroTeslaToGauss_;
  orSensorData.fTemperatureInDegreesCelsius_ = fTemperature;

  return orSensorData;
}

bool CBmm350Driver::configureInterrupt()
{
  bool bStatus = false;
  uint8_t uRegValue{};

  bStatus = readReg(skuBmm350RegIntCtrl_, &uRegValue, 1);
  if (true == bStatus)
  {
    uRegValue |= skuBmm350RegIntCtrlValue_;
    bStatus = writeReg(skuBmm350RegIntCtrl_, &uRegValue, 1);
  }

  return bStatus;
}

bool CBmm350Driver::configureOdrAndAveraging()
{
  bool bStatus = false;
  uint8_t uRegValue{};

  bStatus = readReg(skuBmm350RegPmuCmdAggrSet_, &uRegValue, 1);
  if (true == bStatus)
  {
    uRegValue |= skuBmm350Odr12Hz5Average8_;
    bStatus = writeReg(skuBmm350RegPmuCmdAggrSet_, &uRegValue, 1);
  }

  return bStatus;
}

bool CBmm350Driver::enableXYZAxis()
{
  return writeReg(skuBmm350RegPmuCmdAxisEn_, &skuBmm350EnableXYZAxis_, 1);
}

bool CBmm350Driver::enableNormalMode()
{
  bool bStatus = false;
  uint8_t uPreviousPowerMode{};

  bStatus = readReg(skuBmm350RegPmuCmd_, &uPreviousPowerMode, 1);

  if (true == bStatus)
  {
    if (uPreviousPowerMode > skuBmm350PmuCmdBrFast_)
    {
      bStatus = false;
    }

    if ((true == bStatus) && ((uPreviousPowerMode == skuBmm350PmuCmdNm_) || (uPreviousPowerMode == skuBmm350PmuCmdOae_)))
    {
      bStatus = writeReg(skuBmm350RegPmuCmd_, &skuBmm350PmuCmdSus_, 1);

      if (true == bStatus)
      {
        HAL_Delay(6); // 6000 us from API
      }
    }

    if (true == bStatus)
    {
      bStatus = writeReg(skuBmm350RegPmuCmd_, &skuBmm350PmuCmdNm_, 1);

      if (true == bStatus)
      {
        HAL_Delay(38); // 38000 us from API
      }
    }
  }

  return bStatus;
}

bool CBmm350Driver::readReg(uint8_t uRegAddr, uint8_t* upBuffer, uint16_t uLen)
{
  bool bStatus = false;
  std::array<uint8_t, skuBmm350I2cBufferSize_> auBuffer{};
  uint16_t uIndex{};

  if ((uLen + skuBmm350DummyBytes_) <= skuBmm350I2cBufferSize_)
  {
    bStatus = true;
  }

  if (true == bStatus)
  {
    bStatus = CI2CHandler::GetInstance().MemRead(CI2CHandler::EBus::eI2c2,
                                                 skuBmm350I2CAddress_,
                                                 uRegAddr,
                                                 1U,
                                                 auBuffer.data(),
                                                 static_cast<uint16_t>(uLen + skuBmm350DummyBytes_),
                                                 skuBmm350I2cTimeout_);
    if (true == bStatus)
    {
      // Skip dummy bytes
      while (uIndex < uLen)
      {
        upBuffer[uIndex] = auBuffer[uIndex + skuBmm350DummyBytes_];
        uIndex++;
      }
    }
  }

  return bStatus;
}

bool CBmm350Driver::writeReg(uint8_t uRegAddr, const uint8_t* upBuffer, uint16_t uLen)
{
  return CI2CHandler::GetInstance().MemWrite(CI2CHandler::EBus::eI2c2,
                                             skuBmm350I2CAddress_,
                                             uRegAddr,
                                             1U,
                                             upBuffer,
                                             uLen,
                                             skuBmm350I2cTimeout_);
}
