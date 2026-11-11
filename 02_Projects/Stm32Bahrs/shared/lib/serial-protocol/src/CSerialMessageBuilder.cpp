/// @file CSerialMessageBuilder.cpp
/// @brief Implementation of the CSerialMessageBuilder class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CSerialMessageBuilder.h"
#include "crc.h"
#include "stm32f4xx_hal.h"
#include <cstring>
#include <limits>
#include "UintToBool.h"
#include "CMathConstants.h"
#include "CSoftwareVersion.h"
#include "AmsAssert.h"

#ifdef SEND_DEBUG_OUTPUT
#include "cmsis_os.h" // Dynamic memory allocation
#endif

CMutex CSerialMessageBuilder::oCrcMutex_;

CSerialMessageBuilder::CSerialMessageBuilder()
{
  AMS_HARD_ASSERT(oCrcMutex_.Create());
}

CSerialProtocol::SInertialDataMessage CSerialMessageBuilder::BuildInertialDataMessage(const SOutputImuData& korImuData)
{
  CSerialProtocol::SInertialDataMessage oMessage;

  oMessage.oInertialData_.uSequenceCounter_ = uInertialDataSequenceCounter_;

  auto encodeSignal = [](float fSignal, float fScale, float fMaxValue, uint32_t uHealthBitsPosition, CSerialProtocol::ESignalHealthInfo eHealthInfo, int16_t& irOutputSignal, uint16_t& urHealthBitfield)
    {
      uint16_t uHealthToSet;
      urHealthBitfield &= static_cast<uint16_t>(~(0b11U << uHealthBitsPosition)); // Clear health bits

      if ((fSignal > fMaxValue) || (fSignal < -fMaxValue))
      {
        uHealthToSet = 0b11 & static_cast<uint16_t>(CSerialProtocol::ESignalHealthInfo::eUnavailable);
      }
      else
      {
        uHealthToSet = 0b11 & static_cast<uint16_t>(eHealthInfo);
        irOutputSignal = static_cast<int16_t>(fSignal * fScale);
      }

      urHealthBitfield |= (uHealthToSet << uHealthBitsPosition);
    };

  if (CSerialProtocol::ESignalHealthInfo::eUnavailable != korImuData.eSpecificForceHealth_)
  {
    static constexpr float skfScaleFloatToInt = 1.0F / CSerialProtocol::skfSpecificForceScale_;

    encodeSignal(korImuData.fSpecificForceX_, skfScaleFloatToInt, CSerialProtocol::skfMaxSpecificForce_, BIT_POS_HEALTH_SPECIFIC_FORCE_X,
                 korImuData.eSpecificForceHealth_, oMessage.oInertialData_.iSpecificForceX_, oMessage.oInertialData_.uHealthInfo_);

    encodeSignal(korImuData.fSpecificForceY_, skfScaleFloatToInt, CSerialProtocol::skfMaxSpecificForce_, BIT_POS_HEALTH_SPECIFIC_FORCE_Y,
                 korImuData.eSpecificForceHealth_, oMessage.oInertialData_.iSpecificForceY_, oMessage.oInertialData_.uHealthInfo_);

    encodeSignal(korImuData.fSpecificForceZ_, skfScaleFloatToInt, CSerialProtocol::skfMaxSpecificForce_, BIT_POS_HEALTH_SPECIFIC_FORCE_Z,
                 korImuData.eSpecificForceHealth_, oMessage.oInertialData_.iSpecificForceZ_, oMessage.oInertialData_.uHealthInfo_);
  }

  if (CSerialProtocol::ESignalHealthInfo::eUnavailable != korImuData.eAngularRateHealth_)
  {
    static constexpr float skfScaleFloatToInt = 1.0F / CSerialProtocol::skfAngularRateScale_;

    encodeSignal(korImuData.fAngularRateX_, skfScaleFloatToInt, CSerialProtocol::skfMaxAngularRate_, BIT_POS_HEALTH_ANGULAR_RATE_X,
                 korImuData.eAngularRateHealth_, oMessage.oInertialData_.iAngularRateX_, oMessage.oInertialData_.uHealthInfo_);

    encodeSignal(korImuData.fAngularRateY_, skfScaleFloatToInt, CSerialProtocol::skfMaxAngularRate_, BIT_POS_HEALTH_ANGULAR_RATE_Y,
                 korImuData.eAngularRateHealth_, oMessage.oInertialData_.iAngularRateY_, oMessage.oInertialData_.uHealthInfo_);

    encodeSignal(korImuData.fAngularRateZ_, skfScaleFloatToInt, CSerialProtocol::skfMaxAngularRate_, BIT_POS_HEALTH_ANGULAR_RATE_Z,
                 korImuData.eAngularRateHealth_, oMessage.oInertialData_.iAngularRateZ_, oMessage.oInertialData_.uHealthInfo_);
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));

  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uInertialDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::STimeOfInertialDataMessage CSerialMessageBuilder::BuildTimeOfInertialDataMessage(const SOutputImuData& korImuData)
{
  CSerialProtocol::STimeOfInertialDataMessage oMessage;

  oMessage.oTimeOfInertialData_.uSequenceCounter_ = uTimeOfInertialDataSequenceCounter_;

  if ((CSerialProtocol::ESignalHealthInfo::eUnavailable != korImuData.eSpecificForceHealth_) ||
      (CSerialProtocol::ESignalHealthInfo::eUnavailable != korImuData.eAngularRateHealth_))
  {
    oMessage.oTimeOfInertialData_.uTimestampUs_ = korImuData.uTimestampUs_;
    oMessage.oTimeOfInertialData_.uInertialDataSequenceCounter_ = uInertialDataSequenceCounter_;
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));

  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uTimeOfInertialDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::SNavigationDataMessage CSerialMessageBuilder::BuildNavigationDataMessage(const SSafeVerticalChannelData& korVerticalChannelData,
                                                                                          const SSafeAttitudeData& korVehicleAttitude,
                                                                                          const SSafeMagneticHeadingData& korMagneticHeading)
{
  using EHealth = CSerialProtocol::ESignalHealthInfo;
  CSerialProtocol::SNavigationDataMessage oMessage{};
  float fTmp;

  oMessage.oNavigationData_.uSequenceCounter_ = uNavigationDataSequenceCounter_;

  if (CSerialProtocol::ESignalHealthInfo::eUnavailable != korVerticalChannelData.eHealth_)
  {
    // Fill height
    if ((korVerticalChannelData.fHeight_ >= CSerialProtocol::skfMinHeight_) && (korVerticalChannelData.fHeight_ <= CSerialProtocol::skfMaxHeight_))
    {
      fTmp = 1.0F / CSerialProtocol::skfHeightScale_;
      oMessage.oNavigationData_.uPressureHeight_ = static_cast<uint16_t>(fTmp * (korVerticalChannelData.fHeight_ + CSerialProtocol::skfHeighOffset_));

      const uint16_t kuHealthToSet = 0b11 & static_cast<uint16_t>(korVerticalChannelData.eHealth_);
      oMessage.oNavigationData_.uHealthInfo_ &= static_cast<uint16_t>(~(0b11U << BIT_POS_HEALTH_HEIGHT)); // Clear health bits
      oMessage.oNavigationData_.uHealthInfo_ |= (kuHealthToSet << BIT_POS_HEALTH_HEIGHT);
    }

    // Fill velocity downwards
    if ((korVerticalChannelData.fVelocityDown_ >= CSerialProtocol::skfMinVelocityDown_) && (korVerticalChannelData.fVelocityDown_ <= CSerialProtocol::skfMaxVelocityDown_))
    {
      fTmp = 1.0F / CSerialProtocol::skfVelocityDownScale_;
      oMessage.oNavigationData_.iVelocityDown_ = static_cast<int16_t>(fTmp * korVerticalChannelData.fVelocityDown_);

      const uint16_t kuHealthToSet = 0b11 & static_cast<uint16_t>(korVerticalChannelData.eHealth_);
      oMessage.oNavigationData_.uHealthInfo_ &= static_cast<uint16_t>(~(0b11U << BIT_POS_HEALTH_VELOCITY_DOWN)); // Clear health bits
      oMessage.oNavigationData_.uHealthInfo_ |= (kuHealthToSet << BIT_POS_HEALTH_VELOCITY_DOWN);
    }
  }

  if (EHealth::eUnavailable != korVehicleAttitude.eHealth_)
  {
    // Fill attitude angles
    fTmp = 1.0F / CSerialProtocol::skfAngleScale_;
    oMessage.oNavigationData_.iRoll_ = static_cast<int16_t>(fTmp * korVehicleAttitude.fRoll_);
    oMessage.oNavigationData_.iPitch_ = static_cast<int16_t>(fTmp * korVehicleAttitude.fPitch_);

    const uint16_t kuHealthToSet = 0b11 & static_cast<uint16_t>(korVehicleAttitude.eHealth_);
    oMessage.oNavigationData_.uHealthInfo_ &= static_cast<uint16_t>(~(0b11U << BIT_POS_HEALTH_ROLL)); // Clear health bits
    oMessage.oNavigationData_.uHealthInfo_ |= (kuHealthToSet << BIT_POS_HEALTH_ROLL);
    oMessage.oNavigationData_.uHealthInfo_ &= static_cast<uint16_t>(~(0b11U << BIT_POS_HEALTH_PITCH)); // Clear health bits
    oMessage.oNavigationData_.uHealthInfo_ |= (kuHealthToSet << BIT_POS_HEALTH_PITCH);
  }

  if (EHealth::eUnavailable != korMagneticHeading.eHealth_)
  {
    fTmp = 1.0F / CSerialProtocol::skfAngleScale_;
    oMessage.oNavigationData_.uMagneticHeading_ = static_cast<uint16_t>(fTmp * korMagneticHeading.fMagneticHeading_);

    const uint16_t kuHealthToSet = 0b11 & static_cast<uint16_t>(korMagneticHeading.eHealth_);
    oMessage.oNavigationData_.uHealthInfo_ &= static_cast<uint16_t>(~(0b11U << BIT_POS_HEALTH_MAGNETIC_HEADING)); // Clear health bits
    oMessage.oNavigationData_.uHealthInfo_ |= (kuHealthToSet << BIT_POS_HEALTH_MAGNETIC_HEADING);
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uNavigationDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::STimeOfNavigationDataMessage CSerialMessageBuilder::BuildTimeOfNavigationDataMessage(const SSafeAttitudeData& korVehicleAttitude)
{
  CSerialProtocol::STimeOfNavigationDataMessage oMessage;

  oMessage.oTimeOfNavigationData_.uSequenceCounter_ = uTimeOfNavigationDataSequenceCounter_;

  if (CSerialProtocol::ESignalHealthInfo::eUnavailable != korVehicleAttitude.eHealth_)
  {
    oMessage.oTimeOfNavigationData_.uNavigationDataSequenceCounter_ = uNavigationDataSequenceCounter_;
    oMessage.oTimeOfNavigationData_.uTimestampUs_ = korVehicleAttitude.uTimestampUs_;
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uTimeOfNavigationDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::SAccuracyDataMessage CSerialMessageBuilder::BuildAccuracyDataMessage(const SSafeAttitudeData& korVehicleAttitude)
{
  CSerialProtocol::SAccuracyDataMessage oMessage;

  oMessage.oAccuracy_.uSequenceCounter_ = uAccuracyDataSequenceCounter_;

  if (CSerialProtocol::ESignalHealthInfo::eUnavailable != korVehicleAttitude.eHealth_)
  {
    float fTmp = 1.0F / CSerialProtocol::skfAngleScale_;

    uint16_t uStd = static_cast<uint16_t>(fTmp * korVehicleAttitude.fAttitudeStd1_);
    oMessage.oAccuracy_.uAttitudeStdN_ = ((0U == uStd) ? 0x01 : uStd);

    uStd = static_cast<uint16_t>(fTmp * korVehicleAttitude.fAttitudeStd2_);
    oMessage.oAccuracy_.uAttitudeStdE_ = ((0U == uStd) ? 0x01 : uStd);

    oMessage.oAccuracy_.uMagneticHeadingStd_ = 0U;

    oMessage.oAccuracy_.uTimestampUs_ = korVehicleAttitude.uTimestampUs_;
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uAccuracyDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::STimeOfLatestSyncPulseMessage CSerialMessageBuilder::BuildTimeOfLatestSyncPulseMessage(uint64_t uTimestamp)
{
  CSerialProtocol::STimeOfLatestSyncPulseMessage oMessage;

  oMessage.oTimeOfLatestSyncPulse_.uSequenceCounter_ = uTimeOfLatestPulseSequenceCounter_;
  oMessage.oTimeOfLatestSyncPulse_.uTimestampUs_ = uTimestamp;

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uTimeOfLatestPulseSequenceCounter_;

  return oMessage;
}

CSerialProtocol::SSoftwareVersionMessage CSerialMessageBuilder::BuildSoftwareVersionMessage()
{
  CSerialProtocol::SSoftwareVersionMessage oMessage;

  static_assert(sizeof(CSoftwareVersion::skacProjectCode_) == sizeof(oMessage.oSoftwareVersion_.acProjectCode_));

  oMessage.oSoftwareVersion_.uMajor_ = CSoftwareVersion::skuMajor_;
  oMessage.oSoftwareVersion_.uMinor_ = CSoftwareVersion::skuMinor_;
  oMessage.oSoftwareVersion_.acProjectCode_[0] = CSoftwareVersion::skacProjectCode_[0];
  oMessage.oSoftwareVersion_.acProjectCode_[1] = CSoftwareVersion::skacProjectCode_[1];
  oMessage.oSoftwareVersion_.acProjectCode_[2] = CSoftwareVersion::skacProjectCode_[2];

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  return oMessage;
}

CSerialProtocol::SHardwareVersionMessage CSerialMessageBuilder::BuildHardwareVersionMessage()
{
  static const uint16_t skuMcuId{static_cast<uint16_t>(HAL_GetDEVID())};
  static const uint32_t skuUniqueId1{HAL_GetUIDw0()};
  static const uint32_t skuUniqueId2{HAL_GetUIDw1()};
  static const uint32_t skuUniqueId3{HAL_GetUIDw2()};

  CSerialProtocol::SHardwareVersionMessage oMessage;
  oMessage.oHardwareVersion_.uMcuId_ = skuMcuId;
  oMessage.oHardwareVersion_.uUniqueId1_ = skuUniqueId1;
  oMessage.oHardwareVersion_.uUniqueId2_ = skuUniqueId2;
  oMessage.oHardwareVersion_.uUniqueId3_ = skuUniqueId3;

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  return oMessage;
}

#ifdef SEND_DEBUG_OUTPUT
uint8_t* CSerialMessageBuilder::BuildDebugMessage(const CSerialProtocol::SDebugData& korDebugData, size_t& urMessageLen)
{
  size_t uMessageLen = sizeof(CSerialProtocol::SMessageHeader);
  uint8_t* upMessageBytes { nullptr };

  if (korDebugData.uMsgType_ == CSerialProtocol::EMessageIds::eDebugEventWriteToPort)
  {
    uMessageLen += sizeof(CSerialProtocol::SWriteToPortEventPartialData);
    uMessageLen += korDebugData.oByteVector_.size();
    uMessageLen += ((PROTOCOL_WORD_LEN - (uMessageLen % PROTOCOL_WORD_LEN))); // pad to multiple of 32 bit words
    uMessageLen += sizeof(CSerialProtocol::CrcType_t);
  }
  else if (korDebugData.uMsgType_ == CSerialProtocol::EMessageIds::eDebugEventRunnableCall)
  {
    uMessageLen += sizeof(CSerialProtocol::SRunnableCallEventData);
    uMessageLen += ((PROTOCOL_WORD_LEN - (uMessageLen % PROTOCOL_WORD_LEN))); // pad to multiple of 32 bit words
    uMessageLen += sizeof(CSerialProtocol::CrcType_t);
  }
  else
  {
    uMessageLen = 0;
  }

  if (uMessageLen > 0)
  {
    upMessageBytes = static_cast<uint8_t*>(pvPortMalloc(uMessageLen));

    if (nullptr != upMessageBytes)
    {
      if (korDebugData.uMsgType_ == CSerialProtocol::EMessageIds::eDebugEventWriteToPort)
      {
        CSerialProtocol::SMessageHeader oHeader;
        uint8_t* upCopyAddress = upMessageBytes;

        memset(upMessageBytes, 0, uMessageLen);

        oHeader.uMsgType_ = static_cast<uint8_t>(korDebugData.uMsgType_);

        putBytesIntoArray(&upCopyAddress, reinterpret_cast<uint8_t*>(&oHeader), sizeof(oHeader));
        putBytesIntoArray(&upCopyAddress, &korDebugData.uId_, sizeof(korDebugData.uId_));
        putBytesIntoArray(&upCopyAddress, reinterpret_cast<const uint8_t*>(&korDebugData.uTimestampUs_), sizeof(korDebugData.uTimestampUs_));

        AMS_HARD_ASSERT(korDebugData.oByteVector_.size() <= std::numeric_limits<uint16_t>::max());

        uint16_t uDataLen = static_cast<uint16_t>(korDebugData.oByteVector_.size());
        putBytesIntoArray(&upCopyAddress, reinterpret_cast<uint8_t*>(&uDataLen), sizeof(uDataLen));

        for (unsigned uIndex = 0U; uIndex < korDebugData.oByteVector_.size(); ++uIndex)
        {
          *upCopyAddress = korDebugData.oByteVector_[uIndex];
          ++upCopyAddress;
        }

        uint32_t uCrc = CalculateCrc(reinterpret_cast<uint32_t*>(upMessageBytes), (uMessageLen / PROTOCOL_WORD_LEN) - 1);

        // We need to account for padding
        upCopyAddress = upMessageBytes + (uMessageLen - sizeof(uCrc));
        putBytesIntoArray(&upCopyAddress, reinterpret_cast<uint8_t*>(&uCrc), sizeof(uCrc));
      }
      else if (korDebugData.uMsgType_ == CSerialProtocol::EMessageIds::eDebugEventRunnableCall)
      {
        CSerialProtocol::SMessageHeader oHeader;
        uint8_t* upCopyAddress = upMessageBytes;

        memset(upMessageBytes, 0, uMessageLen);

        oHeader.uMsgType_ = static_cast<uint8_t>(korDebugData.uMsgType_);

        putBytesIntoArray(&upCopyAddress, reinterpret_cast<uint8_t*>(&oHeader), sizeof(oHeader));
        putBytesIntoArray(&upCopyAddress, &korDebugData.uId_, sizeof(korDebugData.uId_));
        putBytesIntoArray(&upCopyAddress, reinterpret_cast<const uint8_t*>(&korDebugData.uTimestampUs_), sizeof(korDebugData.uTimestampUs_));
        putBytesIntoArray(&upCopyAddress, &korDebugData.bAllPortsLocked_, sizeof(korDebugData.bAllPortsLocked_));

        uint32_t uCrc = CalculateCrc(reinterpret_cast<uint32_t*>(upMessageBytes), (uMessageLen / PROTOCOL_WORD_LEN) - 1);

        // We need to account for padding
        upCopyAddress = upMessageBytes + (uMessageLen - sizeof(uCrc));
        putBytesIntoArray(&upCopyAddress, reinterpret_cast<uint8_t*>(&uCrc), sizeof(uCrc));
      }
      else
      {
        // This shall never happen
      }
    }
  }

  urMessageLen = uMessageLen;

  return upMessageBytes;
}
#endif // SEND_DEBUG_OUTPUT

uint32_t CSerialMessageBuilder::CalculateCrc(uint32_t* pData, uint32_t uNumberOfWords_)
{
  uint32_t uCrc{};

  if (oCrcMutex_.IsInitialized() && oCrcMutex_.Acquire(skuCrcMutexTimeoutInTicks_))
  {
    uCrc = HAL_CRC_Calculate(&hcrc, pData, uNumberOfWords_);
    oCrcMutex_.Release();
  }
  else
  {
    // We do not expect the mutex to not be initialized or fail to be aquired with
    // the relatively large timeout, since CRC calculation is fast.
    AMS_HARD_ASSERT(false);
  }

  return uCrc;
}

#ifdef SEND_DEBUG_OUTPUT
void CSerialMessageBuilder::putBytesIntoArray(uint8_t** upAddress, const uint8_t* upData, size_t uLen)
{
  memcpy(*upAddress, upData, uLen);
  *upAddress += uLen;
}
#endif // SEND_DEBUG_OUTPUT
