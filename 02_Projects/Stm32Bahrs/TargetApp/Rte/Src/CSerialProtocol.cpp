/**
 * @file CSerialProtocol.cpp
 * @brief Implementation of the BAHRS serial protocol
 * @author Fedor Baklanov
 * @date 07 June 2022
 */

#include "CSerialProtocol.h"
#include "crc.h"
#include <cstring>
#include "UintToBool.h"
#include "NavigationUtilities.h"
#include "CSoftwareVersion.h"

CSerialProtocol::SInertialDataMessage CSerialProtocol::BuildInertialDataMessage(const SOutputImuData& korImuData)
{
  SInertialDataMessage oMessage;

  oMessage.oInertialData_.uSequenceCounter_ = uInertialDataSequenceCounter_;

  if (true == UintToBool(korImuData.uSpecificForceValid_))
  {
    float fTmp = 1.0F / skfSpecificForceScale_;
    oMessage.oInertialData_.iSpecificForceX_ = static_cast<int16_t>(korImuData.fSpecificForceX_ * fTmp);
    oMessage.oInertialData_.iSpecificForceY_ = static_cast<int16_t>(korImuData.fSpecificForceY_ * fTmp);
    oMessage.oInertialData_.iSpecificForceZ_ = static_cast<int16_t>(korImuData.fSpecificForceZ_ * fTmp);
    oMessage.oInertialData_.uValidity_ |= 0x07; // 0000 0111
  }

  if (true == UintToBool(korImuData.uAngularRateValid_))
  {
    float fTmp = 1.0F / skfAngularRateScale_;
    oMessage.oInertialData_.iAngularRateX_ = static_cast<int16_t>(korImuData.fAngularRateX_ * fTmp);
    oMessage.oInertialData_.iAngularRateY_ = static_cast<int16_t>(korImuData.fAngularRateY_ * fTmp);
    oMessage.oInertialData_.iAngularRateZ_ = static_cast<int16_t>(korImuData.fAngularRateZ_ * fTmp);
    oMessage.oInertialData_.uValidity_ |= 0x38; // 0011 1000
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));

  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uInertialDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::STimeOfInertialDataMessage CSerialProtocol::BuildTimeOfInertialDataMessage(const SOutputImuData& korImuData)
{
  STimeOfInertialDataMessage oMessage;

  oMessage.oTimeOfInertialData_.uSequenceCounter_ = uTimeOfInertialDataSequenceCounter_;

  if ((true == UintToBool(korImuData.uSpecificForceValid_)) || (true == UintToBool(korImuData.uAngularRateValid_)))
  {
    oMessage.oTimeOfInertialData_.uTimestampUs_ = korImuData.uTimestampUs_;
    oMessage.oTimeOfInertialData_.uInertialDataSequenceCounter_ = uInertialDataSequenceCounter_;
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));

  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uTimeOfInertialDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::SNavigationDataMessage CSerialProtocol::BuildNavigationDataMessage(const NBahrsFilterApi::SOutputData& korBahrsOutput)
{
  SNavigationDataMessage oMessage;

  oMessage.oNavigationData_.uSequenceCounter_ = uNavigationDataSequenceCounter_;

  if ((CClosedLoopErrorStateKfApi::EFilterModes::RUNNING == korBahrsOutput.eFilterMode_) ||
      (CClosedLoopErrorStateKfApi::EFilterModes::IDLE == korBahrsOutput.eFilterMode_))
  {
    const NBahrsFilterApi::SBahrsState& korState = korBahrsOutput.oState_;
    float fTmp;

    // Fill height
    if ((korState.fHeight_ >= skfMinHeight_) && (korState.fHeight_ <= skfMaxHeight_))
    {
      fTmp = 1.0F / skfHeightScale_;
      oMessage.oNavigationData_.uPressureHeight_ = static_cast<uint16_t>(fTmp * (korState.fHeight_ + skfHeighOffset_));
      oMessage.oNavigationData_.uValidity_ |= BIT_VALID_HEIGHT;
    }

    // Fill velocity downwards
    if ((korState.fVelocityDown_ >= skfMinVelocityDown_) && (korState.fVelocityDown_ <= skfMaxVelocityDown_))
    {
      fTmp = 1.0F / skfVelocityDownScale_;
      oMessage.oNavigationData_.iVelocityDown_ = static_cast<int16_t>(fTmp * korState.fVelocityDown_);
      oMessage.oNavigationData_.uValidity_ |= BIT_VALID_VELOCITY_DOWN;
    }

    float fRoll, fPitch, fYaw;

    NNavigationUtilities::EulerFromQuaternion(korState.oQuaternionBodyToNed_, fRoll, fPitch, fYaw);

    // Fill attitude angles
    fTmp = 1.0F / skfAngleScale_;
    oMessage.oNavigationData_.iRoll_ = static_cast<int16_t>(fTmp * fRoll);
    oMessage.oNavigationData_.iPitch_ = static_cast<int16_t>(fTmp * fPitch);

    oMessage.oNavigationData_.uMagneticHeading_ = 0.0F;
    oMessage.oNavigationData_.uValidity_ |= (BIT_VALID_ROLL | BIT_VALID_PITCH);
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uNavigationDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::STimeOfNavigationDataMessage CSerialProtocol::BuildTimeOfNavigationDataMessage(const NBahrsFilterApi::SOutputData& korBahrsOutput)
{
  STimeOfNavigationDataMessage oMessage;

  oMessage.oTimeOfNavigationData_.uSequenceCounter_ = uTimeOfNavigationDataSequenceCounter_;

  if ((CClosedLoopErrorStateKfApi::EFilterModes::RUNNING == korBahrsOutput.eFilterMode_) ||
      (CClosedLoopErrorStateKfApi::EFilterModes::IDLE == korBahrsOutput.eFilterMode_))
  {
    oMessage.oTimeOfNavigationData_.uNavigationDataSequenceCounter_ = uNavigationDataSequenceCounter_;
    oMessage.oTimeOfNavigationData_.uTimestampUs_ = korBahrsOutput.uTimestampUs_;
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uTimeOfNavigationDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::SAccuracyDataMessage CSerialProtocol::BuildAccuracyDataMessage(const NBahrsFilterApi::SOutputData& korBahrsOutput)
{
  SAccuracyDataMessage oMessage;

  oMessage.oAccuracy_.uSequenceCounter_ = uAccuracyDataSequenceCounter_;

  if ((CClosedLoopErrorStateKfApi::EFilterModes::RUNNING == korBahrsOutput.eFilterMode_) ||
      (CClosedLoopErrorStateKfApi::EFilterModes::IDLE == korBahrsOutput.eFilterMode_))
  {
    float fTmp = 1.0F / skfAngleScale_;

    uint16_t uStd = static_cast<uint16_t>(fTmp * korBahrsOutput.oStateStd_.oAttitude_(0));
    oMessage.oAccuracy_.uAttitudeStdN_ = ((0U == uStd) ? 0x01 : uStd);

    uStd = static_cast<uint16_t>(fTmp * korBahrsOutput.oStateStd_.oAttitude_(1));
    oMessage.oAccuracy_.uAttitudeStdE_ = ((0U == uStd) ? 0x01 : uStd);

    oMessage.oAccuracy_.uMagneticHeadingStd_ = 0U;

    oMessage.oAccuracy_.uTimestampUs_ = korBahrsOutput.uTimestampUs_;
  }

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uAccuracyDataSequenceCounter_;

  return oMessage;
}

CSerialProtocol::STimeOfLatestSyncPulseMessage CSerialProtocol::BuildTimeOfLatestSyncPulseMessage(uint64_t uTimestamp)
{
  STimeOfLatestSyncPulseMessage oMessage;

  oMessage.oTimeOfLatestSyncPulse_.uSequenceCounter_ = uTimeOfLatestPulseSequenceCounter_;
  oMessage.oTimeOfLatestSyncPulse_.uTimestampUs_ = uTimestamp;

  memset(oMessage.auPadding_, 0, sizeof(oMessage.auPadding_));
  oMessage.uCrc_ = CalculateCrc(reinterpret_cast<uint32_t*>(&oMessage), (sizeof(oMessage) / PROTOCOL_WORD_LEN) - 1);

  ++uTimeOfLatestPulseSequenceCounter_;

  return oMessage;
}

CSerialProtocol::SSoftwareVersionMessage CSerialProtocol::BuildSoftwareVersionMessage()
{
  SSoftwareVersionMessage oMessage;

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

uint32_t CSerialProtocol::CalculateCrc(uint32_t* pData, uint32_t uNumberOfWords_)
{
  return HAL_CRC_Calculate(&hcrc, pData, uNumberOfWords_);
}

