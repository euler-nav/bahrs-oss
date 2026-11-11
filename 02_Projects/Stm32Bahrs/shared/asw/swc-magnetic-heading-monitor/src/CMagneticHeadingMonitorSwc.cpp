/// @file CMagneticHeadingMonitorSwc.cpp
/// @brief Implementation of the magnetic heading monitor software component.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CMagneticHeadingMonitorSwc.h"
#include "UintToBool.h"
#include "AmsAssert.h"
#include "CRte.h"

void CMagneticHeadingMonitorSwc::Init()
{
  // Do nothing
}

bool CMagneticHeadingMonitorSwc::IsInitialized()
{
  return true;
}

CMagneticHeadingMonitorSwc& CMagneticHeadingMonitorSwc::getInstanceImpl(unsigned uInstanceIndex)
{
  static CMagneticHeadingMonitorSwc soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

void CMagneticHeadingMonitorSwc::Run()
{
  using ESensorId = NFusionLibCommon::ESensorId;

  const auto korInputs = CPortReader::ReadPorts(
    CRte::ERunnableIds::eInvalid,
    CRte::GetInstance().oPortMagneticHeading1_,
    CRte::GetInstance().oPortMagneticHeading2_,
    CRte::GetInstance().oPortMagneticHeading3_);

  if (korInputs.has_value())
  {
    const auto& korHeading1 = std::get<0>(korInputs.value());
    const auto& korHeading2 = std::get<1>(korInputs.value());
    const auto& korHeading3 = std::get<2>(korInputs.value());
    NMagneticHeadingMonitorApi::CRedundantInputData oRedundantInputData{};

    populateRedundantInputDataObject(korHeading1, ESensorId::eMagneticHeadingFilter1, oRedundantInputData);
    populateRedundantInputDataObject(korHeading2, ESensorId::eMagneticHeadingFilter2, oRedundantInputData);
    populateRedundantInputDataObject(korHeading3, ESensorId::eMagneticHeadingFilter3, oRedundantInputData);

    const NMagneticHeadingMonitorApi::COutputData koMonitorOutput{NMagneticHeadingMonitorApi::MagneticHeadingMonitorRun(oRedundantInputData)};

    SSafeMagneticHeadingData oSafeHeading{};

    if (UintToBool(korHeading1.uValid_) && isHeadingDataSafe(ESensorId::eMagneticHeadingFilter1, koMonitorOutput))
    {
      oSafeHeading = convertToSafeHeading(korHeading1, CSerialProtocol::ESignalHealthInfo::eSafe);
    }
    else if (UintToBool(korHeading2.uValid_) && isHeadingDataSafe(ESensorId::eMagneticHeadingFilter2, koMonitorOutput))
    {
      oSafeHeading = convertToSafeHeading(korHeading2, CSerialProtocol::ESignalHealthInfo::eSafe);
    }
    else if (UintToBool(korHeading3.uValid_) && isHeadingDataSafe(ESensorId::eMagneticHeadingFilter3, koMonitorOutput))
    {
      oSafeHeading = convertToSafeHeading(korHeading3, CSerialProtocol::ESignalHealthInfo::eSafe);
    }
    else
    {
      oSafeHeading = computeUnsafeHeadingOutput(koMonitorOutput);
    }

    CRte::GetInstance().oPortSafeMagneticHeading_.Write(oSafeHeading);
  }
  else
  {
    const SSafeMagneticHeadingData koTmpData{};
    CRte::GetInstance().oPortSafeMagneticHeading_.Write(koTmpData);
  }
}

bool CMagneticHeadingMonitorSwc::isHeadingDataSafe(NFusionLibCommon::ESensorId eSensorId, const NMagneticHeadingMonitorApi::COutputData& korMonitorOutput)
{
  AMS_HARD_ASSERT(NMagneticHeadingMonitorApi::CRedundantInputData::IsSensorSupported(eSensorId));

  using ESignals = NMagneticHeadingMonitorApi::COutputData::EScalarSignals;
  using EDetectionResult = NMagneticHeadingMonitorApi::COutputData::EDetectionResult;
  using EIsolationResult = NMagneticHeadingMonitorApi::COutputData::EIsolationResult;

  const auto& korSignal = korMonitorOutput.GetSignal(ESignals::eMagneticHeading);

  return (korSignal.eDetectionResults_ == EDetectionResult::eGood) ||
         ((korSignal.eDetectionResults_ == EDetectionResult::eFailure) &&
          (korSignal.eIsolationResults_ == EIsolationResult::eGood) &&
          (korSignal.eIsolatedSensor_ != eSensorId));
}

SSafeMagneticHeadingData CMagneticHeadingMonitorSwc::convertToSafeHeading(const SMagneticHeading& korHeading, CSerialProtocol::ESignalHealthInfo eHealth)
{
  SSafeMagneticHeadingData oSafeHeading;

  if (UintToBool(korHeading.uValid_))
  {
    oSafeHeading.fMagneticHeading_ = korHeading.fMagneticHeading_;
    oSafeHeading.uTimestampUs_ = korHeading.uTimestampUs_;
    oSafeHeading.eHealth_ = eHealth;
  }

  return oSafeHeading;
}

SSafeMagneticHeadingData CMagneticHeadingMonitorSwc::computeUnsafeHeadingOutput(const NMagneticHeadingMonitorApi::COutputData& korMonitorOutput)
{
  using ESignals = NMagneticHeadingMonitorApi::COutputData::EScalarSignals;
  SSafeMagneticHeadingData oHeadingData{};

  const auto& korHeadingSignal = korMonitorOutput.GetSignal(ESignals::eMagneticHeading);

  if (korHeadingSignal.bValid_)
  {
    oHeadingData.eHealth_ = CSerialProtocol::ESignalHealthInfo::eIntegrityRisk;
    oHeadingData.fMagneticHeading_ = korHeadingSignal.fSignal_;
    oHeadingData.uTimestampUs_ = korHeadingSignal.uTimestampUs_;
  }

  return oHeadingData;
}

void CMagneticHeadingMonitorSwc::populateRedundantInputDataObject(const SMagneticHeading& korHeading,
                                                                  NFusionLibCommon::ESensorId eSensorId,
                                                                  NMagneticHeadingMonitorApi::CRedundantInputData& orRedundantInputData)
{
  if (UintToBool(korHeading.uValid_))
  {
    NFusionLibCommon::SMagneticHeadingData oTmpObject;

    oTmpObject.bValid_ = true;
    oTmpObject.eSensorId_ = eSensorId;
    oTmpObject.uTimestampUs_ = korHeading.uTimestampUs_;
    oTmpObject.fMagneticHeading_ = korHeading.fMagneticHeading_;

    orRedundantInputData.Set(oTmpObject);
  }
}
