/**
 * @file CVerticalChannelMonitorSwc.cpp
 * @brief Implementation of the vertical channel data (height and velocity downwards) monitor software component
 * @author Fedor Baklanov
 * @date 26 July 2024
 * @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#include "CVerticalChannelMonitorSwc.h"
#include "UintToBool.h"

void CVerticalChannelMonitorSwc::Init()
{
  // Do nothing
}

bool CVerticalChannelMonitorSwc::IsInitialized()
{
  return true;
}

CVerticalChannelMonitorSwc& CVerticalChannelMonitorSwc::getInstanceImpl(unsigned uInstanceIndex)
{
  static CVerticalChannelMonitorSwc soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CVerticalChannelMonitorSwc::Run()
{
  NVerticalChannelMonitorApi::CRedundantInputData oRedundantInputData{};
  NVerticalChannelMonitorApi::COutputData oMonitorOutput{};

  using ESensorId = NFusionLibCommon::ESensorId;

  const auto korInputs = CPortReader::ReadPorts(
    CRte::ERunnableIds::eInvalid,
    CRte::GetInstance().oPortBahrsFilterOutput1_,
    CRte::GetInstance().oPortBahrsFilterOutput2_,
    CRte::GetInstance().oPortBahrsFilterOutput3_);

  if (korInputs.has_value())
  {
    const auto& korFilterOutput1 = std::get<0>(korInputs.value());
    const auto& korFilterOutput2 = std::get<1>(korInputs.value());
    const auto& korFilterOutput3 = std::get<2>(korInputs.value());
    SVerticalChannelData oData1, oData2, oData3;

    oData1 = pickVerticalChannelDataFromFilterState(korFilterOutput1);
    oData2 = pickVerticalChannelDataFromFilterState(korFilterOutput2);
    oData3 = pickVerticalChannelDataFromFilterState(korFilterOutput3);

    populateRedundantInputDataObject(oData1, ESensorId::eBahrsFilter1, oRedundantInputData);
    populateRedundantInputDataObject(oData2, ESensorId::eBahrsFilter2, oRedundantInputData);
    populateRedundantInputDataObject(oData3, ESensorId::eBahrsFilter3, oRedundantInputData);

    oMonitorOutput = NVerticalChannelMonitorApi::VerticalChannelMonitorRun(oRedundantInputData);

    if (UintToBool(oData1.uValid_) && isDatasetSafe(ESensorId::eBahrsFilter1, oMonitorOutput))
    {
      CRte::GetInstance().oPortSafeVerticalChannelData_.Write(
          convertVerticalChannelDataToSafeVerticalChannelData(oData1, CSerialProtocol::ESignalHealthInfo::eSafe));
    }
    else if (UintToBool(oData2.uValid_) && isDatasetSafe(ESensorId::eBahrsFilter2, oMonitorOutput))
    {
      CRte::GetInstance().oPortSafeVerticalChannelData_.Write(
          convertVerticalChannelDataToSafeVerticalChannelData(oData2, CSerialProtocol::ESignalHealthInfo::eSafe));
    }
    else if (UintToBool(oData3.uValid_) && isDatasetSafe(ESensorId::eBahrsFilter3, oMonitorOutput))
    {
      CRte::GetInstance().oPortSafeVerticalChannelData_.Write(
          convertVerticalChannelDataToSafeVerticalChannelData(oData3, CSerialProtocol::ESignalHealthInfo::eSafe));
    }
    else
    {
      CRte::GetInstance().oPortSafeVerticalChannelData_.Write(computeUnsafeVerticalChannelOutput(oMonitorOutput));
    }
  }
  else
  {
    const SSafeVerticalChannelData koTmpData{};
    CRte::GetInstance().oPortSafeVerticalChannelData_.Write(koTmpData);
  }
}

SVerticalChannelData CVerticalChannelMonitorSwc::pickVerticalChannelDataFromFilterState(const CBahrsFilterOutput& korFilterOutput)
{
  SVerticalChannelData oData;

  if ((CClosedLoopErrorStateKfApi::EFilterModes::RUNNING == korFilterOutput.eFilterMode_) &&
      (false == korFilterOutput.bVerticalChannelDiverged_))
  {
    oData.fHeight_ = korFilterOutput.oState_.fHeight_;
    oData.fVelocityDown_ = korFilterOutput.oState_.fVelocityDown_;
    oData.uTimestampUs_ = korFilterOutput.uTimestampUs_;
    oData.uValid_ = BoolToUint(true);
  }

  return oData;
}

void CVerticalChannelMonitorSwc::populateRedundantInputDataObject(const SVerticalChannelData& korVerticalChannelData, NFusionLibCommon::ESensorId eSensorId, NVerticalChannelMonitorApi::CRedundantInputData& orRedundantInputData)
{
  if (UintToBool(korVerticalChannelData.uValid_))
  {
    NFusionLibCommon::SVerticalChannelData oTmpObject;

    oTmpObject.bValid_ = true;
    oTmpObject.eSensorId_ = eSensorId;
    oTmpObject.uTimestampUs_ = korVerticalChannelData.uTimestampUs_;
    oTmpObject.fHeight_ = korVerticalChannelData.fHeight_;
    oTmpObject.fVelocityDown_ = korVerticalChannelData.fVelocityDown_;

    orRedundantInputData.Set(oTmpObject);
  }
}

bool CVerticalChannelMonitorSwc::isDatasetSafe(NFusionLibCommon::ESensorId eSensorId, const NVerticalChannelMonitorApi::COutputData& korMonitorOutput)
{
  assert(NVerticalChannelMonitorApi::CRedundantInputData::IsSensorSupported(eSensorId));
  using EDetectionResult = NVerticalChannelMonitorApi::COutputData::EDetectionResult;
  using EIsolationResult = NVerticalChannelMonitorApi::COutputData::EIsolationResult;

  static const auto skoSignals = NVerticalChannelMonitorApi::COutputData::SLabeledArrayStruct::GetSensorLabels();
  bool bIsDatasetSafe{ true };

  for (auto eSignal : skoSignals)
  {
    const auto& korSignal = korMonitorOutput.GetSignal(eSignal);

    if (false == ((korSignal.eDetectionResults_ == EDetectionResult::eGood) ||
                  ((korSignal.eDetectionResults_ == EDetectionResult::eFailure) &&
                   (korSignal.eIsolationResults_ == EIsolationResult::eGood) &&
                   (korSignal.eIsolatedSensor_ != eSensorId))))
    {
      bIsDatasetSafe = false;
      break;
    }
  }

  return bIsDatasetSafe;
}

SSafeVerticalChannelData CVerticalChannelMonitorSwc::convertVerticalChannelDataToSafeVerticalChannelData(const SVerticalChannelData& korVerticalChannelData,
                                                                                                         CSerialProtocol::ESignalHealthInfo eHealth)
{
  SSafeVerticalChannelData oSafeVerticalChannelData{};

  if (UintToBool(korVerticalChannelData.uValid_))
  {
    oSafeVerticalChannelData.fHeight_ = korVerticalChannelData.fHeight_;
    oSafeVerticalChannelData.fVelocityDown_ = korVerticalChannelData.fVelocityDown_;
    oSafeVerticalChannelData.uTimestampUs_ = korVerticalChannelData.uTimestampUs_;
    oSafeVerticalChannelData.eHealth_ = eHealth;
  }

  return oSafeVerticalChannelData;
}

SSafeVerticalChannelData CVerticalChannelMonitorSwc::computeUnsafeVerticalChannelOutput(const NVerticalChannelMonitorApi::COutputData& korMonitorOutput)
{
  SSafeVerticalChannelData oVerticalChannelData{};
  using ESignals = NVerticalChannelMonitorApi::COutputData::EScalarSignals;

  const auto& korHeightSignal = korMonitorOutput.GetSignal(ESignals::eHeight);
  const auto& korVelocitySignal = korMonitorOutput.GetSignal(ESignals::eVelocityDown);

  if (korHeightSignal.bValid_ && korVelocitySignal.bValid_)
  {
    oVerticalChannelData.eHealth_ = CSerialProtocol::ESignalHealthInfo::eIntegrityRisk;
    oVerticalChannelData.fHeight_ = korHeightSignal.fSignal_;
    oVerticalChannelData.fVelocityDown_ = korVelocitySignal.fSignal_;

    if ((NFusionLibCommon::ESensorId::eUnknown != korHeightSignal.eSensorId_) &&
        (korHeightSignal.eSensorId_ == korVelocitySignal.eSensorId_))
    {
      // Height and velocity originate from the same filter.
      oVerticalChannelData.uTimestampUs_ = korHeightSignal.uTimestampUs_;
    }
    else
    {
      oVerticalChannelData.uTimestampUs_ = (korHeightSignal.uTimestampUs_ >> 1) + (korVelocitySignal.uTimestampUs_ >> 1);
    }
  }

  return oVerticalChannelData;
}

