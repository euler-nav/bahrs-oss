/**
 * @file CAttitudeMonitorSwc.cpp
 * @brief Implementation of the attitude monitor software component.
 * @author Fedor Baklanov
 * @date 26 July 2024
 * @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#include "CAttitudeMonitorSwc.h"
#include "UintToBool.h"

void CAttitudeMonitorSwc::Init()
{
  // Do nothing
}

bool CAttitudeMonitorSwc::IsInitialized()
{
  return true;
}

CAttitudeMonitorSwc& CAttitudeMonitorSwc::getInstanceImpl(unsigned uInstanceIndex)
{
  static CAttitudeMonitorSwc soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CAttitudeMonitorSwc::Run()
{
  NAttitudeMonitorApi::CRedundantInputData oRedundantInputData{};
  NAttitudeMonitorApi::COutputData oMonitorOutput{};

  const auto korInputs = CPortReader::ReadPorts(
    CRte::ERunnableIds::eInvalid,
    CRte::GetInstance().oPortVehicleAttitude1_,
    CRte::GetInstance().oPortVehicleAttitude2_,
    CRte::GetInstance().oPortVehicleAttitude3_);

  if (korInputs.has_value())
  {
    const auto& korAttitudeData1 = std::get<0>(korInputs.value());
    const auto& korAttitudeData2 = std::get<1>(korInputs.value());
    const auto& korAttitudeData3 = std::get<2>(korInputs.value());

    populateRedundantInputDataObject(korAttitudeData1, NFusionLibCommon::ESensorId::eBahrsFilter1, oRedundantInputData);
    populateRedundantInputDataObject(korAttitudeData2, NFusionLibCommon::ESensorId::eBahrsFilter2, oRedundantInputData);
    populateRedundantInputDataObject(korAttitudeData3, NFusionLibCommon::ESensorId::eBahrsFilter3, oRedundantInputData);

    oMonitorOutput = NAttitudeMonitorApi::AttitudeMonitorRun(oRedundantInputData);

    if (UintToBool(korAttitudeData1.uValid_) && isAttitudeDataSafe(NFusionLibCommon::ESensorId::eBahrsFilter1, oMonitorOutput))
    {
      CRte::GetInstance().oPortSafeVehicleAttitude_.Write(convertAttitudeToSafeAttitude(korAttitudeData1, CSerialProtocol::ESignalHealthInfo::eSafe));
    }
    else if (UintToBool(korAttitudeData2.uValid_) && isAttitudeDataSafe(NFusionLibCommon::ESensorId::eBahrsFilter2, oMonitorOutput))
    {
      CRte::GetInstance().oPortSafeVehicleAttitude_.Write(convertAttitudeToSafeAttitude(korAttitudeData2, CSerialProtocol::ESignalHealthInfo::eSafe));
    }
    else if (UintToBool(korAttitudeData3.uValid_) && isAttitudeDataSafe(NFusionLibCommon::ESensorId::eBahrsFilter3, oMonitorOutput))
    {
      CRte::GetInstance().oPortSafeVehicleAttitude_.Write(convertAttitudeToSafeAttitude(korAttitudeData3, CSerialProtocol::ESignalHealthInfo::eSafe));
    }
    else
    {
      CRte::GetInstance().oPortSafeVehicleAttitude_.Write(computeUnsafeAttitudeOutput(oMonitorOutput, korAttitudeData1, korAttitudeData2, korAttitudeData3));
    }
  }
  else
  {
    const SSafeAttitudeData koTmpData{};
    CRte::GetInstance().oPortSafeVehicleAttitude_.Write(koTmpData);
  }
}

bool CAttitudeMonitorSwc::isAttitudeDataSafe(NFusionLibCommon::ESensorId eSensorId, const NAttitudeMonitorApi::COutputData& korMonitorOutput)
{
  assert(NAttitudeMonitorApi::CRedundantInputData::IsSensorSupported(eSensorId));

  static const auto skoSignals = NAttitudeMonitorApi::COutputData::SLabeledArrayStruct::GetSensorLabels();
  using EDetectionResult = NAttitudeMonitorApi::COutputData::EDetectionResult;
  using EIsolationResult = NAttitudeMonitorApi::COutputData::EIsolationResult;

  bool bIsAttitudeDatasetSafe{ true };

  for (auto eSignal : skoSignals)
  {
    const auto& korSignal = korMonitorOutput.GetSignal(eSignal);

    if (false == ((korSignal.eDetectionResults_ == EDetectionResult::eGood) ||
                  ((korSignal.eDetectionResults_ == EDetectionResult::eFailure) &&
                   (korSignal.eIsolationResults_ == EIsolationResult::eGood) &&
                   (korSignal.eIsolatedSensor_ != eSensorId))))
    {
      bIsAttitudeDatasetSafe = false;
      break;
    }
  }

  return bIsAttitudeDatasetSafe;
}

SSafeAttitudeData CAttitudeMonitorSwc::convertAttitudeToSafeAttitude(const SAttitudeData& korAttitude, CSerialProtocol::ESignalHealthInfo eHealth)
{
  SSafeAttitudeData oSafeAttitude;

  if (UintToBool(korAttitude.uValid_))
  {
    oSafeAttitude.fRoll_ = korAttitude.fRoll_;
    oSafeAttitude.fPitch_ = korAttitude.fPitch_;
    oSafeAttitude.fAttitudeStd1_ = korAttitude.fAttitudeStd1_;
    oSafeAttitude.fAttitudeStd2_ = korAttitude.fAttitudeStd2_;
    oSafeAttitude.uTimestampUs_ = korAttitude.uTimestampUs_;
    oSafeAttitude.eHealth_ = eHealth;
  }

  return oSafeAttitude;
}

SSafeAttitudeData CAttitudeMonitorSwc::computeUnsafeAttitudeOutput(const NAttitudeMonitorApi::COutputData& korMonitorOutput,
                                                                   const SAttitudeData& korAttitude1,
                                                                   const SAttitudeData& korAttitude2,
                                                                   const SAttitudeData& korAttitude3)
{
  using ESignals = NAttitudeMonitorApi::COutputData::EScalarSignals;
  SSafeAttitudeData oAttitudeData{};

  const auto& korRollSignal = korMonitorOutput.GetSignal(ESignals::eRoll);
  const auto& korPitchSignal = korMonitorOutput.GetSignal(ESignals::ePitch);

  auto getAttitudeDataBySensorId = [&korAttitude1, &korAttitude2, &korAttitude3](NFusionLibCommon::ESensorId eSensorId) -> const SAttitudeData&
  {
    switch (eSensorId)
    {
      case NFusionLibCommon::ESensorId::eBahrsFilter1:
      {
        return korAttitude1;
      }
      case NFusionLibCommon::ESensorId::eBahrsFilter2:
      {
        return korAttitude2;
      }
      case NFusionLibCommon::ESensorId::eBahrsFilter3:
      {
        return korAttitude3;
      }
      default:
      {
        AMS_HARD_ASSERT(false);
        return korAttitude1; // Intentionally unreachable
      }
    }
  };

  if (korRollSignal.bValid_ && korPitchSignal.bValid_)
  {
    oAttitudeData.eHealth_ = CSerialProtocol::ESignalHealthInfo::eIntegrityRisk;
    oAttitudeData.fRoll_ = korRollSignal.fSignal_;
    oAttitudeData.fPitch_ = korPitchSignal.fSignal_;

    if ((NFusionLibCommon::ESensorId::eUnknown != korRollSignal.eSensorId_) && (korRollSignal.eSensorId_ == korPitchSignal.eSensorId_))
    {
      // Roll and pitch originate from the same filter.
      oAttitudeData.uTimestampUs_ = korRollSignal.uTimestampUs_;

      const auto& korAttitudeData = getAttitudeDataBySensorId(korRollSignal.eSensorId_);
      oAttitudeData.fAttitudeStd1_ = korAttitudeData.fAttitudeStd1_;
      oAttitudeData.fAttitudeStd2_ = korAttitudeData.fAttitudeStd2_;
    }
    else
    {
      oAttitudeData.fAttitudeStd1_ = CMathConstants::skfPi_ / 3.0F;
      oAttitudeData.fAttitudeStd2_ = CMathConstants::skfPi_ / 3.0F;
      oAttitudeData.uTimestampUs_ = (korRollSignal.uTimestampUs_ >> 1) + (korPitchSignal.uTimestampUs_ >> 1);
    }
  }

  return oAttitudeData;
}

void CAttitudeMonitorSwc::populateRedundantInputDataObject(const SAttitudeData& korAttitudeData,
                                                           NFusionLibCommon::ESensorId eSensorId,
                                                           NAttitudeMonitorApi::CRedundantInputData& orRedundantInputData)
{
  if (UintToBool(korAttitudeData.uValid_))
  {
    NFusionLibCommon::SAttitudeOutputData oTmpObject;

    oTmpObject.bValid_ = true;
    oTmpObject.eSensorId_ = eSensorId;
    oTmpObject.uTimestampUs_ = korAttitudeData.uTimestampUs_;
    oTmpObject.fRoll_ = korAttitudeData.fRoll_;
    oTmpObject.fPitch_ = korAttitudeData.fPitch_;

    orRedundantInputData.Set(oTmpObject);
  }
}

