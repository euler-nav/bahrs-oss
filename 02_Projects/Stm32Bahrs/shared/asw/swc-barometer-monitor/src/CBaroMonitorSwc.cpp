/**
 * @file CBaroMonitorSwc.cpp
 * @brief Implementation of the barometer monitor software component.
 * @author Fedor Baklanov
 * @date 15 July 2024
 * @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#include "CBaroMonitorSwc.h"
#include "UintToBool.h"
#include "AmsAssert.h"


void CBaroMonitorSwc::Init()
{
  // Do nothing
}

bool CBaroMonitorSwc::IsInitialized()
{
  return true;
}

CBaroMonitorSwc& CBaroMonitorSwc::getInstanceImpl(unsigned uInstanceIndex)
{
  static CBaroMonitorSwc soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

void CBaroMonitorSwc::Run()
{
  CRedundantPressureData oRedundantInputData{};
  NBaroMonitorApi::COutputData oMonitorOutput{};

  const auto korInputs = CPortReader::ReadPorts(
    CRte::ERunnableIds::eRunnableBarometerMonitorRun,
    CRte::GetInstance().oPortCompensatedPressureInput1_,
    CRte::GetInstance().oPortCompensatedPressureInput2_,
    CRte::GetInstance().oPortCompensatedPressureInput3_);

  if (korInputs.has_value())
  {
    const auto& korPressureData1 = std::get<0>(korInputs.value());
    const auto& korPressureData2 = std::get<1>(korInputs.value());
    const auto& korPressureData3 = std::get<2>(korInputs.value());

    populateRedundantPressureDataObject(korPressureData1, skeBarometerId1_, oRedundantInputData);
    populateRedundantPressureDataObject(korPressureData2, skeBarometerId2_, oRedundantInputData);
    populateRedundantPressureDataObject(korPressureData3, skeBarometerId3_, oRedundantInputData);

    oMonitorOutput = NBaroMonitorApi::BarometerMonitorRun(oRedundantInputData);

    auto safeOrInvalidPressure = [](const SBarometerMeasurement& korPressureData,
                                    NFusionLibCommon::ESensorId eSensorId,
                                    const NBaroMonitorApi::COutputData& korMonitorOutput)
    {
      SBarometerMeasurement oOutputData{};

      if (UintToBool(korPressureData.uValid_) && CBaroMonitorSwc::isPressureMeasurementSafe(eSensorId, korMonitorOutput))
      {
        oOutputData = korPressureData;
      }

      return oOutputData;
    };

    SBarometerMeasurement oTmpMeasurement{};

    oTmpMeasurement = safeOrInvalidPressure(korPressureData1, skeBarometerId1_, oMonitorOutput);
    CRte::GetInstance().oPortSafePressureData1_.Write(oTmpMeasurement);

    oTmpMeasurement = safeOrInvalidPressure(korPressureData2, skeBarometerId2_, oMonitorOutput);
    CRte::GetInstance().oPortSafePressureData2_.Write(oTmpMeasurement);

    oTmpMeasurement = safeOrInvalidPressure(korPressureData3, skeBarometerId3_, oMonitorOutput);
    CRte::GetInstance().oPortSafePressureData3_.Write(oTmpMeasurement);
  }
  else
  {
    // Invalidate the ports
    SBarometerMeasurement oInvalidMeasurement{};
    CRte::GetInstance().oPortSafePressureData1_.Write(oInvalidMeasurement);
    CRte::GetInstance().oPortSafePressureData2_.Write(oInvalidMeasurement);
    CRte::GetInstance().oPortSafePressureData3_.Write(oInvalidMeasurement);
  }
}

bool CBaroMonitorSwc::isPressureMeasurementSafe(NFusionLibCommon::ESensorId eSensorId, const NBaroMonitorApi::COutputData& korMonitorOutput)
{
  AMS_HARD_ASSERT(CRedundantPressureData::IsSensorSupported(eSensorId));

  using EDetectionResult = NBaroMonitorApi::COutputData::EDetectionResult;
  using EIsolationResult = NBaroMonitorApi::COutputData::EIsolationResult;

  bool bIsSignalSafe{ true };
  const auto& korSignal = korMonitorOutput.GetSignal(NBaroMonitorApi::COutputData::EScalarSignals::ePressure);

  if (false == ((korSignal.eDetectionResults_ == EDetectionResult::eGood) ||
                ((korSignal.eDetectionResults_ == EDetectionResult::eFailure) &&
                 (korSignal.eIsolationResults_ == EIsolationResult::eGood) &&
                 (korSignal.eIsolatedSensor_ != eSensorId))))
  {
    bIsSignalSafe = false;
  }

  return bIsSignalSafe;
}

void CBaroMonitorSwc::populateRedundantPressureDataObject(const SBarometerMeasurement& korPressureData,
                                                          NFusionLibCommon::ESensorId eSensorId,
                                                          CRedundantPressureData& orRedundantPressureData)
{
  if (UintToBool(korPressureData.uValid_))
  {
    NFusionLibCommon::SBarometerData oTmpObject;

    oTmpObject.bValid_ = true;
    oTmpObject.eSensorId_ = eSensorId;
    oTmpObject.uTimestampUs_ = korPressureData.uTimestampUs_;
    oTmpObject.fPressure_ = korPressureData.fPressure_;

    orRedundantPressureData.Set(oTmpObject);
  }
}

