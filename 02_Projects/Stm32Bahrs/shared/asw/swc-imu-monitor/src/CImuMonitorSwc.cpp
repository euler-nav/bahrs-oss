/**
 * @file CImuMonitorSwc.cpp
 * @brief Implementation of the BAHRS filter software component
 * @author Fedor Baklanov
 * @date 08 June 2022
 */

#include "CImuMonitorSwc.h"
#include "UintToBool.h"
#include "AmsAssert.h"

void CImuMonitorSwc::Init()
{
  // Do nothing
}

bool CImuMonitorSwc::IsInitialized()
{
  return true;
}

CImuMonitorSwc& CImuMonitorSwc::getInstanceImpl(unsigned uInstanceIndex)
{
  static CImuMonitorSwc soInstance;
  AMS_HARD_ASSERT(uInstanceIndex == 0U);
  return soInstance;
}

void CImuMonitorSwc::Run()
{
  CRedundantImuData oRedundantImuData{};
  CImuDataAfterMonitor oDataAfterMonitor{};

  const auto korInputs = CPortReader::ReadPorts(
    CRte::ERunnableIds::eRunnableImuMonitorRun,
    CRte::GetInstance().oPortImuInput1_,
    CRte::GetInstance().oPortImuInput2_,
    CRte::GetInstance().oPortImuInput3_);

  if (korInputs.has_value())
  {
    populateRedundantImuDataObject(std::get<0>(korInputs.value()), skeImuId1_, oRedundantImuData);
    populateRedundantImuDataObject(std::get<1>(korInputs.value()), skeImuId2_, oRedundantImuData);
    populateRedundantImuDataObject(std::get<2>(korInputs.value()), skeImuId3_, oRedundantImuData);
  
    static_cast<NImuMonitorApi::COutputData&>(oDataAfterMonitor) = NImuMonitorApi::ImuMonitorRun(oRedundantImuData);
  }

  CRte::GetInstance().oPortImuDataAfterMonitor_.Write(oDataAfterMonitor);
}

void CImuMonitorSwc::populateRedundantImuDataObject(const SImuMeasurement& korImuData, NFusionLibCommon::ESensorId eSensorId, CRedundantImuData& orRedundantImuData)
{
  if (UintToBool(korImuData.uImuValid_))
  {
    NFusionLibCommon::SImuMeasurement oTmpObject;

    oTmpObject.bValid_ = true;
    oTmpObject.eSensorId_ = eSensorId;
    oTmpObject.uTimestampUs_ = korImuData.uTimestampUs_;
    oTmpObject.fAngularRateX_ = korImuData.fAngularRateX_;
    oTmpObject.fAngularRateY_ = korImuData.fAngularRateY_;
    oTmpObject.fAngularRateZ_ = korImuData.fAngularRateZ_;
    oTmpObject.fSpecificForceX_ = korImuData.fSpecificForceX_;
    oTmpObject.fSpecificForceY_ = korImuData.fSpecificForceY_;
    oTmpObject.fSpecificForceZ_ = korImuData.fSpecificForceZ_;

    orRedundantImuData.Set(oTmpObject);
  }
}

