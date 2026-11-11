/// @file CImuMonitorSwc.cpp
/// @brief Implementation of the BAHRS filter software component
/// @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CImuMonitorSwc.h"
#include "UintToBool.h"
#include "AmsAssert.h"
#include "CRte.h"

#ifdef SEND_DEBUG_OUTPUT

#include "GetMicroseconds.h"

#endif // SEND_DEBUG_OUTPUT

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
    CRte::GetInstance().oPortImuInputFiltered1_,
    CRte::GetInstance().oPortImuInputFiltered2_,
    CRte::GetInstance().oPortImuInputFiltered3_);

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

