/**
 * @file CBahrsFilerSwc.cpp
 * @brief Implementation of the BAHRS filter software component
 * @author Fedor Baklanov
 * @date 08 June 2022
 */

#include "CImuMonitorSwc.h"
#include "UintToBool.h"

#include "CRte.h"

#ifdef SEND_DEBUG_OUTPUT

#include "GetMicroseconds.h"

#endif /* SEND_DEBUG_OUTPUT */

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
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CImuMonitorSwc::Run()
{
  SImuDataScha63T oImuData1;
  SImuMeasurement oImuData2, oImuData3;
  NImuMonitorApi::CRedundantInputData oRedundantImuData;

  if (true == CRte::GetInstance().oScha63TDriverPort_.Read(oImuData1))
  {
    if (UintToBool(oImuData1.uValid_))
    {
      NFusionLibCommon::SImuMeasurement oTmpObject;

      oTmpObject.bValid_ = true;
      oTmpObject.eSensorId_ = NFusionLibCommon::ESensorId::eScha63T;
      oTmpObject.uTimestampUs_ = oImuData1.uTimestampUs_;
      oTmpObject.fAngularRateX_ = oImuData1.fAngularRateX_;
      oTmpObject.fAngularRateY_ = oImuData1.fAngularRateY_;
      oTmpObject.fAngularRateZ_ = oImuData1.fAngularRateZ_;
      oTmpObject.fSpecificForceX_ = oImuData1.fSpecificForceX_;
      oTmpObject.fSpecificForceY_ = oImuData1.fSpecificForceY_;
      oTmpObject.fSpecificForceZ_ = oImuData1.fSpecificForceZ_;

      oRedundantImuData.Set(oTmpObject);
    }
  }

  if (true == CRte::GetInstance().oPortIcm20789ImuInput1_.Read(oImuData2))
  {
    if (UintToBool(oImuData2.uValid_))
    {
      NFusionLibCommon::SImuMeasurement oTmpObject;

      oTmpObject.bValid_ = true;
      oTmpObject.eSensorId_ = NFusionLibCommon::ESensorId::eIcm20789Imu1;
      oTmpObject.uTimestampUs_ = oImuData2.uTimestampUs_;
      oTmpObject.fAngularRateX_ = oImuData2.fAngularRateX_;
      oTmpObject.fAngularRateY_ = oImuData2.fAngularRateY_;
      oTmpObject.fAngularRateZ_ = oImuData2.fAngularRateZ_;
      oTmpObject.fSpecificForceX_ = oImuData2.fSpecificForceX_;
      oTmpObject.fSpecificForceY_ = oImuData2.fSpecificForceY_;
      oTmpObject.fSpecificForceZ_ = oImuData2.fSpecificForceZ_;

      oRedundantImuData.Set(oTmpObject);
    }
  }

  if (true == CRte::GetInstance().oPortIcm20789ImuInput2_.Read(oImuData3))
  {
    if (UintToBool(oImuData3.uValid_))
    {
      NFusionLibCommon::SImuMeasurement oTmpObject;

      oTmpObject.bValid_ = true;
      oTmpObject.eSensorId_ = NFusionLibCommon::ESensorId::eIcm20789Imu2;
      oTmpObject.uTimestampUs_ = oImuData3.uTimestampUs_;
      oTmpObject.fAngularRateX_ = oImuData3.fAngularRateX_;
      oTmpObject.fAngularRateY_ = oImuData3.fAngularRateY_;
      oTmpObject.fAngularRateZ_ = oImuData3.fAngularRateZ_;
      oTmpObject.fSpecificForceX_ = oImuData3.fSpecificForceX_;
      oTmpObject.fSpecificForceY_ = oImuData3.fSpecificForceY_;
      oTmpObject.fSpecificForceZ_ = oImuData3.fSpecificForceZ_;

      oRedundantImuData.Set(oTmpObject);
    }
  }

  NImuMonitorApi::COutputData oDataAfterMonitor = NImuMonitorApi::ImuMonitorRun(oRedundantImuData);

  CRte::GetInstance().oPortImuDataAfterMonitor_.Write(oDataAfterMonitor);
}

