/**
 * @file CBahrsFilerSwc.cpp
 * @brief Implementation of the BAHRS filter software component
 * @author Fedor Baklanov
 * @date 08 June 2022
 */

#include "CBahrsFilterSwc.h"
#include "UintToBool.h"

#include "CRte.h"

CBahrsFilterSwc::CBahrsFilterSwc()
{
#ifndef _MSC_VER
  pMutexHandle_ = osMutexNew(&sMutexAttributes_);

  if (NULL == pMutexHandle_)
  {
    assert(false);
  }
#endif /* _MSC_VER */
}

void CBahrsFilterSwc::Init()
{
  // Do nothing
}

bool CBahrsFilterSwc::IsInitialized()
{
  return true;
}

CBahrsFilterSwc& CBahrsFilterSwc::getInstanceImpl(unsigned uInstanceIndex)
{
  static CBahrsFilterSwc soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

void CBahrsFilterSwc::SetImuInput()
{
  SImuDataScha63T oImuData;
  bool bReadStatus = CRte::GetInstance().oScha63TDriverPort_.Read(oImuData);

  if ((true == bReadStatus) && (true == UintToBool(oImuData.uValid_)))
  {
    NFusionLibCommon::SImuMeasurement oBahrsImuData;

    oBahrsImuData.uTimestampUs_ = oImuData.uTimestampUs_;

    oBahrsImuData.fSpecificForceX_ = oImuData.fSpecificForceX_;
    oBahrsImuData.fSpecificForceY_ = oImuData.fSpecificForceY_;
    oBahrsImuData.fSpecificForceZ_ = oImuData.fSpecificForceZ_;
    oBahrsImuData.fAngularRateX_ = oImuData.fAngularRateX_;
    oBahrsImuData.fAngularRateY_ = oImuData.fAngularRateY_;
    oBahrsImuData.fAngularRateZ_ = oImuData.fAngularRateZ_;
    oBahrsImuData.eSensorId_ = NFusionLibCommon::ESensorId::eScha63T;
    oBahrsImuData.bValid_ = true;

#ifndef _MSC_VER
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      NBahrsFilterApi::BahrsFilterSetInput(oBahrsImuData, 0);
      osMutexRelease(pMutexHandle_);
    }
#else
    NBahrsFilterApi::BahrsFilterSetInput(oBahrsImuData, 0);
#endif /* _MSC_VER */
  }
}

void CBahrsFilterSwc::SetPressureInput()
{
  SBarometerMeasurement oPressureData;
  bool bReadStatus = CRte::GetInstance().oPortBmp384Input_.Read(oPressureData);

  if ((true == bReadStatus) && (true == UintToBool(oPressureData.uValid_)))
  {
    NFusionLibCommon::SBarometerData oBahrsPressureData;

    oBahrsPressureData.uTimestampUs_ = oPressureData.uTimestampUs_;
    oBahrsPressureData.fPressure_ = oPressureData.fPressure_;
    oBahrsPressureData.eSensorId_ = NFusionLibCommon::ESensorId::eBmp384;
    oBahrsPressureData.bValid_ = true;

#ifndef _MSC_VER
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      NBahrsFilterApi::BahrsFilterSetInput(oBahrsPressureData, 0);
      osMutexRelease(pMutexHandle_);
    }
#else
    NBahrsFilterApi::BahrsFilterSetInput(oBahrsPressureData, 0);
#endif /* _MSC_VER */
  }
}

void CBahrsFilterSwc::Step(uint64_t uTimestampUs)
{
#ifndef _MSC_VER
  osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

  if (osOK == eStatus)
  {
    NBahrsFilterApi::BahrsFilterPrepareInputs(0);
    osMutexRelease(pMutexHandle_);
  }
#else
  NBahrsFilterApi::BahrsFilterPrepareInputs(0);
#endif /* _MSC_VER */

  NBahrsFilterApi::BahrsFilterStep(uTimestampUs, 0);

#ifndef _MSC_VER
  eStatus = osMutexAcquire(pMutexHandle_, 1);

  if (osOK == eStatus)
  {
    NBahrsFilterApi::BahrsFilterCompleteEpoch(0);
    osMutexRelease(pMutexHandle_);
  }
#else
  NBahrsFilterApi::BahrsFilterCompleteEpoch(0);
#endif /* _MSC_VER */

  CRte::GetInstance().oPortBahrsFilterOutput_.Write(NBahrsFilterApi::BahrsFilterGetOutput(0));
}

