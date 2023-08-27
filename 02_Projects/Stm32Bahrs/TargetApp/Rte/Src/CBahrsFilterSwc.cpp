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
    NBahrsFilterApi::SImuData oBahrsImuData;

    oBahrsImuData.uTimestampUs_ = oImuData.uTimestampUs_;
    oBahrsImuData.oSpecificForce_(0) = oImuData.fSpecificForceX_;
    oBahrsImuData.oSpecificForce_(1) = oImuData.fSpecificForceY_;
    oBahrsImuData.oSpecificForce_(2) = oImuData.fSpecificForceZ_;
    oBahrsImuData.oAngularRate_(0) = oImuData.fAngularRateX_;
    oBahrsImuData.oAngularRate_(1) = oImuData.fAngularRateY_;
    oBahrsImuData.oAngularRate_(2) = oImuData.fAngularRateZ_;

#ifndef _MSC_VER
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      NBahrsFilterApi::BahrsFilterSetInput(oBahrsImuData);
      osMutexRelease(pMutexHandle_);
    }
#else
    NBahrsFilterApi::BahrsFilterSetInput(oBahrsImuData);
#endif /* _MSC_VER */
  }
}

void CBahrsFilterSwc::SetPressureInput()
{
  SBarometerMeasurement oPressureData;
  bool bReadStatus = CRte::GetInstance().oPortBmp384Input_.Read(oPressureData);

  if ((true == bReadStatus) && (true == UintToBool(oPressureData.uValid_)))
  {
    NBahrsFilterApi::SPressureData oBahrsPressureData;

    oBahrsPressureData.uTimestampUs_ = oPressureData.uTimestampUs_;
    oBahrsPressureData.fPressureInPascals = oPressureData.fPressure_;

#ifndef _MSC_VER
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      NBahrsFilterApi::BahrsFilterSetInput(oBahrsPressureData);
      osMutexRelease(pMutexHandle_);
    }
#else
    NBahrsFilterApi::BahrsFilterSetInput(oBahrsPressureData);
#endif /* _MSC_VER */
  }
}

void CBahrsFilterSwc::Step(uint64_t uTimestampUs)
{
#ifndef _MSC_VER
  osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

  if (osOK == eStatus)
  {
    NBahrsFilterApi::BahrsFilterPrepareInputs();
    osMutexRelease(pMutexHandle_);
  }
#else
  NBahrsFilterApi::BahrsFilterPrepareInputs();
#endif /* _MSC_VER */

  NBahrsFilterApi::BahrsFilterStep(uTimestampUs);

#ifndef _MSC_VER
  eStatus = osMutexAcquire(pMutexHandle_, 1);

  if (osOK == eStatus)
  {
    NBahrsFilterApi::BahrsFilterCompleteEpoch();
    osMutexRelease(pMutexHandle_);
  }
#else
  NBahrsFilterApi::BahrsFilterCompleteEpoch();
#endif /* _MSC_VER */

  CRte::GetInstance().oPortBahrsFilterOutput_.Write(NBahrsFilterApi::BahrsFilterGetOutput());
}

