/// @file CMagneticHeadingFilterSwc.cpp
/// @brief Implementation of the magnetic heading filter software component.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CMagneticHeadingFilterSwc.h"
#include "UintToBool.h"
#include "AmsAssert.h"

#include "CRte.h"

CMagneticHeadingFilterSwc& CMagneticHeadingFilterSwc::getInstanceImpl(unsigned uInstanceIndex)
{
  static CMagneticHeadingFilterSwc saoMagneticHeadingFilters[skuInstanceCount_]
  {
    CMagneticHeadingFilterSwc(getFilterConfiguration<ESensorId::eMagnetometer1>()),
    CMagneticHeadingFilterSwc(getFilterConfiguration<ESensorId::eMagnetometer2>()),
    CMagneticHeadingFilterSwc(getFilterConfiguration<ESensorId::eMagnetometer3>())
  };

  AMS_HARD_ASSERT(uInstanceIndex < skuInstanceCount_);
  return saoMagneticHeadingFilters[uInstanceIndex];
}

CMagneticHeadingFilterSwc::CMagneticHeadingFilterSwc(SFilterConfiguration oFilterConfig)
  : keSensorId_{oFilterConfig.eSensorId}
  , keFusionLibSensorId_{oFilterConfig.eFusionLibSensorId}
{
  AMS_HARD_ASSERT(toIndex(keSensorId_) < skuInstanceCount_);

#ifndef _MSC_VER
  pMutexHandle_ = osMutexNew(&sMutexAttributes_);

  if (NULL == pMutexHandle_)
  {
    AMS_HARD_ASSERT(false);
  }
#endif // _MSC_VER
}

void CMagneticHeadingFilterSwc::Init()
{
  // Do nothing
}

bool CMagneticHeadingFilterSwc::IsInitialized()
{
  return true;
}

void CMagneticHeadingFilterSwc::SetAttitudeInput()
{
  SSafeAttitudeData oAttitude;
  const bool kbReadStatus = CRte::GetInstance().oPortSafeVehicleAttitude_.Read(oAttitude);

  if ((true == kbReadStatus) && (CSerialProtocol::ESignalHealthInfo::eSafe == oAttitude.eHealth_))
  {
#ifndef _MSC_VER
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      NMagneticHeadingFilterApi::MagneticHeadingFilterSetInput(oAttitude.uTimestampUs_, oAttitude.fRoll_, oAttitude.fPitch_, toIndex(keSensorId_));
      osMutexRelease(pMutexHandle_);
    }
#else
    NMagneticHeadingFilterApi::MagneticHeadingFilterSetInput(oAttitude.uTimestampUs_, oAttitude.fRoll_, oAttitude.fPitch_, toIndex(keSensorId_));
#endif // _MSC_VER
  }
}

void CMagneticHeadingFilterSwc::Step(uint64_t uTimestampUs)
{
  SMagneticMeasurement oMagDataInVehicleFrame;
  const bool kbReadStatus = readMagnetometerDataPort(oMagDataInVehicleFrame);

  if ((true == kbReadStatus) && (true == UintToBool(oMagDataInVehicleFrame.uValid_)))
  {
    NFusionLibCommon::SMagnetometerData oMagDataInput;

    oMagDataInput.uTimestampUs_ = oMagDataInVehicleFrame.uTimestampUs_;
    oMagDataInput.oInductionVectorInGauss_(0) = oMagDataInVehicleFrame.fVectorX_;
    oMagDataInput.oInductionVectorInGauss_(1) = oMagDataInVehicleFrame.fVectorY_;
    oMagDataInput.oInductionVectorInGauss_(2) = oMagDataInVehicleFrame.fVectorZ_;
    oMagDataInput.eSensorId_ = keFusionLibSensorId_;
    oMagDataInput.bValid_ = true;

#ifndef _MSC_VER
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      NMagneticHeadingFilterApi::MagneticHeadingFilterPrepareInputs(toIndex(keSensorId_));
      osMutexRelease(pMutexHandle_);
    }
#else
    NMagneticHeadingFilterApi::MagneticHeadingFilterPrepareInputs(toIndex(keSensorId_));
#endif // _MSC_VER

    NMagneticHeadingFilterApi::MagneticHeadingFilterStep(uTimestampUs, oMagDataInput, toIndex(keSensorId_));
  }

  SMagneticHeading oMagneticHeading;
  const NMagneticHeadingFilterApi::SOutputData koFilterOutput = NMagneticHeadingFilterApi::MagneticHeadingFilterGetOutput(toIndex(keSensorId_));

  if (koFilterOutput.bValid_)
  {
    oMagneticHeading.fMagneticHeading_ = koFilterOutput.fMagneticHeading_;
    oMagneticHeading.uTimestampUs_ = koFilterOutput.uTimestampUs_;
    oMagneticHeading.uValid_ = BoolToUint(true);
  }

  writeOutputToPort(oMagneticHeading);
}

bool CMagneticHeadingFilterSwc::readMagnetometerDataPort(SMagneticMeasurement& orMagData) const
{
  bool bStatus = false;

  switch (keSensorId_)
  {
    case ESensorId::eMagnetometer1:
      bStatus = CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame1_.Read(orMagData);
      break;
    case ESensorId::eMagnetometer2:
      bStatus = CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame2_.Read(orMagData);
      break;
    case ESensorId::eMagnetometer3:
      bStatus = CRte::GetInstance().oPortCompensatedMagnetometerDataInVehicleFrame3_.Read(orMagData);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }

  return bStatus;
}

void CMagneticHeadingFilterSwc::writeOutputToPort(const SMagneticHeading& korOutput) const
{
  switch (keSensorId_)
  {
    case ESensorId::eMagnetometer1:
      CRte::GetInstance().oPortMagneticHeading1_.Write(korOutput);
      break;
    case ESensorId::eMagnetometer2:
      CRte::GetInstance().oPortMagneticHeading2_.Write(korOutput);
      break;
    case ESensorId::eMagnetometer3:
      CRte::GetInstance().oPortMagneticHeading3_.Write(korOutput);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }
}
