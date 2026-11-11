/// @file CImuPreFilterSwc.cpp
/// @brief Implementation of the IMU pre-filter SWC class.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CImuPreFilterSwc.h"

#include "CRte.h"
#include "AmsAssert.h"
#include "UintToBool.h"

CImuPreFilterSwc& CImuPreFilterSwc::getInstanceImpl(unsigned uInstanceIndex)
{
  static CImuPreFilterSwc saoImuPreFilters[skuInstanceCount_]
  {
    CImuPreFilterSwc(getFilterConfiguration<ESensorId::eImu1>()),
    CImuPreFilterSwc(getFilterConfiguration<ESensorId::eImu2>()),
    CImuPreFilterSwc(getFilterConfiguration<ESensorId::eImu3>())
  };

  AMS_HARD_ASSERT(uInstanceIndex < skuInstanceCount_);
  return saoImuPreFilters[uInstanceIndex];
}

CImuPreFilterSwc::CImuPreFilterSwc(SFilterConfiguration oFilterConfig)
  : keSensorId_{oFilterConfig.eSensorId}
  , kfCutoffFrequencyHz_{oFilterConfig.fCutoffFrequencyHz}
{
}

void CImuPreFilterSwc::Init()
{
  if (false == bIsInitialized_)
  {
    oFilterSpecificForceX_.Configure(kfCutoffFrequencyHz_, skfSamplingFrequencyHz_);
    oFilterSpecificForceY_.Configure(kfCutoffFrequencyHz_, skfSamplingFrequencyHz_);
    oFilterSpecificForceZ_.Configure(kfCutoffFrequencyHz_, skfSamplingFrequencyHz_);
    bIsInitialized_ = true;
  }
}

bool CImuPreFilterSwc::IsInitialized()
{
  return bIsInitialized_;
}

void CImuPreFilterSwc::Run()
{
  SImuMeasurement oInput{};
  SImuMeasurement oOutput{};

  if (true == bIsInitialized_)
  {
    const bool bStatus = readImuDataPort(oInput);

    if ((true == bStatus) && (true == UintToBool(oInput.uImuValid_)))
    {
      if (uInvalidInputCounter_ >= skuMaxInvalidInputCount_)
      {
        resetFilters();
      }

      uInvalidInputCounter_ = 0U;

      oFilterSpecificForceX_.Add(oInput.fSpecificForceX_);
      oFilterSpecificForceY_.Add(oInput.fSpecificForceY_);
      oFilterSpecificForceZ_.Add(oInput.fSpecificForceZ_);

      oOutput.fSpecificForceX_ = oFilterSpecificForceX_.Get();
      oOutput.fSpecificForceY_ = oFilterSpecificForceY_.Get();
      oOutput.fSpecificForceZ_ = oFilterSpecificForceZ_.Get();
      oOutput.fAngularRateX_ = oInput.fAngularRateX_;
      oOutput.fAngularRateY_ = oInput.fAngularRateY_;
      oOutput.fAngularRateZ_ = oInput.fAngularRateZ_;

      oOutput.fTemperature_ = oInput.fTemperature_;
      oOutput.uTemperatureValid_ = oInput.uTemperatureValid_;

      oOutput.uTimestampUs_ = oInput.uTimestampUs_;
      oOutput.uImuValid_ = BoolToUint(true);
    }
    else
    {
      if (uInvalidInputCounter_ < std::numeric_limits<decltype(uInvalidInputCounter_)>::max())
      {
        ++uInvalidInputCounter_;
      }
    }
  }

  writeFilteredImuDataPort(oOutput);
}

bool CImuPreFilterSwc::readImuDataPort(SImuMeasurement& orInput) const
{
  bool bStatus = false;

  switch (keSensorId_)
  {
    case ESensorId::eImu1:
      bStatus = CRte::GetInstance().oPortImuInput1_.Read(orInput);
      break;
    case ESensorId::eImu2:
      bStatus = CRte::GetInstance().oPortImuInput2_.Read(orInput);
      break;
    case ESensorId::eImu3:
      bStatus = CRte::GetInstance().oPortImuInput3_.Read(orInput);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }

  return bStatus;
}

void CImuPreFilterSwc::writeFilteredImuDataPort(const SImuMeasurement& orOutput) const
{
  switch (keSensorId_)
  {
    case ESensorId::eImu1:
      CRte::GetInstance().oPortImuInputFiltered1_.Write(orOutput);
      break;
    case ESensorId::eImu2:
      CRte::GetInstance().oPortImuInputFiltered2_.Write(orOutput);
      break;
    case ESensorId::eImu3:
      CRte::GetInstance().oPortImuInputFiltered3_.Write(orOutput);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }
}

void CImuPreFilterSwc::resetFilters()
{
  oFilterSpecificForceX_.Reset();
  oFilterSpecificForceY_.Reset();
  oFilterSpecificForceZ_.Reset();
}

extern "C" void ImuPreFilterSwcRun(uint32_t uSensorIndex)
{
  AMS_HARD_ASSERT(uSensorIndex < CImuPreFilterSwc::skuInstanceCount_);
  CImuPreFilterSwc::GetInstance(uSensorIndex).Run();
}
