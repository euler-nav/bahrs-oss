/// @file CPressureCompensator.cpp
/// @brief Implementation of the pressure compensator SWC class.
/// @copyright Copyright 2026 AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CPressureCompensator.h"

#include "CRte.h"
#include "AmsAssert.h"
#include "UintToBool.h"

CPressureCompensator& CPressureCompensator::getInstanceImpl(unsigned uInstanceIndex)
{
  static CPressureCompensator saoPressureCompensators[skuInstanceCount_]
  {
    CPressureCompensator(ESensorId::ePressureInput1),
    CPressureCompensator(ESensorId::ePressureInput2),
    CPressureCompensator(ESensorId::ePressureInput3)
  };

  AMS_HARD_ASSERT(uInstanceIndex < skuInstanceCount_);
  return saoPressureCompensators[uInstanceIndex];
}

CPressureCompensator::CPressureCompensator(ESensorId eSensorId)
  : keSensorId_{eSensorId}
{
  AMS_HARD_ASSERT(toIndex(keSensorId_) < skuInstanceCount_);
}

void CPressureCompensator::Init()
{
  // Placeholder for the open-source variant: no NVM driver is included, so compensation
  // parameters are not loaded here. A user may hard-code inter-barometer offsets,
  // implement their own NVM-based loading, or use the Basic EULER-NAV BAHRS software license.
  fPressureOffsetPa_ = 0.0F;
  bIsInitialized_ = true;
}

bool CPressureCompensator::IsInitialized()
{
  return bIsInitialized_;
}

void CPressureCompensator::CompensateMeasurements()
{
  SBarometerMeasurement oInput{};
  SBarometerMeasurement oOutput{};

  if (true == bIsInitialized_)
  {
    const bool bStatus = readPressureDataPort(oInput);

    if ((true == bStatus) && (true == UintToBool(oInput.uValid_)))
    {
      oOutput = oInput;
      oOutput.fPressure_ += fPressureOffsetPa_;
    }
  }

  writePressureDataPort(oOutput);
}

bool CPressureCompensator::readPressureDataPort(SBarometerMeasurement& orInput) const
{
  bool bStatus = false;

  switch (keSensorId_)
  {
    case ESensorId::ePressureInput1:
      bStatus = CRte::GetInstance().oPortPressureInput1_.Read(orInput);
      break;
    case ESensorId::ePressureInput2:
      bStatus = CRte::GetInstance().oPortPressureInput2_.Read(orInput);
      break;
    case ESensorId::ePressureInput3:
      bStatus = CRte::GetInstance().oPortPressureInput3_.Read(orInput);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }

  return bStatus;
}

void CPressureCompensator::writePressureDataPort(const SBarometerMeasurement& orOutput) const
{
  switch (keSensorId_)
  {
    case ESensorId::ePressureInput1:
      CRte::GetInstance().oPortCompensatedPressureInput1_.Write(orOutput);
      break;
    case ESensorId::ePressureInput2:
      CRte::GetInstance().oPortCompensatedPressureInput2_.Write(orOutput);
      break;
    case ESensorId::ePressureInput3:
      CRte::GetInstance().oPortCompensatedPressureInput3_.Write(orOutput);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }
}

extern "C" void PressureCompensatorSwcCompensate(uint32_t uSensorIndex)
{
  AMS_HARD_ASSERT(uSensorIndex < CPressureCompensator::skuInstanceCount_);
  CPressureCompensator::GetInstance(uSensorIndex).CompensateMeasurements();
}
