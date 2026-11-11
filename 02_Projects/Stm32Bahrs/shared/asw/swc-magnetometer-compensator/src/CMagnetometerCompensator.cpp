/// @file CMagnetometerCompensator.cpp
/// @brief Implementation of the magnetometer Compensator SWC class.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CRte.h"
#include "crc32.h"
#include "UintToBool.h"
#include "AmsAssert.h"

#ifndef _MSC_VER
#include "CNvmDriver.h"
#endif // _MSC_VER

CMagnetometerCompensator& CMagnetometerCompensator::getInstanceImpl(unsigned uInstanceIndex)
{
  static CMagnetometerCompensator saoMagnetometerCompensators[skuInstanceCount_]
  {
    CMagnetometerCompensator(ESensorId::eMagnetometer1),
    CMagnetometerCompensator(ESensorId::eMagnetometer2),
    CMagnetometerCompensator(ESensorId::eMagnetometer3)
  };

  AMS_HARD_ASSERT(uInstanceIndex < skuInstanceCount_);
  return saoMagnetometerCompensators[uInstanceIndex];
}

CMagnetometerCompensator::CMagnetometerCompensator(ESensorId eSensorId)
  : keSensorId_{eSensorId}
{
  AMS_HARD_ASSERT(toIndex(keSensorId_) < skuInstanceCount_);
}

#ifndef _MSC_VER

namespace
{

template<typename TBlockType, typename TParamType>
bool loadCalibrationParameters(Eigen::Vector3f& orHardIron, Eigen::Matrix3f& orSoftIron)
{
  TParamType oCalibParams{};
  bool bStatus = CNvmDriver::ReadBlock<TBlockType>(oCalibParams);

  if (true == bStatus)
  {
    Crc32_t const kuComputedCrc = crc32(reinterpret_cast<void*>(&oCalibParams),
                                        sizeof(oCalibParams) - sizeof(oCalibParams.uCrc_));
    bStatus = (kuComputedCrc == oCalibParams.uCrc_);
  }

  if (true == bStatus)
  {
    for (uint8_t i = 0U; i < 3U; ++i)
    {
      orHardIron(i) = oCalibParams.afHardIronOffset_[i];

      for (uint8_t j = 0U; j < 3U; ++j)
      {
        orSoftIron(i, j) = oCalibParams.afSoftIronDistortion_[i * 3U + j];
      }
    }
  }

  return bStatus;
}

} // anonymous namespace

#endif // _MSC_VER

void CMagnetometerCompensator::Init()
{
#ifndef _MSC_VER
  if (false == bIsInitialized_)
  {
    bool bStatus = false;

    switch (keSensorId_)
    {
      case ESensorId::eMagnetometer1:
        bStatus = loadCalibrationParameters<CNvmDriver::CNvmBlockMagnetometerCalibrationParameters1,
                                            CNvmDriver::SMagnetometerCalibrationParameters1>(oHardIron_, oSoftIron_);
        break;
      case ESensorId::eMagnetometer2:
        bStatus = loadCalibrationParameters<CNvmDriver::CNvmBlockMagnetometerCalibrationParameters2,
                                            CNvmDriver::SMagnetometerCalibrationParameters2>(oHardIron_, oSoftIron_);
        break;
      case ESensorId::eMagnetometer3:
        bStatus = loadCalibrationParameters<CNvmDriver::CNvmBlockMagnetometerCalibrationParameters3,
                                            CNvmDriver::SMagnetometerCalibrationParameters3>(oHardIron_, oSoftIron_);
        break;
      default:
        AMS_HARD_ASSERT(false);
        break;
    }

    if (true == bStatus)
    {
      bIsInitialized_ = true;
    }
  }
#else
  // For the logs eulernav_log_20260827_085843.bin and eulernav_log_20260827_084643.bin.
  static const float skafHardIron[skuInstanceCount_][3] =
  {
    { 0.258647195F, -0.0753456477F, -0.125805571F },
    { -0.0904932483F, -0.104845927F, -0.0647079805F },
    { 0.166511635F, -0.173837151F, -0.180666071F }
  };

  static const float skafSoftIron[skuInstanceCount_][9] =
  {
    { 1.05878768F, 0.00539076048F, -0.0210361456F,
      0.0173660208F, 1.02516287F, -0.0257869811F,
      -0.0310290304F, -0.0187905716F, 1.06706391F },
    { 1.04028331F, 0.0137697411F, -0.0182341422F,
      0.013906515F, 1.0294059F, -0.0280877705F,
      -0.0180423663F, -0.0275847887F, 1.08089051F },
    { 1.0665767F, -0.0224242516F, -0.00316720078F,
      -0.0103643533F, 1.03636832F, -0.0324852594F,
      -0.00647135347F, -0.0315736668F, 1.05743885F }
  };

  const uint32_t kuIndex = toIndex(keSensorId_);

  for (uint8_t i = 0U; i < 3U; ++i)
  {
    oHardIron_(i) = skafHardIron[kuIndex][i];

    for (uint8_t j = 0U; j < 3U; ++j)
    {
      oSoftIron_(i, j) = skafSoftIron[kuIndex][i * 3U + j];
    }
  }

  bIsInitialized_ = true;
#endif // _MSC_VER
}

bool CMagnetometerCompensator::IsInitialized()
{
  return bIsInitialized_;
}

void CMagnetometerCompensator::CompensateMeasurements()
{
  SMagneticMeasurement oMagDataIn{};
  SMagneticMeasurement oMagDataOut{};

  if (true == bIsInitialized_)
  {
    const bool bStatus = readMagnetometerDataPort(oMagDataIn);

    if ((true == bStatus) && (true == UintToBool(oMagDataIn.uValid_)))
    {
      const Eigen::Vector3f oMagneticField = oSoftIron_ * (Eigen::Vector3f(oMagDataIn.fVectorX_,
                                                                           oMagDataIn.fVectorY_,
                                                                           oMagDataIn.fVectorZ_) + oHardIron_);

      oMagDataOut.uValid_ = BoolToUint(true);
      oMagDataOut.uTimestampUs_ = oMagDataIn.uTimestampUs_;
      oMagDataOut.fVectorX_ = oMagneticField(0);
      oMagDataOut.fVectorY_ = oMagneticField(1);
      oMagDataOut.fVectorZ_ = oMagneticField(2);
    }
  }

  writeMagnetometerDataPort(oMagDataOut);
}

bool CMagnetometerCompensator::readMagnetometerDataPort(SMagneticMeasurement& orInput) const
{
  bool bStatus = false;

  switch (keSensorId_)
  {
    case ESensorId::eMagnetometer1:
      bStatus = CRte::GetInstance().oPortMagnetometerInput1_.Read(orInput);
      break;
    case ESensorId::eMagnetometer2:
      bStatus = CRte::GetInstance().oPortMagnetometerInput2_.Read(orInput);
      break;
    case ESensorId::eMagnetometer3:
      bStatus = CRte::GetInstance().oPortMagnetometerInput3_.Read(orInput);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }

  return bStatus;
}

void CMagnetometerCompensator::writeMagnetometerDataPort(const SMagneticMeasurement& orOutput) const
{
  switch (keSensorId_)
  {
    case ESensorId::eMagnetometer1:
      CRte::GetInstance().oPortCompensatedMagnetometerData1_.Write(orOutput);
      break;
    case ESensorId::eMagnetometer2:
      CRte::GetInstance().oPortCompensatedMagnetometerData2_.Write(orOutput);
      break;
    case ESensorId::eMagnetometer3:
      CRte::GetInstance().oPortCompensatedMagnetometerData3_.Write(orOutput);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }
}


