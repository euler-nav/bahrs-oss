/// @file CBahrsFilterSwc.cpp
/// @brief Implementation of the BAHRS filter software component
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CBahrsFilterSwc.h"
#include "UintToBool.h"
#include "AmsAssert.h"

#include "CRte.h"

#ifdef SEND_DEBUG_OUTPUT

#include "GetMicroseconds.h"
#ifndef _MSC_VER
#include "CRs232OutputHandler.h"
using CDebugOutputHandler = CRs232OutputHandler;
#else
#include "CFileDebugOutputHandler.h"
using CDebugOutputHandler = CFileDebugOutputHandler;
#endif // _MSC_VER

#endif // SEND_DEBUG_OUTPUT

namespace NBahrsFilterApi
{
#ifdef BAHRS_HW_V3
using namespace ::NBahrsFilterApi::NBahrsV3;
#elif defined(BAHRS_HW_V2)
using namespace ::NBahrsFilterApi::NBahrsV2;
#else
#error "BAHRS hardware version is not defined"
#endif
} // namespace NBahrsFilterApi

CBahrsFilterSwc::CBahrsFilterSwc(uint32_t uInstanceIndex) :
  kuInstanceIndex_(uInstanceIndex)
{
  AMS_HARD_ASSERT(uInstanceIndex < skuInstanceCount_);

#ifndef _MSC_VER
  pMutexHandle_ = osMutexNew(&sMutexAttributes_);

  if (NULL == pMutexHandle_)
  {
    AMS_HARD_ASSERT(false);
  }
#endif // _MSC_VER
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
  AMS_HARD_ASSERT(uInstanceIndex < skuInstanceCount_);

  static CBahrsFilterSwc soInstance1{ 0U };
  static CBahrsFilterSwc soInstance2{ 1U };
  static CBahrsFilterSwc soInstance3{ 2U };

  CBahrsFilterSwc* opFilter{ nullptr };

  switch (uInstanceIndex)
  {
    case 0U:
      opFilter = &soInstance1;
      break;
    case 1U:
      opFilter = &soInstance2;
      break;
    case 2U:
      opFilter = &soInstance3;
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }

  return *opFilter;
}

void CBahrsFilterSwc::SetImuInput()
{
  SImuMeasurement oImuData;
  NFusionLibCommon::ESensorId eSensorId;
  bool bReadStatus = readImuDataPort(oImuData, eSensorId);


  if ((true == bReadStatus) && (true == UintToBool(oImuData.uImuValid_)))
  {
    NFusionLibCommon::SImuMeasurement oBahrsImuData;

    oBahrsImuData.uTimestampUs_ = oImuData.uTimestampUs_;
    oBahrsImuData.eSensorId_ = eSensorId;
    oBahrsImuData.fSpecificForceX_ = oImuData.fSpecificForceX_;
    oBahrsImuData.fSpecificForceY_ = oImuData.fSpecificForceY_;
    oBahrsImuData.fSpecificForceZ_ = oImuData.fSpecificForceZ_;
    oBahrsImuData.fAngularRateX_ = oImuData.fAngularRateX_;
    oBahrsImuData.fAngularRateY_ = oImuData.fAngularRateY_;
    oBahrsImuData.fAngularRateZ_ = oImuData.fAngularRateZ_;
    oBahrsImuData.bValid_ = true;

#ifndef _MSC_VER
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      NBahrsFilterApi::BahrsFilterSetInput(oBahrsImuData, kuInstanceIndex_);
      osMutexRelease(pMutexHandle_);
    }
#else
    NBahrsFilterApi::BahrsFilterSetInput(oBahrsImuData, kuInstanceIndex_);
#endif // _MSC_VER
  }
}

extern "C" void BahrsFilterSwcSetImuInput(uint32_t uFilterIndex)
{
  AMS_HARD_ASSERT(uFilterIndex < CBahrsFilterSwc::skuInstanceCount_);
  CBahrsFilterSwc::GetInstance(uFilterIndex).SetImuInput();
}

void CBahrsFilterSwc::SetPressureInput()
{
  SBarometerMeasurement oPressureData;
  NFusionLibCommon::ESensorId eSensorId;
  bool bReadStatus = readPressureDataPort(oPressureData, eSensorId);

  if ((true == bReadStatus) && (true == UintToBool(oPressureData.uValid_)))
  {
    NFusionLibCommon::SBarometerData oBahrsPressureData;

    oBahrsPressureData.uTimestampUs_ = oPressureData.uTimestampUs_;
    oBahrsPressureData.fPressure_ = oPressureData.fPressure_;
    oBahrsPressureData.eSensorId_ = eSensorId;
    oBahrsPressureData.bValid_ = true;

#ifndef _MSC_VER
    osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

    if (osOK == eStatus)
    {
      NBahrsFilterApi::BahrsFilterSetInput(oBahrsPressureData, kuInstanceIndex_);
      osMutexRelease(pMutexHandle_);
    }
#else
    NBahrsFilterApi::BahrsFilterSetInput(oBahrsPressureData, kuInstanceIndex_);
#endif // _MSC_VER
  }
}

void CBahrsFilterSwc::Step(uint64_t uTimestampUs)
{
#ifndef _MSC_VER
  osStatus_t eStatus = osMutexAcquire(pMutexHandle_, 1);

  if (osOK == eStatus)
  {
    NBahrsFilterApi::BahrsFilterPrepareInputs(kuInstanceIndex_);
    osMutexRelease(pMutexHandle_);
  }
#else
  NBahrsFilterApi::BahrsFilterPrepareInputs(kuInstanceIndex_);
#endif // _MSC_VER

#ifdef SEND_DEBUG_OUTPUT
  {
    CSerialProtocol::SDebugData oDebugData;
    oDebugData.uMsgType_ = CSerialProtocol::EMessageIds::eDebugEventRunnableCall;
    oDebugData.uId_ = getRunnableIdForDebug();
    oDebugData.uTimestampUs_ = GetMicroseconds();
#ifndef _MSC_VER
    oDebugData.bAllPortsLocked_ = BoolToUint(osOK == eStatus);
#else
    oDebugData.bAllPortsLocked_ = BoolToUint(true);
#endif // _MSC_VER
    CDebugOutputHandler::GetInstance().SendDebugData(oDebugData);
  }
#endif // SEND_DEBUG_OUTPUT

  NBahrsFilterApi::BahrsFilterStep(uTimestampUs, kuInstanceIndex_);

#ifndef _MSC_VER
  eStatus = osMutexAcquire(pMutexHandle_, 1);

  if (osOK == eStatus)
  {
    NBahrsFilterApi::BahrsFilterCompleteEpoch(kuInstanceIndex_);
    osMutexRelease(pMutexHandle_);
  }
#else
  NBahrsFilterApi::BahrsFilterCompleteEpoch(kuInstanceIndex_);
#endif // _MSC_VER

  CBahrsFilterOutput oOutput;
  static_cast<NBahrsFilterApi::SOutputData&>(oOutput) = NBahrsFilterApi::BahrsFilterGetOutput(kuInstanceIndex_);

  writeOutputToPort(oOutput);
}

void CBahrsFilterSwc::writeOutputToPort(const CBahrsFilterOutput& korFilterOutput) const
{
  switch (kuInstanceIndex_)
  {
    case 0U:
      CRte::GetInstance().oPortBahrsFilterOutput1_.Write(korFilterOutput);
      break;
    case 1U:
      CRte::GetInstance().oPortBahrsFilterOutput2_.Write(korFilterOutput);
      break;
    case 2U:
      CRte::GetInstance().oPortBahrsFilterOutput3_.Write(korFilterOutput);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }
}

namespace
{

template<typename TPortType>
bool readPort(CRte::ERunnableIds eRunnableId, TPortType& orPort, typename TPortType::DataType& orPortData)
{
  bool bStatus{ false };
  const auto koInput = CPortReader::ReadPorts(eRunnableId, orPort);

  if (koInput.has_value())
  {
    orPortData = std::get<0>(koInput.value());
    bStatus = true;
  }

  return bStatus;
}

} // anonymous namespace

bool CBahrsFilterSwc::readImuDataPort(SImuMeasurement& orImuData, NFusionLibCommon::ESensorId& erSensorId) const
{
  bool bStatus = false;
  using ERunnableIds = CRte::ERunnableIds;

  switch (kuInstanceIndex_)
  {
    case 0U:
      bStatus = readPort(ERunnableIds::eRunnableBahrsFilterSetImuInput1, CRte::GetInstance().oPortImuInputFiltered1_, orImuData);
      erSensorId = skeImuId1_;
      break;
    case 1U:
      bStatus = readPort(ERunnableIds::eRunnableBahrsFilterSetImuInput2, CRte::GetInstance().oPortImuInputFiltered2_, orImuData);
      erSensorId = skeImuId2_;
      break;
    case 2U:
      bStatus = readPort(ERunnableIds::eRunnableBahrsFilterSetImuInput3, CRte::GetInstance().oPortImuInputFiltered3_, orImuData);
      erSensorId = skeImuId3_;
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }

  return bStatus;
}

bool CBahrsFilterSwc::readPressureDataPort(SBarometerMeasurement& orPressureData, NFusionLibCommon::ESensorId& erSensorId) const
{
  bool bStatus = false;

  switch (kuInstanceIndex_)
  {
    case 0U:
      bStatus = readPort(CRte::ERunnableIds::eRunnableBahrsFilterSetPressureInput1, CRte::GetInstance().oPortSafePressureData1_, orPressureData);
      erSensorId = skeBaroId1_;
      break;
    case 1U:
      bStatus = readPort(CRte::ERunnableIds::eRunnableBahrsFilterSetPressureInput2, CRte::GetInstance().oPortSafePressureData2_, orPressureData);
      erSensorId = skeBaroId2_;
      break;
    case 2U:
      bStatus = readPort(CRte::ERunnableIds::eRunnableBahrsFilterSetPressureInput3, CRte::GetInstance().oPortSafePressureData3_, orPressureData);
      erSensorId = skeBaroId3_;
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  }

  return bStatus;
}

#ifdef SEND_DEBUG_OUTPUT

uint8_t CBahrsFilterSwc::getRunnableIdForDebug() const
{
  uint8_t eRunnableId{ static_cast<uint8_t>(CRte::ERunnableIds::eInvalid) };

  switch (kuInstanceIndex_)
  {
    case 0U:
      eRunnableId = static_cast<uint8_t>(CRte::ERunnableIds::eRunnableBahrsFilterStep1);
      break;
    case 1U:
      eRunnableId = static_cast<uint8_t>(CRte::ERunnableIds::eRunnableBahrsFilterStep2);
      break;
    case 2U:
      eRunnableId = static_cast<uint8_t>(CRte::ERunnableIds::eRunnableBahrsFilterStep3);
      break;
    default:
      AMS_HARD_ASSERT(false);
      break;
  };

  return eRunnableId;
}

#endif // SEND_DEBUG_OUTPUT
