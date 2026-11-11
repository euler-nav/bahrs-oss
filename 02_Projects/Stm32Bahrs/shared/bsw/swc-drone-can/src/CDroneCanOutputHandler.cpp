/// @file CDroneCanOutputHandler.cpp
/// @brief Implementation of the CDroneCanOutputHandler class.
///
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

// The DroneCAN stack is a build variant: this translation unit is empty
// unless USE_DRONECAN is defined.
#ifdef USE_DRONECAN

#include "CDroneCanOutputHandler.h"
#include "CRte.h"
#include "UintToBool.h"
#include "NavigationUtilities.h"

#include <cmath>

static constexpr float skfCelsiusToKelvin { 273.15F };

// Definition of the static uptime counter.
uint32_t CDroneCanOutputHandler::suUptimeSec_ { 0U };

// ---------------------------------------------------------------------------
// Singleton
// ---------------------------------------------------------------------------

CDroneCanOutputHandler& CDroneCanOutputHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  static CDroneCanOutputHandler soInstance;
  assert(uInstanceIndex == 0U);
  return soInstance;
}

const CDroneCanOutputHandler::CFrameHandleArray& CDroneCanOutputHandler::GetFrameHandles()
{
  return GetInstance().apFrameHandles_;
}

// ---------------------------------------------------------------------------
// Constructor: set default publish rates
// ---------------------------------------------------------------------------

CDroneCanOutputHandler::CDroneCanOutputHandler() :
    apFrameHandles_
    {
      SCanFrameHandle( CDroneCanOutputHandler::SendNodeStatus,          1U ),
      SCanFrameHandle( CDroneCanOutputHandler::SendRawImu,            100U ),
      SCanFrameHandle( CDroneCanOutputHandler::SendSolution,          100U ),
      SCanFrameHandle( CDroneCanOutputHandler::SendStaticPressure,     25U ),
      SCanFrameHandle( CDroneCanOutputHandler::SendStaticTemperature,  25U )
    }
{
}

// ---------------------------------------------------------------------------
// Init / IsInitialized
// ---------------------------------------------------------------------------

void CDroneCanOutputHandler::Init()
{
  if (false == bIsInitialized_)
  {
    // Future: load per-message rates from NVM (analogous to CCanOutputHandler).
    bIsInitialized_ = true;
  }
}

bool CDroneCanOutputHandler::IsInitialized()
{
  return bIsInitialized_;
}

// ---------------------------------------------------------------------------
// Utility
// ---------------------------------------------------------------------------

uint32_t CDroneCanOutputHandler::HerzToMs(uint32_t uFrequency)
{
  uint32_t uPeriodMs = 0U;

  if (0U != uFrequency)
  {
    uPeriodMs = 1000U / uFrequency;
  }

  return uPeriodMs;
}

bool CDroneCanOutputHandler::isSignalInRange(float fSignal, float fMin, float fMax)
{
  bool bInRange = std::isfinite(fSignal);

  if (true == bInRange)
  {
    bInRange = (fSignal >= fMin) && (fSignal <= fMax);
  }

  return bInRange;
}

// ---------------------------------------------------------------------------
// SendNodeStatus -- uavcan.protocol.NodeStatus (ID 341)
// ---------------------------------------------------------------------------

void CDroneCanOutputHandler::SendNodeStatus()
{
  static uint8_t suTransferId { 0U };

  uint8_t auPayload[DRONE_CAN_DSDL_PROTOCOL_NODE_STATUS_PAYLOAD_SIZE];
  const uint16_t kuPayloadLen = DroneCanDsdl_SerializeNodeStatus(
      auPayload,
      suUptimeSec_,
      DRONE_CAN_DSDL_NODE_HEALTH_OK,
      DRONE_CAN_DSDL_NODE_MODE_OPERATIONAL,
      /* sub_mode */ 0U,
      /* vendor_specific_status_code */ 0U);

  CDroneCanHandler::GetInstance().Broadcast(
      DRONE_CAN_DSDL_PROTOCOL_NODE_STATUS_SIGNATURE,
      DRONE_CAN_DSDL_PROTOCOL_NODE_STATUS_ID,
      &suTransferId,
      CANARD_TRANSFER_PRIORITY_MEDIUM,
      auPayload,
      kuPayloadLen);

  // This handler is dispatched at 1 Hz, so the call count is the uptime in seconds.
  ++suUptimeSec_;
}

uint32_t CDroneCanOutputHandler::GetUptimeSec()
{
  return suUptimeSec_;
}

// ---------------------------------------------------------------------------
// SendRawImu -- uavcan.equipment.ahrs.RawIMU (ID 1003)
// ---------------------------------------------------------------------------

void CDroneCanOutputHandler::SendRawImu()
{
  static uint8_t suTransferId { 0U };

  SOutputImuData oImuData;
  bool bNeedToContinue = true == CRte::GetInstance().oPortImuOutput_.Read(oImuData);

  if (true == bNeedToContinue)
  {
    bNeedToContinue = (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eSpecificForceHealth_) &&
                      (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eAngularRateHealth_);
  }

  if (true == bNeedToContinue)
  {
    // Validate signal ranges.
    const float kfGravity = 9.80665F;
    const float kfMaxAccelMs2 = skfMaxSpecificForceInG_ * kfGravity;

    bNeedToContinue = isSignalInRange(oImuData.fSpecificForceX_, -kfMaxAccelMs2,  kfMaxAccelMs2) &&
                      isSignalInRange(oImuData.fSpecificForceY_, -kfMaxAccelMs2,  kfMaxAccelMs2) &&
                      isSignalInRange(oImuData.fSpecificForceZ_, -kfMaxAccelMs2,  kfMaxAccelMs2) &&
                      isSignalInRange(oImuData.fAngularRateX_,  -skfMaxAngularRateInRadPerSecond_,
                                                                  skfMaxAngularRateInRadPerSecond_) &&
                      isSignalInRange(oImuData.fAngularRateY_,  -skfMaxAngularRateInRadPerSecond_,
                                                                  skfMaxAngularRateInRadPerSecond_) &&
                      isSignalInRange(oImuData.fAngularRateZ_,  -skfMaxAngularRateInRadPerSecond_,
                                                                  skfMaxAngularRateInRadPerSecond_);
  }

  if (true == bNeedToContinue)
  {
    uint8_t auPayload[DRONE_CAN_DSDL_AHRS_RAWIMU_PAYLOAD_SIZE];
    const uint16_t kuPayloadLen = DroneCanDsdl_SerializeRawImu(
        auPayload,
        oImuData.uTimestampUs_,
        /* integration_interval */ 0.0F,  // No integration implemented
        oImuData.fAngularRateX_,
        oImuData.fAngularRateY_,
        oImuData.fAngularRateZ_,
        oImuData.fSpecificForceX_,
        oImuData.fSpecificForceY_,
        oImuData.fSpecificForceZ_);

    CDroneCanHandler::GetInstance().Broadcast(
        DRONE_CAN_DSDL_AHRS_RAWIMU_SIGNATURE,
        DRONE_CAN_DSDL_AHRS_RAWIMU_ID,
        &suTransferId,
        CANARD_TRANSFER_PRIORITY_MEDIUM,
        auPayload,
        kuPayloadLen);
  }
}

// ---------------------------------------------------------------------------
// SendSolution -- uavcan.equipment.ahrs.Solution (ID 1000)
// ---------------------------------------------------------------------------

void CDroneCanOutputHandler::SendSolution()
{
  static uint8_t suTransferId { 0U };

  // Orientation source mirrors CCanOutputHandler::SendBodyPitchAngle/SendBodyRollAngle/SendMagneticHeading:
  // safety-monitored roll/pitch plus the safety-monitored magnetic heading, not a raw unmonitored filter instance.
  SSafeAttitudeData oAttitudeData;
  const bool kbAttitudeOk = CRte::GetInstance().oPortSafeVehicleAttitude_.Read(oAttitudeData);

  SSafeMagneticHeadingData oHeadingData;
  const bool kbHeadingOk = CRte::GetInstance().oPortSafeMagneticHeading_.Read(oHeadingData);

  SOutputImuData oImuData;
  const bool kbImuOk = CRte::GetInstance().oPortImuOutput_.Read(oImuData);

  bool bNeedToContinue = (true == kbAttitudeOk) && (true == kbHeadingOk) && (true == kbImuOk);

  if (true == bNeedToContinue)
  {
    bNeedToContinue = (CSerialProtocol::ESignalHealthInfo::eUnavailable != oAttitudeData.eHealth_) &&
                      (CSerialProtocol::ESignalHealthInfo::eUnavailable != oHeadingData.eHealth_);
  }

  if (true == bNeedToContinue)
  {
    bNeedToContinue = (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eSpecificForceHealth_) &&
                      (CSerialProtocol::ESignalHealthInfo::eUnavailable != oImuData.eAngularRateHealth_);
  }

  if (true == bNeedToContinue)
  {
    bNeedToContinue = isSignalInRange(oAttitudeData.fRoll_, -skfMaxRollInRad_, skfMaxRollInRad_) &&
                      isSignalInRange(oAttitudeData.fPitch_, -skfMaxPitchInRad_, skfMaxPitchInRad_) &&
                      isSignalInRange(oHeadingData.fMagneticHeading_, -skfMaxHeadingInRad_, skfMaxHeadingInRad_);
  }

  if (true == bNeedToContinue)
  {
    const float kfGravity = 9.80665F;
    const float kfMaxAccelMs2 = skfMaxSpecificForceInG_ * kfGravity;

    bNeedToContinue = isSignalInRange(oImuData.fSpecificForceX_, -kfMaxAccelMs2,  kfMaxAccelMs2) &&
                      isSignalInRange(oImuData.fSpecificForceY_, -kfMaxAccelMs2,  kfMaxAccelMs2) &&
                      isSignalInRange(oImuData.fSpecificForceZ_, -kfMaxAccelMs2,  kfMaxAccelMs2) &&
                      isSignalInRange(oImuData.fAngularRateX_,  -skfMaxAngularRateInRadPerSecond_, skfMaxAngularRateInRadPerSecond_) &&
                      isSignalInRange(oImuData.fAngularRateY_,  -skfMaxAngularRateInRadPerSecond_, skfMaxAngularRateInRadPerSecond_) &&
                      isSignalInRange(oImuData.fAngularRateZ_,  -skfMaxAngularRateInRadPerSecond_, skfMaxAngularRateInRadPerSecond_);
  }

  if (true == bNeedToContinue)
  {
    const CQuaternion koQuaternion = NNavigationUtilities::QuaternionFromEuler(oAttitudeData.fRoll_,
                                                                              oAttitudeData.fPitch_,
                                                                              oHeadingData.fMagneticHeading_);

    // Our quaternion convention is [W, X, Y, Z]; DroneCAN Solution field is [X, Y, Z, W].
    const float kfQx = koQuaternion.X();
    const float kfQy = koQuaternion.Y();
    const float kfQz = koQuaternion.Z();
    const float kfQw = koQuaternion.W();

    uint8_t auPayload[DRONE_CAN_DSDL_AHRS_SOLUTION_PAYLOAD_SIZE];
    const uint16_t kuPayloadLen = DroneCanDsdl_SerializeSolution(
        auPayload,
        oAttitudeData.uTimestampUs_,
        kfQx, kfQy, kfQz, kfQw,
        oImuData.fAngularRateX_,
        oImuData.fAngularRateY_,
        oImuData.fAngularRateZ_,
        oImuData.fSpecificForceX_,
        oImuData.fSpecificForceY_,
        oImuData.fSpecificForceZ_);

    CDroneCanHandler::GetInstance().Broadcast(
        DRONE_CAN_DSDL_AHRS_SOLUTION_SIGNATURE,
        DRONE_CAN_DSDL_AHRS_SOLUTION_ID,
        &suTransferId,
        CANARD_TRANSFER_PRIORITY_MEDIUM,
        auPayload,
        kuPayloadLen);
  }
}

// ---------------------------------------------------------------------------
// SendStaticPressure -- uavcan.equipment.air_data.StaticPressure (ID 1028)
// ---------------------------------------------------------------------------

void CDroneCanOutputHandler::SendStaticPressure()
{
  static uint8_t suTransferId { 0U };

  // Fall back across the 3 redundant barometers, mirroring CCanOutputHandler::SendStaticPressure.
  SBarometerMeasurement oBaroData;
  bool bReadOk = CRte::GetInstance().oPortSafePressureData1_.Read(oBaroData);

  if ((false == bReadOk) || (0U == oBaroData.uValid_))
  {
    bReadOk = CRte::GetInstance().oPortSafePressureData2_.Read(oBaroData);

    if ((false == bReadOk) || (0U == oBaroData.uValid_))
    {
      bReadOk = CRte::GetInstance().oPortSafePressureData3_.Read(oBaroData);
    }
  }

  bool bNeedToContinue = (true == bReadOk) && (0U != oBaroData.uValid_);

  if (true == bNeedToContinue)
  {
    bNeedToContinue = isSignalInRange(oBaroData.fPressure_, skfMinPressurePa_, skfMaxPressurePa_);
  }

  if (true == bNeedToContinue)
  {
    uint8_t auPayload[DRONE_CAN_DSDL_AIR_DATA_STATIC_PRESSURE_PAYLOAD_SIZE];
    const uint16_t kuPayloadLen = DroneCanDsdl_SerializeStaticPressure(
        auPayload,
        oBaroData.fPressure_,
        /* variance */ 0.0F);

    CDroneCanHandler::GetInstance().Broadcast(
        DRONE_CAN_DSDL_AIR_DATA_STATIC_PRESSURE_SIGNATURE,
        DRONE_CAN_DSDL_AIR_DATA_STATIC_PRESSURE_ID,
        &suTransferId,
        CANARD_TRANSFER_PRIORITY_MEDIUM,
        auPayload,
        kuPayloadLen);
  }
}

// ---------------------------------------------------------------------------
// SendStaticTemperature -- uavcan.equipment.air_data.StaticTemperature (ID 1029)
// ---------------------------------------------------------------------------

void CDroneCanOutputHandler::SendStaticTemperature()
{
  static uint8_t suTransferId { 0U };

  // Fall back across the 3 redundant barometers, mirroring CCanOutputHandler::SendStaticPressure.
  SBarometerMeasurement oBaroData;
  bool bReadOk = CRte::GetInstance().oPortSafePressureData1_.Read(oBaroData);

  if ((false == bReadOk) || (0U == oBaroData.uValid_))
  {
    bReadOk = CRte::GetInstance().oPortSafePressureData2_.Read(oBaroData);

    if ((false == bReadOk) || (0U == oBaroData.uValid_))
    {
      bReadOk = CRte::GetInstance().oPortSafePressureData3_.Read(oBaroData);
    }
  }

  bool bNeedToContinue = (true == bReadOk) && (0U != oBaroData.uValid_);
  float fTemperatureK = 0.0F;

  if (true == bNeedToContinue)
  {
    fTemperatureK = oBaroData.fTemperature_ + skfCelsiusToKelvin;
    bNeedToContinue = isSignalInRange(fTemperatureK, skfMinTemperatureK_, skfMaxTemperatureK_);
  }

  if (true == bNeedToContinue)
  {
    uint8_t auPayload[DRONE_CAN_DSDL_AIR_DATA_STATIC_TEMPERATURE_PAYLOAD_SIZE];
    const uint16_t kuPayloadLen = DroneCanDsdl_SerializeStaticTemperature(
        auPayload,
        fTemperatureK,
        /* variance */ 0.0F);

    CDroneCanHandler::GetInstance().Broadcast(
        DRONE_CAN_DSDL_AIR_DATA_STATIC_TEMPERATURE_SIGNATURE,
        DRONE_CAN_DSDL_AIR_DATA_STATIC_TEMPERATURE_ID,
        &suTransferId,
        CANARD_TRANSFER_PRIORITY_MEDIUM,
        auPayload,
        kuPayloadLen);
  }
}

#else
// ISO C/C++ forbid an empty translation unit.
typedef int CDroneCanOutputHandlerNoDroneCan_t;
#endif // USE_DRONECAN
