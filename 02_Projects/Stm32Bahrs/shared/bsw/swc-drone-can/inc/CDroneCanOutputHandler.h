/// @file CDroneCanOutputHandler.h
/// @brief Declaration of the CDroneCanOutputHandler class.
///
/// CDroneCanOutputHandler publishes the device's sensor data as DroneCAN messages.
/// It reads from RTE ports, serializes payloads using the helpers in
/// DroneCanDsdlMessages.h, and enqueues the transfers via CDroneCanHandler::Broadcast().
///
/// The caller (CAN sender task) must invoke CDroneCanHandler::TransmitFrames()
/// after iterating through the frame handles so that queued frames are sent to
/// the bus.
///
/// Supported DroneCAN output messages:
///  - uavcan.protocol.NodeStatus           (ID 341)  - 1 Hz (required for node discovery)
///  - uavcan.equipment.ahrs.RawIMU         (ID 1003) - 100 Hz default
///  - uavcan.equipment.ahrs.Solution       (ID 1000) - 100 Hz default
///  - uavcan.equipment.air_data.StaticPressure    (ID 1028) - 25 Hz default
///  - uavcan.equipment.air_data.StaticTemperature (ID 1029) - 25 Hz default
///
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_DRONE_CAN_OUTPUT_HANDLER_H
#define C_DRONE_CAN_OUTPUT_HANDLER_H

#include "CDroneCanHandler.h"
#include "DroneCanDsdlMessages.h"
#include "cmsis_os.h"
#include <array>
#include "General/CSoftwareComponentBase.h"

/// @brief DroneCAN Output Handler software component.
class CDroneCanOutputHandler : public CSoftwareComponent<CDroneCanOutputHandler, 1U>
{
  friend class CSoftwareComponent<CDroneCanOutputHandler, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CDroneCanOutputHandler)
  DECLARE_MANDATORY_APIS(CDroneCanOutputHandler)

public:

  /// @brief Enumeration of supported DroneCAN output message types.
  enum class EMessageIds : uint8_t
  {
    eNodeStatus = 0,       ///< uavcan.protocol.NodeStatus
    eRawImu,               ///< uavcan.equipment.ahrs.RawIMU
    eSolution,             ///< uavcan.equipment.ahrs.Solution
    eStaticPressure,       ///< uavcan.equipment.air_data.StaticPressure
    eStaticTemperature,    ///< uavcan.equipment.air_data.StaticTemperature
    eLastMessage           ///< Sentinel - do not add below this.
  };

  typedef void(*const SendFrameFunction_t)();

  /// @brief Send uavcan.protocol.NodeStatus.
  ///
  /// Broadcast at 1 Hz.  DroneCAN network tools discover a node exclusively via
  /// this message, so it must be published for the device to appear in the GUI.
  static void SendNodeStatus();

  /// @brief Get the current node uptime in seconds.
  ///
  /// Incremented once per SendNodeStatus() call (1 Hz). Used by the GetNodeInfo
  /// service response so its NodeStatus sub-field is consistent with the periodic
  /// NodeStatus broadcast.
  ///
  /// @return Seconds since the first NodeStatus broadcast.
  static uint32_t GetUptimeSec();

  /// @brief Send uavcan.equipment.ahrs.RawIMU.
  static void SendRawImu();

  /// @brief Send uavcan.equipment.ahrs.Solution.
  static void SendSolution();

  /// @brief Send uavcan.equipment.air_data.StaticPressure.
  static void SendStaticPressure();

  /// @brief Send uavcan.equipment.air_data.StaticTemperature.
  static void SendStaticTemperature();

  /// @brief Convert a frequency in Hz to a period in milliseconds.
  /// @param [in] uFrequency  Frequency [Hz].
  /// @return                 Period [ms]; returns 0 if uFrequency is 0.
  static uint32_t HerzToMs(uint32_t uFrequency);

  /// @brief A frame handle combining a sender function pointer with a rate.
  struct SCanFrameHandle
  {
    SCanFrameHandle(SendFrameFunction_t pSender, uint32_t uFrameFrequency) :
      uFrameFrequency_(uFrameFrequency),
      pSend_(pSender)
    {}

    SCanFrameHandle() = delete;

    uint32_t uFrameFrequency_;
    SendFrameFunction_t pSend_;
  };

  static constexpr int skiSupportedFrameCount_ { static_cast<int>(EMessageIds::eLastMessage) };
  using CFrameHandleArray = std::array<SCanFrameHandle, skiSupportedFrameCount_>;

  /// @brief Get a const reference to the frame handle array.
  /// @return Reference to the array of frame handles.
  static const CFrameHandleArray& GetFrameHandles();

private:
  CDroneCanOutputHandler();
  ~CDroneCanOutputHandler() = default;

  static constexpr float skfMaxSpecificForceInG_            { 5.0F };
  static constexpr float skfMaxAngularRateInRadPerSecond_   { 5.24F }; ///< ~300 deg/s
  static constexpr float skfMaxRollInRad_                   { 3.14159265F }; ///< 180 deg
  static constexpr float skfMaxPitchInRad_                  { 1.57079633F }; ///< 90 deg
  static constexpr float skfMaxHeadingInRad_                { 3.14159265F }; ///< 180 deg
  static constexpr float skfMinPressurePa_                  { 30000.0F };
  static constexpr float skfMaxPressurePa_                  { 125000.0F };
  static constexpr float skfMinTemperatureK_                { 173.15F }; ///< -100 deg C
  static constexpr float skfMaxTemperatureK_                { 358.15F }; ///< +85 deg C

  /// @brief Check that a signal value is finite and within [fMin, fMax].
  /// @param [in] fSignal   Value to check.
  /// @param [in] fMin      Lower bound (inclusive).
  /// @param [in] fMax      Upper bound (inclusive).
  /// @return               true if the value is in range and finite.
  static bool isSignalInRange(float fSignal, float fMin, float fMax);

  CFrameHandleArray apFrameHandles_; ///< Frame handle array - initialized in constructor.
  bool bIsInitialized_ { false };

  static uint32_t suUptimeSec_; ///< Seconds since first NodeStatus broadcast (1 Hz counter).
};

#endif // C_DRONE_CAN_OUTPUT_HANDLER_H
