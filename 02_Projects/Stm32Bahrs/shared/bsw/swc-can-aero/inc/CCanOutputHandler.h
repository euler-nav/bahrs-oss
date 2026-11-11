/// @file CCanOutputHandler.h
/// @brief Declaration of the CCanOutputHandler class (CAN output handler software component).
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_CAN_OUTPUT_HANDLER_H
#define C_CAN_OUTPUT_HANDLER_H

#include "canaerospace/canaerospace.h"
#include "canaerospace/param_id/nod_default.h"
#include "canaerospace/services/std_identification.h"
#include "can_driver.h"
#include "GetMicroseconds.h"
#include "cmsis_os.h"
#include <array>
#include "General/CSoftwareComponentBase.h"

/// @brief CAN Output Handler software component.
class CCanOutputHandler : public CSoftwareComponent<CCanOutputHandler, 1U>
{
  friend class CSoftwareComponent<CCanOutputHandler, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CCanOutputHandler)
  DECLARE_MANDATORY_APIS(CCanOutputHandler)

public:

  /// @brief RTE Supported CANAerospace output messages.
  enum class ECanIds : uint8_t
  {
    eBodyLongitudinalAcceleration = 0,  ///< CAN Aerospace body longitudinal acceleration
    eBodyLateralAcceleration,           ///< CAN Aerospace body lateral acceleration
    eBodyNormalAcceleration,            ///< CAN Aerospace body normal acceleration
    eBodyPitchRate,                     ///< CAN Aerospace body pitch rate
    eBodyRollRate,                      ///< CAN Aerospace body roll rate
    eBodyYawRate,                       ///< CAN Aerospace body yaw rate
    eBodyPitchAngle,                    ///< CAN Aerospace body pitch angle
    eBodyRollAngle,                     ///< CAN Aerospace body roll angle
    eAltitudeRate,                      ///< CAN Aerospace altitude rate
    eMagneticHeading,                   ///< CAN Aerospace magnetic heading
    eStandardAltitude,                  ///< CAN Aerospace standard altitude
    eStaticPressure,                    ///< CAN Aerospace static pressure
    eLastMessage,                       ///< Definition for the last message. Don't add below this!
  };

  typedef void(*const SendFrameFunction_t)();

  /// @brief Compose and send the body longitudinal acceleration.
  static void SendBodyLongitudinalAcceleration();

  /// @brief Compose and send the body lateral acceleration.
  static void SendBodyLateralAcceleration();

  /// @brief Compose and send the body normal acceleration.
  static void SendBodyNormalAcceleration();

  /// @brief Compose and send the body pitch rate.
  static void SendBodyPitchRate();

  /// @brief Compose and send the body roll rate.
  static void SendBodyRollRate();

  /// @brief Compose and send the body yaw rate.
  static void SendBodyYawRate();

  /// @brief Compose and send the body pitch angle.
  static void SendBodyPitchAngle();

  /// @brief Compose and send the body roll angle.
  static void SendBodyRollAngle();

  /// @brief Compose and send the altitude rate.
  static void SendAltitudeRate();

  /// @brief Compose and send the magnetic heading.
  static void SendMagneticHeading();

  /// @brief Compose and send the standard altitude.
  static void SendStandardAltitude();

  /// @brief Compose and send the static pressure.
  static void SendStaticPressure();

  /// @brief Provides the period expressed in [ms] for a given frequency
  static uint32_t HerzToMs(uint32_t uFrequency);

  /// A frame handle structure that combines a pointer to a sender function
  /// and desired frame frequency.
  struct SCanFrameHandle
  {
    SCanFrameHandle(SendFrameFunction_t pSender, uint32_t uFrameFrequency_) :
      uFrameFrequency_(uFrameFrequency_),
      pSend_(pSender)
    {};

    SCanFrameHandle() = delete;

    uint32_t uFrameFrequency_;
    SendFrameFunction_t pSend_;
  };

  static constexpr int skiSupportedFrameCount_ {static_cast<int>(ECanIds::eLastMessage)}; ///< Number of supported output frames
  using CFrameHanleArray = std::array<SCanFrameHandle, skiSupportedFrameCount_>;

  /// Get a reference to the array of CAN output frame handles.
  /// @return
  static const CFrameHanleArray& GetFrameHandles();

protected:

private:
  CCanOutputHandler();
  ~CCanOutputHandler() = default;

  static constexpr float skfMaxSpecificForceInG_{5.0F};
  static constexpr float skfMaxAngularRateInDegreesPerSecond_{300.0F};
  static constexpr float skfMaxRollInDegrees_{180.0F};
  static constexpr float skfMaxPitchInDegrees_{90.0F};
  static constexpr float skfMaxHeadingInDegrees_{180.0F};
  static constexpr float skfMaxVelocityDown_{300.0F};
  static constexpr float skfMaxAltitude_{10000.0F};
  static constexpr float skfMinAltitude_{-1000.0F};
  static constexpr float skfMinPressureInHPa_{300.0F};
  static constexpr float skfMaxPressureInHPa_{1250.0F};

  /// @brief Check if the signal is in the specified range.
  ///
  /// The function returns false if any of the input arguments is not finite,
  /// i. e. is infinity or NaN.
  ///
  /// The function returns true if the signal is in the specified range,
  /// including MIN and MAX values.
  ///
  /// @param [in] fSignal Signal to check
  /// @param [in] fSignalMin Minimum value of the signal
  /// @param [in] fSignalMax Maximum value of the signal
  /// @return See function description.
  static bool isSignalInRange(float fSignal, float fSignalMin, float fSignalMax);

  CFrameHanleArray apFrameHandles_; ///< An array of CAN frame handles. Allows easy looping through all the frames.

  bool bIsInitialized_ { false };

};

#endif // C_CAN_OUTPUT_HANDLER_H
