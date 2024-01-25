/**
* @file ImuMonitorApi.h
* @brief Declaration of the IMU monitor API.
* @author Fedor Baklanov
* @date 6 December 2023
* @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef IMU_MONITOR_API_H
#define IMU_MONITOR_API_H

#include "MeasurementStructs.h"
#include "CommonMonitorTypes.h"
#include "TLabeledArray.h"

namespace NImuMonitorApi
{
  /**
   * @brief Monitor state enum.
  */
  enum class EMonitorState : int32_t
  {
    eInvalid = 0, ///< Invalid (default) state
    eInit, ///< Monitor is initializing
    eRunning, ///< Monitor is running
    eInitFailed ///< Monitor failed to initialize
  };

  DECLARE_LABELED_ARRAY_TEMPLATE(SArrayLabeledByImuSignals, eSpecificForceX, eSpecificForceY, eSpecificForceZ, eAngularRateX, eAngularRateY, eAngularRateZ)

  /**
   * @brief IMU data type used by the monitor's output interface.
  */
  class CImuData
  {
  public:
    using ESensorId = NFusionLibCommon::ESensorId;
    using EDetectionResult = NMonitorTypes::EDetectionResult;
    using EIsolationResult = NMonitorTypes::EIsolationResult;

    using EImuSignals = SArrayLabeledByImuSignals::EnumType;

    struct SScalarSignal
    {
      float fSignal_{ 0.0F }; ///< IMU signal
      uint64_t uTimestampUs_{ 0U }; ///< Timestamps of the scalar IMU signal, [us]
      ESensorId eSensorId_{ ESensorId::eUnknown }; ///< ID of the sensor
      EDetectionResult eDetectionResults_{ EDetectionResult::eInvalid }; ///< Result of the fault detection
      EIsolationResult eIsolationResults_{ EIsolationResult::eInvalid }; ///< Result of the fault isolation
      bool bValid_{ false }; ///< Scalar signal validity, true -- valid, false -- invalid
    };

    static constexpr uint32_t skuSignalCount_{static_cast<uint32_t>(EImuSignals::eCount)};

    /**
     * @brief Get IMU signal reference by signal label
    */
    const SScalarSignal& GetSignal(EImuSignals eSignal) const;

    /**
     * @brief Get IMU signal reference by signal label
    */
    SScalarSignal& GetSignal(EImuSignals eSignal);

  private:
    SArrayLabeledByImuSignals::TLabeledArray<SScalarSignal> oSignals_; ///< Storage for IMU signals
  };

  /**
   * @brief Redundant IMU measurements.
   * The type serves as an input interface of the IMU monitor.
  */
  class CRedundantImuData
  {
  public:
    /**
     * @brief Array labeled by redundant IMU indices.
    */
    DECLARE_LABELED_ARRAY_TEMPLATE(SRedundantImuLabels, eImu1, eImu2, eImu3)

    using ERedundantImuLabels = SRedundantImuLabels::EnumType;

    /**
     * @brief Set IMU data from a single 6D inertial measurement unit.
     * @param korImuMeasurement Reference to the data.
     * @return True -- success, false -- failure.
    */
    bool Set(const NFusionLibCommon::SImuMeasurement& korImuMeasurement);
 
    /**
     * @brief Get internally stored data of the specific sensor by its logical index.
     * @param eIndex Logical index (label) of the sensor.
     * @return IMU data
    */
    NFusionLibCommon::SImuMeasurement Get(ERedundantImuLabels eIndex) const;

    /**
     * @brief Get internally stored data of the specific sensor.
     * @param eSensor ID of the sensor
     * @return IMU data
    */
    NFusionLibCommon::SImuMeasurement Get(NFusionLibCommon::ESensorId eSensor) const;

    static constexpr int32_t skiMaxRedundantMeasurementCount_{ static_cast<int32_t>(ERedundantImuLabels::eCount) }; ///< Maximum number of supported redundant measurement

    /**
     * @brief Get logical index of an IMU sensor by its sensor ID.
     * @param eSensor ID of the sensor
     * @return Label (logical index) of the redundant sensor, eCount if the sensor ID is not supported.
    */
    static ERedundantImuLabels IndexOf(NFusionLibCommon::ESensorId eSensor);

    /**
     * @brief Check if the monitor supports the sensor with the given ID.
     * @param eSensor Sensor ID.
     * @return True -- supported, false otherwise.
    */
    static bool IsSensorSupported(NFusionLibCommon::ESensorId eSensor);

  protected:

  private:
    SRedundantImuLabels::TLabeledArray<NFusionLibCommon::SImuMeasurement> oData_; ///< Internal IMU data storage
  };

  /**
   * @brief Get monitor state.
   * @return Monitor state
  */
  NImuMonitorApi::EMonitorState ImuMonitorGetState();

  /**
   * @brief Run redundancy-base IMU signal check.
   * @param korMeasurements Input redundant measurements.
   * @return Output IMU measurement.
  */
  NImuMonitorApi::CImuData ImuMonitorRun(const CRedundantImuData& korMeasurements);
}

#endif /* IMU_MONITOR_API_H */
