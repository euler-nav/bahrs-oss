/**
* @file CommonMonitorTypes.h
* @brief Type declaration common to all sensor monitors.
* @author Fedor Baklanov
* @date 11 December 2023
* @copyright Copyright 2023 AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef COMMON_MONITOR_TYPES_H
#define COMMON_MONITOR_TYPES_H

#include "MeasurementStructs.h"
#include "TLabeledArray.h"

namespace NMonitorTypes
{
  /**
   * @brief Fault detection result
  */
  enum class EDetectionResult : int
  {
    eInvalid = 0, ///< Invalid (default) value
    eUnavailable, ///< Fault detection function is unavailable
    eGood,///< No failure detected
    eFailure ///< Failure detected
  };

  /**
   * @brief Fault isolation result
  */
  enum class EIsolationResult : int
  {
    eInvalid = 0, ///< Invalid (default) value
    eUnavailable, ///< Fault isolation function is unavailable
    eGood, ///< Fault isolation succeeded
    eFailed ///< Fault isolation failed
  };

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

  /**
   * @brief Array labeled by redundant sensor indices.
  */
  DECLARE_LABELED_ARRAY_TEMPLATE(SRedundantSensorLabels, eSensor1, eSensor2, eSensor3)

  /**
   * @brief Template class for storing triple redundant sensor data.
   * @tparam SensorDataType Data type to store single data of a single sensor.
   * @tparam SensorId1 Global ID of the first redundant sensor.
   * @tparam SensorId2 Global ID of the second redundant sensor.
   * @tparam SensorId3 Global ID of the third redundant sensor.
  */
  template<typename SensorDataType, NFusionLibCommon::ESensorId SensorId1, NFusionLibCommon::ESensorId SensorId2, NFusionLibCommon::ESensorId SensorId3>
  class TRedundantSensorData
  {
  public:
    using ERedundantSensorLabels = SRedundantSensorLabels::EnumType;
    using DataType = SensorDataType;

    static_assert(SensorId1 != SensorId2);
    static_assert(SensorId1 != SensorId3);

    /**
     * @brief Set sensor data from a single sensor measurement.
     * @param korImuMeasurement Reference to the data.
     * @return True -- success, false -- failure.
    */
    bool Set(const SensorDataType& korMeasurement);

    /**
     * @brief Get internally stored data of the specific sensor by its logical index.
     * @param eIndex Logical index (label) of the sensor.
     * @return Sensor data
    */
    SensorDataType Get(ERedundantSensorLabels eIndex) const;

    /**
     * @brief Get internally stored data of the specific sensor.
     * @param eSensor ID of the sensor
     * @return Sensor data
    */
    SensorDataType Get(NFusionLibCommon::ESensorId eSensor) const;

    static constexpr int32_t skiMaxRedundantMeasurementCount_{ static_cast<int32_t>(ERedundantSensorLabels::eCount) }; ///< Maximum number of supported redundant measurement

    /**
     * @brief Get logical index of a sensor by its sensor ID.
     * @param eSensor ID of the sensor
     * @return Label (logical index) of the redundant sensor, eCount if the sensor ID is not supported.
    */
    static ERedundantSensorLabels IndexOf(NFusionLibCommon::ESensorId eSensor);

    /**
     * @brief Check if the monitor supports the sensor with the given ID.
     * @param eSensor Sensor ID.
     * @return True -- supported, false otherwise.
    */
    static bool IsSensorSupported(NFusionLibCommon::ESensorId eSensor);

  protected:

  private:
    SRedundantSensorLabels::TLabeledArray<SensorDataType> oData_; ///< Internal sensor data storage

#ifdef GOOGLETEST_INCLUDE_GTEST_GTEST_H_
    FRIEND_TEST(TRedundantSensorDataTest, SetGet);
#endif
  };

  /**
   * @brief Base class for multidimensional output of a parity monitor.
  */
  struct SMultidimensionalSignalBase
  {
    using ESensorId = NFusionLibCommon::ESensorId;
    using EDetectionResult = NMonitorTypes::EDetectionResult;
    using EIsolationResult = NMonitorTypes::EIsolationResult;

    /**
     * @brief Data and attributes associted with a single scalar signal.
    */
    struct SScalarSignal
    {
      float fSignal_{ 0.0F }; ///< IMU signal
      uint64_t uTimestampUs_{ 0U }; ///< Timestamps of the scalar IMU signal, [us]
      ESensorId eSensorId_{ ESensorId::eUnknown }; ///< ID of the sensor
      EDetectionResult eDetectionResults_{ EDetectionResult::eInvalid }; ///< Result of the fault detection
      EIsolationResult eIsolationResults_{ EIsolationResult::eInvalid }; ///< Result of the fault isolation
      ESensorId eIsolatedSensor_{ ESensorId::eUnknown }; ///< ID of the isolated sensor
      bool bValid_{ false }; ///< Scalar signal validity, true -- valid, false -- invalid
    };
  };

  /**
   * @brief A template for multidimensional output of a parity monitor.
   * @tparam LabeledArrayStruct Labeled array struct created by macro DECLARE_LABELED_ARRAY_TEMPLATE. Defines signal demension and labels.
  */
  template<class LabeledArrayStruct>
  class TMultidimensionalSignal : public SMultidimensionalSignalBase
  {
  public:
    using SLabeledArrayStruct = LabeledArrayStruct;
    using EScalarSignals = typename LabeledArrayStruct::EnumType;
    static constexpr uint32_t skuSignalCount_{ static_cast<uint32_t>(EScalarSignals::eCount) };

    /**
     * @brief Get reference to signal by signal label.
    */
    const SScalarSignal& GetSignal(EScalarSignals eSignal) const;

    /**
     * @brief Get reference to signal by signal label.
    */
    SScalarSignal& GetSignal(EScalarSignals eSignal);

  private:
    typename LabeledArrayStruct::template TLabeledArray<SScalarSignal> oSignals_; ///< Storage for scalar signals
  };

  template<typename SensorDataType, NFusionLibCommon::ESensorId SensorId1, NFusionLibCommon::ESensorId SensorId2, NFusionLibCommon::ESensorId SensorId3>
  typename TRedundantSensorData<SensorDataType, SensorId1, SensorId2, SensorId3>::ERedundantSensorLabels TRedundantSensorData<SensorDataType, SensorId1, SensorId2, SensorId3>::IndexOf(NFusionLibCommon::ESensorId eSensor)
  {
    ERedundantSensorLabels eLabel;

    using NFusionLibCommon::ESensorId;

    switch (eSensor)
    {
    case SensorId1:
      eLabel = ERedundantSensorLabels::eSensor1;
      break;
    case SensorId2:
      eLabel = ERedundantSensorLabels::eSensor2;
      break;
    case SensorId3:
      eLabel = ERedundantSensorLabels::eSensor3;
      break;
    default:
      eLabel = ERedundantSensorLabels::eCount;
      break;
    }

    return eLabel;
  }

  template<typename SensorDataType, NFusionLibCommon::ESensorId SensorId1, NFusionLibCommon::ESensorId SensorId2, NFusionLibCommon::ESensorId SensorId3>
  bool TRedundantSensorData<SensorDataType, SensorId1, SensorId2, SensorId3>::IsSensorSupported(NFusionLibCommon::ESensorId eSensor)
  {
    return (IndexOf(eSensor) == ERedundantSensorLabels::eCount) ? false : true;
  }

  template<typename SensorDataType, NFusionLibCommon::ESensorId SensorId1, NFusionLibCommon::ESensorId SensorId2, NFusionLibCommon::ESensorId SensorId3>
  bool TRedundantSensorData<SensorDataType, SensorId1, SensorId2, SensorId3>::Set(const SensorDataType& korMeasurement)
  {
    ERedundantSensorLabels eLabel = IndexOf(korMeasurement.eSensorId_);
    bool bStatus = false;

    if (ERedundantSensorLabels::eCount != eLabel)
    {
      oData_[eLabel] = korMeasurement;
      bStatus = true;
    }

    return bStatus;
  }

  template<typename SensorDataType, NFusionLibCommon::ESensorId SensorId1, NFusionLibCommon::ESensorId SensorId2, NFusionLibCommon::ESensorId SensorId3>
  SensorDataType TRedundantSensorData<SensorDataType, SensorId1, SensorId2, SensorId3>::Get(ERedundantSensorLabels eIndex) const
  {
    return oData_[eIndex];
  }

  template<typename SensorDataType, NFusionLibCommon::ESensorId SensorId1, NFusionLibCommon::ESensorId SensorId2, NFusionLibCommon::ESensorId SensorId3>
  SensorDataType TRedundantSensorData<SensorDataType, SensorId1, SensorId2, SensorId3>::Get(NFusionLibCommon::ESensorId eSensor) const
  {
    SensorDataType oMeasurement;
    oMeasurement.eSensorId_ = NFusionLibCommon::ESensorId::eUnknown;

    if (IsSensorSupported(eSensor))
    {
      oMeasurement = oData_[IndexOf(eSensor)];
    }

    return oMeasurement;
  }

  template<class LabeledArrayStruct>
  const typename TMultidimensionalSignal<LabeledArrayStruct>::SScalarSignal& TMultidimensionalSignal<LabeledArrayStruct>::GetSignal(EScalarSignals eSignal) const
  {
    return oSignals_[eSignal];
  }

  template<class LabeledArrayStruct>
  typename TMultidimensionalSignal<LabeledArrayStruct>::SScalarSignal& TMultidimensionalSignal<LabeledArrayStruct>::GetSignal(EScalarSignals eSignal)
  {
    return oSignals_[eSignal];
  }
}

#endif /* COMMON_MONITOR_TYPES_H */
