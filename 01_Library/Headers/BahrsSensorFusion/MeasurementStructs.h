
/// @file MeasurementStructs.h
/// @brief Declaration of measurement structs.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
#ifndef S_MEASUREMENT_STRUCTS_H
#define S_MEASUREMENT_STRUCTS_H

#include <stdint.h>
#include "Eigen/Dense"

namespace NFusionLibCommon
{
  /// @brief An enumeration of sensors.
  enum class ESensorId : int
  {
    eUnknown = 0, ///< Unknown/invalid
    eScha63T = 1, ///< Murata SCHA63T inertial sensor
    eBmp384 = 2, ///< Bosch BMP384 barometer
    eIcm20789Imu1 = 3, ///< The first TDK ICM20789 IMU
    eIcm20789Imu2 = 4, ///< The second TDK ICM20789 IMU
    eIcm20789Baro1 = 5, ///< The first TDK ICM20789 barometer
    eIcm20789Baro2 = 6, ///< The second TDK ICM20789 barometer
    eBmm150X1 = 7, ///< The first Bosch BMM150 magnetometer
    eBmm150X2 = 8, ///< The second Bosch BMM150 magnetometer
    eMmc5983 = 9, ///< MEMSIC MMC5983 magnetometer
    eBahrsFilter1 = 10, ///< Virtual sensor: output of BAHRS filter
    eBahrsFilter2 = 11, ///< Virtual sensor: output of BAHRS filter
    eBahrsFilter3 = 12, ///< Virtual sensor: output of BAHRS filter
    eBmi270 = 13, ///< Bosch BMI270 inertial sensor
    eAsm330 = 14, ///< STMicroelectronics ASM330 inertial sensor
    eIcp20100 = 15, ///< TDK ICP-20100 pressure sensor
    eLps22 = 16, ///< STMicroelectronics LPS22 barometer
    eBmm350 = 17, ///< Bosch BMM350 magnetometer
    eLis3Mdl = 18 ///< STMicroelectronics LIS3MDL magnetometer
  };

  /// @brief Check if the sensor is an inertial sensor.
  /// @param eSensorId Sensor ID to check.
  /// @return True if the sensor is an inertial sensor, false otherwise.
  inline bool isInertialSensor(ESensorId eSensorId)
  {
    return (ESensorId::eScha63T == eSensorId) ||
           (ESensorId::eIcm20789Imu1 == eSensorId) ||
           (ESensorId::eIcm20789Imu2 == eSensorId) ||
           (ESensorId::eBmi270 == eSensorId) ||
           (ESensorId::eAsm330 == eSensorId);
  }

  /// @brief Check if the sensor is a pressure sensor.
  /// @param eSensorId Sensor ID to check.
  /// @return True if the sensor is a pressure sensor, false otherwise.
  inline bool isPressureSensor(ESensorId eSensorId)
  {
    return (ESensorId::eBmp384 == eSensorId) ||
           (ESensorId::eIcm20789Baro1 == eSensorId) ||
           (ESensorId::eIcm20789Baro2 == eSensorId) ||
           (ESensorId::eIcp20100 == eSensorId) ||
           (ESensorId::eLps22 == eSensorId);
  }

  /// @brief Check if the sensor is a magnetic sensor.
  /// @param eSensorId Sensor ID to check.
  /// @return True if the sensor is a magnetic sensor, false otherwise.
  inline bool isMagneticSensor(ESensorId eSensorId)
  {
    return (ESensorId::eBmm150X1 == eSensorId) ||
           (ESensorId::eBmm150X2 == eSensorId) ||
           (ESensorId::eMmc5983 == eSensorId) ||
           (ESensorId::eBmm350 == eSensorId) ||
           (ESensorId::eLis3Mdl == eSensorId);
  }

  struct SMeasurementBase
  {
    SMeasurementBase() = default;

    SMeasurementBase(uint64_t uTimestampUs, ESensorId eSensorId, bool bValid) :
      uTimestampUs_(uTimestampUs),
      eSensorId_(eSensorId),
      bValid_(bValid)
    {
      // Do nothing
    }

    uint64_t uTimestampUs_{ 0U };
    ESensorId eSensorId_{ ESensorId::eUnknown };
    bool bValid_{ false };
  };

  struct SImuMeasurement : public SMeasurementBase
  {
    SImuMeasurement() = default;

    SImuMeasurement(float fSpecificForceX, float fSpecificForceY, float fSpecificForceZ,
                    float fAngularRateX, float fAngularRateY, float fAngularRateZ,
                    uint64_t uTimestampUs, ESensorId eSensorId, bool bValid) :
      SMeasurementBase(uTimestampUs, eSensorId, bValid),
      fSpecificForceX_(fSpecificForceX),
      fSpecificForceY_(fSpecificForceY),
      fSpecificForceZ_(fSpecificForceZ),
      fAngularRateX_(fAngularRateX),
      fAngularRateY_(fAngularRateY),
      fAngularRateZ_(fAngularRateZ)
    {
      // Do nothing
    }

    float fSpecificForceX_{ 0.0F };
    float fSpecificForceY_{ 0.0F };
    float fSpecificForceZ_{ 0.0F };
    float fAngularRateX_{ 0.0F };
    float fAngularRateY_{ 0.0F };
    float fAngularRateZ_{ 0.0F };
  };

  
  /// @brief Input magnetometer measurements.
  struct SMagnetometerData : public SMeasurementBase
  {
    Eigen::Vector3f oInductionVectorInGauss_{ 0.0F, 0.0F, 0.0F }; ///< Measured magnetic induction, [G]
  };

  struct SBarometerData : public SMeasurementBase
  {
    SBarometerData() = default;

    SBarometerData(float fPressure,
      uint64_t uTimestampUs, ESensorId eSensorId, bool bValid) :
      SMeasurementBase(uTimestampUs, eSensorId, bValid),
      fPressure_(fPressure)
    {
      // Do nothing
    }

    float fPressure_{ 0.0F }; ///< Pressure in Pascals 
  };

  
  /// @brief A virtual measurement created from attitude output of a BAHRS filter.
  /// The measurement is used as input type for the dedicated parity monitor.
  struct SAttitudeOutputData : public SMeasurementBase
  {
    SAttitudeOutputData() = default;

    SAttitudeOutputData(float fRoll, float fPitch,
                        uint64_t uTimestampUs, ESensorId eSensorId, bool bValid) :
      SMeasurementBase(uTimestampUs, eSensorId, bValid),
      fRoll_(fRoll),
      fPitch_(fPitch)
    {
      // Do nothing
    }

    float fRoll_{ 0.0F };
    float fPitch_{ 0.0F };
  };

  
  /// @brief A virtual measurement created from height and velocity down output of a BAHRS filter.
  /// The measurement is used as input type for the dedicated parity monitor.
  struct SVerticalChannelData : public SMeasurementBase
  {
    SVerticalChannelData() = default;

    SVerticalChannelData(float fHeight, float fVelocityDown,
                         uint64_t uTimestampUs, ESensorId eSensorId, bool bValid) :
      SMeasurementBase(uTimestampUs, eSensorId, bValid),
      fHeight_(fHeight),
      fVelocityDown_(fVelocityDown)
    {
      // Do nothing
    }

    float fHeight_{ 0.0F };
    float fVelocityDown_{ 0.0F };
  };
}

#endif // S_MEASUREMENT_STRUCTS_H
