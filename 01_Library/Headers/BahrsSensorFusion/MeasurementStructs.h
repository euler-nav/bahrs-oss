/**
* @file MeasurementStructs.h
* @brief Declaration of measurement structs.
* @author Fedor Baklanov
* @date 6 December 2023
* @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef S_MEASUREMENT_STRUCTS_H
#define S_MEASUREMENT_STRUCTS_H

#include <stdint.h>
#include "Eigen/Dense"

namespace NFusionLibCommon
{
  /**
   * @brief An enumeration of sensors.
  */
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
    eMmc5983 = 9 ///< MEMSIC MMC5983 magnetometer
  };

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

  /**
   * @brief Input magnetometer measurements.
  */
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
}

#endif /* S_MEASUREMENT_STRUCTS_H */
