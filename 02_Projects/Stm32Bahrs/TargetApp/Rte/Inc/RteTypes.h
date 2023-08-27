/**
 * @file RteTypes.h
 * @brief Declaration of software component port types.
 * @author Fedor Baklanov
 * @date 02 June 2022
 */

#ifndef RTE_TYPES_H
#define RTE_TYPES_H

#ifndef __cplusplus
  #error This header must not be included in .c files.
#endif /* __cplusplus */

#pragma pack(push, 1)

// ----IMPORTANT----
//
// Port data can be sent as a part of debug output. Therefore it is
// required to ensure that the port data types below do not use
// any implementation defined types. The type bool is implementation
// defined and its size will not be the same on different platforms.
// This is why we will use uint8_t for logical data in software
// component ports.
//
// ----IMPORTANT----

/**
 *This is a placeholder for the Murata data proxy port type.
 */
struct SImuDataScha63T
{
  float fSpecificForceX_ { 0.0F };
  float fSpecificForceY_ { 0.0F };
  float fSpecificForceZ_ { 0.0F };
  float fAngularRateX_ { 0.0F };
  float fAngularRateY_ { 0.0F };
  float fAngularRateZ_ { 0.0F };
  uint64_t uTimestampUs_ { 0U };
  uint8_t uValid_ { 0U };

};

/**
 * Input from pressure sensor.
 */
struct SBarometerMeasurement
{
  float fPressure_ { 0.0F }; ///< Pressure in Pascals
  float fTemperature_ { 0.0F }; ///< Temperature in degrees Celsius
  uint64_t uTimestampUs_ { 0U }; ///< Timestamp in microseconds
  uint8_t uValid_ { 0U }; ///< Validity flag
};

/**
 * Input from magnetic sensor.
 */
struct SMagneticMeasurement
{
  float fVectorX_ { 0.0F }; ///< Magnetic field vector in mG axis X
  float fVectorY_ { 0.0F }; ///< Magnetic field vector in mG axis Y
  float fVectorZ_ { 0.0F }; ///< Magnetic field vector in mG axis Z
  uint64_t uTimestampUs_ { 0U }; ///< Timestamp in microseconds
  uint8_t uValid_ { 0U }; ///< Validity flag
};

/**
 * @brief Structure of BMM150 to save temperature compensated
 * magnetometer data value for X/Y/Z axis in Gauss unit
 */
struct SMagneticData
{
  float fXAxisVal_ { 0.0F }; ///< X Axis Mag data value

  float fYAxisVal_ { 0.0F }; ///< Y Axis Mag data value

  float fZAxisVal_ { 0.0F }; ///< Z Axis Mag data value

  uint64_t uTimeStampUs_ { 0U }; ///< Save timestamp data

  bool bDataIsValid_ { false }; ///Validity flag
};

struct SImuMeasurement
{
  float fSpecificForceX_ { 0.0F };
  float fSpecificForceY_ { 0.0F };
  float fSpecificForceZ_ { 0.0F };
  float fAngularRateX_ { 0.0F };
  float fAngularRateY_ { 0.0F };
  float fAngularRateZ_ { 0.0F };
  float fTemperature_ { 0.0F };
  uint64_t uTimestampUs_ { 0U };
  uint8_t uValid_ { 0U };
};

#pragma pack(pop)

#endif /* RTE_TYPES_H */
