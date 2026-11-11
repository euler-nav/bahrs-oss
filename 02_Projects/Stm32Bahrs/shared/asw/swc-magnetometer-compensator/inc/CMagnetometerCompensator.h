/// @file CMagnetometerCompensator.h
/// @brief Declaration of the magnetometer Compensator SWC class.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_MAGNETOMETER_COMPENSATOR_H
#define C_MAGNETOMETER_COMPENSATOR_H

#include "Eigen/Dense"
#include "General/CSoftwareComponentBase.h"

/// @brief The class implements a SW component that applies calibration parameters to signals of magnetometers.
class CMagnetometerCompensator : public CSoftwareComponent<CMagnetometerCompensator, 3U>
{
  friend class CSoftwareComponent<CMagnetometerCompensator, 3U>;
  FORBID_CLASS_COPY_AND_MOVE(CMagnetometerCompensator)
  DECLARE_MANDATORY_APIS(CMagnetometerCompensator)

public:
  /// @brief Magnetometer sensor ID for compensator instances.
  enum class ESensorId : uint8_t
  {
    eMagnetometer1 = 0U,
    eMagnetometer2 = 1U,
    eMagnetometer3 = 2U
  };

  /// @brief Apply calibration parameters to magnetometer measurements.
  void CompensateMeasurements();

protected:

private:
  /// @brief Read magnetometer data from the input port.
  /// @param orInput Magnetic measurement read from the port.
  /// @return True if read succeeded.
  bool readMagnetometerDataPort(SMagneticMeasurement& orInput) const;

  /// @brief Write magnetometer data to the compensated output port.
  /// @param orOutput Magnetic measurement to write.
  void writeMagnetometerDataPort(const SMagneticMeasurement& orOutput) const;

  /// @brief Convert sensor ID to a zero-based index.
  /// @param eSensorId Magnetometer sensor ID.
  /// @return Zero-based index.
  static constexpr uint32_t toIndex(ESensorId eSensorId)
  {
    return static_cast<uint32_t>(eSensorId);
  }

  /// @brief Create a compensator for a specific magnetometer sensor.
  /// @param eSensorId Magnetometer sensor ID.
  explicit CMagnetometerCompensator(ESensorId eSensorId);

  ~CMagnetometerCompensator() = default;

  const ESensorId keSensorId_; ///< Magnetometer sensor ID assigned to this instance
  bool bIsInitialized_ { false }; ///< True -- SWC is initialized, false otherwise
  Eigen::Vector3f oHardIron_; ///< Hard-iron compensation parameters
  Eigen::Matrix3f oSoftIron_; ///< Soft-iron compensation parameters
};

#endif // C_MAGNETOMETER_COMPENSATOR_H
