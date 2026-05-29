/// @file CPressureCompensator.h
/// @brief Declaration of the pressure compensator SWC class.
/// @copyright Copyright 2026 AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_PRESSURE_COMPENSATOR_H
#define C_PRESSURE_COMPENSATOR_H

#include <stdint.h>

#include "General/CSoftwareComponentBase.h"

struct SBarometerMeasurement;

/// @brief The class implements a SW component that applies pressure offsets to barometer measurements.
class CPressureCompensator : public CSoftwareComponent<CPressureCompensator, 3U>
{
  friend class CSoftwareComponent<CPressureCompensator, 3U>;
  FORBID_CLASS_COPY_AND_MOVE(CPressureCompensator)
  DECLARE_MANDATORY_APIS(CPressureCompensator)

public:
  /// @brief Pressure sensor ID for compensator instances.
  enum class ESensorId : uint8_t
  {
    ePressureInput1 = 0U,
    ePressureInput2 = 1U,
    ePressureInput3 = 2U
  };

  /// @brief Apply pressure offsets to barometer measurements.
  void CompensateMeasurements();

private:
  /// @brief Create a compensator for a specific pressure sensor.
  /// @param eSensorId Pressure sensor ID.
  explicit CPressureCompensator(ESensorId eSensorId);
  ~CPressureCompensator() = default;

  /// @brief Read pressure data from the input port.
  /// @param orInput Pressure measurement read from the port.
  /// @return True if read succeeded.
  bool readPressureDataPort(SBarometerMeasurement& orInput) const;

  /// @brief Write pressure data to the compensated output port.
  /// @param orOutput Pressure measurement to write.
  void writePressureDataPort(const SBarometerMeasurement& orOutput) const;

  /// @brief Convert sensor ID to a zero-based index.
  /// @param eSensorId Pressure sensor ID.
  /// @return Zero-based index.
  static constexpr uint32_t toIndex(ESensorId eSensorId)
  {
    return static_cast<uint32_t>(eSensorId);
  }

  const ESensorId keSensorId_; ///< Pressure sensor ID assigned to this instance
  bool bIsInitialized_ { false }; ///< True -- SWC is initialized, false otherwise
  float fPressureOffsetPa_ { 0.0F }; ///< Pressure offset in Pascals
};

#endif // C_PRESSURE_COMPENSATOR_H
