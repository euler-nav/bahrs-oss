/// @file CMagneticHeadingMonitorSwc.h
/// @brief Declaration of the magnetic heading monitor software component.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_MAGNETIC_HEADING_MONITOR_SWC_H
#define C_MAGNETIC_HEADING_MONITOR_SWC_H

#include "General/CSoftwareComponentBase.h"
#include "RteTypesGenerated.h"
#include "RteTypesUserDefined.h"
#include "MagneticHeadingMonitorApi.h"

namespace NMagneticHeadingMonitorApi
{
#ifdef BAHRS_HW_V3
  using namespace ::NMagneticHeadingMonitorApi::NBahrsV3;
#elif defined(BAHRS_HW_V2)
  using namespace ::NMagneticHeadingMonitorApi::NBahrsV2;
#else
#error "BAHRS hardware version is not defined"
#endif
} // namespace NMagneticHeadingMonitorApi

/// @brief The class implements a SW component that monitors outputs of the redundant magnetic heading filters
/// and provides a single magnetic heading signal with integrity information.
class CMagneticHeadingMonitorSwc : public CSoftwareComponent<CMagneticHeadingMonitorSwc, 1U>
{
  friend class CSoftwareComponent<CMagneticHeadingMonitorSwc, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CMagneticHeadingMonitorSwc)
  DECLARE_MANDATORY_APIS(CMagneticHeadingMonitorSwc)

public:
  /// @brief Run the magnetic heading monitor and write the safe magnetic heading to the RTE port.
  void Run();

protected:

private:
  CMagneticHeadingMonitorSwc() = default;
  ~CMagneticHeadingMonitorSwc() = default;

  /// @brief Check if the magnetic heading from a specified source is safe.
  /// The heading is safe if one of the following is fulfilled:
  /// -# Fault detection is available AND the monitor did not raise an alarm
  /// -# An alarm was raised, fault isolation is available AND succeeded, the query source is NOT faulty.
  ///
  /// The function asserts if the query source is not supported by the monitor (configuration issue).
  /// @param eSensorId Query source ID.
  /// @param korMonitorOutput Result of fault detection and isolation.
  /// @return True if the heading is safe, false otherwise.
  static bool isHeadingDataSafe(NFusionLibCommon::ESensorId eSensorId, const NMagneticHeadingMonitorApi::COutputData& korMonitorOutput);

  /// @brief Convert a filter output to a safe heading object.
  /// If the input heading is marked invalid, then the function returns data with health status "unavailable".
  /// @param korHeading Magnetic heading filter output.
  /// @param eHealth Health status to assign if the input is valid.
  /// @return Safe magnetic heading.
  static SSafeMagneticHeadingData convertToSafeHeading(const SMagneticHeading& korHeading, CSerialProtocol::ESignalHealthInfo eHealth);

  /// @brief Construct the heading output when no filter output is confirmed safe.
  /// The function sets the health status to "integrity risk", and populates the heading and timestamp fields with the monitor output
  /// if it is valid. Otherwise the output stays "unavailable". Raw filter outputs are intentionally not used as a fallback.
  /// @param korMonitorOutput Result of fault detection and isolation.
  /// @return Magnetic heading with integrity risk, or unavailable heading.
  static SSafeMagneticHeadingData computeUnsafeHeadingOutput(const NMagneticHeadingMonitorApi::COutputData& korMonitorOutput);

  /// @brief A helper function to populate the redundant input data object.
  /// @param korHeading Magnetic heading filter output.
  /// @param eSensorId ID of the virtual sensor that the heading originates from.
  /// @param orRedundantInputData Reference to the object being populated.
  static void populateRedundantInputDataObject(const SMagneticHeading& korHeading,
                                               NFusionLibCommon::ESensorId eSensorId,
                                               NMagneticHeadingMonitorApi::CRedundantInputData& orRedundantInputData);
};

#endif // C_MAGNETIC_HEADING_MONITOR_SWC_H
