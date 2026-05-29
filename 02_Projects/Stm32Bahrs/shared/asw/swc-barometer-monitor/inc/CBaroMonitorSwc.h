/// @file CBaroMonitorSwc.h
/// @brief Declaration of the barometer monitor software component.
/// @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_BARO_MONITOR_SWC_H
#define C_BARO_MONITOR_SWC_H

#include "General/CSoftwareComponentBase.h"
#include "BarometerMonitorApi.h"
#include "CRte.h"

#ifdef BAHRS_HW_V3
using CRedundantPressureData = NBaroMonitorApi::CRedundantInputDataBahrsV3;
#elif defined(BAHRS_HW_V2)
using CRedundantPressureData = NBaroMonitorApi::CRedundantInputDataBahrsV2;
#else
#error "BAHRS hardware version is not defined"
#endif

class CBaroMonitorSwc : public CSoftwareComponent<CBaroMonitorSwc, 1U>
{
  friend class CSoftwareComponent<CBaroMonitorSwc, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CBaroMonitorSwc)
  DECLARE_MANDATORY_APIS(CBaroMonitorSwc)

public:

  /// @brief Run barometer monitor.
  void Run();

protected:

private:
  CBaroMonitorSwc() = default;
  ~CBaroMonitorSwc() = default;


  /// @brief Check if pressure signal from a specified sensor is safe.
  /// A signal is safe if one of the following is fulfilled
  /// -# Fault detection is available AND the monitor did not raise an alarm
  /// -# An alarm was raised, fault isolation is available AND succeeded, the query sensor is NOT faulty.
  /// 
  /// The function asserts if the query sensor is not supported by the monitor (configuration issue).
  /// 
  /// @param eSensorId Query sensor ID.
  /// @param korMonitorOutput Result of fault detection and isolation.
  /// @return True if the measurement is safe, false otherwise.   
  static bool isPressureMeasurementSafe(NFusionLibCommon::ESensorId eSensorId, const NBaroMonitorApi::COutputData& korMonitorOutput);

  /// @brief A helper function to populate redundant input data object.
  /// @param korPressureData Reference to a barometer measurement.
  /// @param eSensorId ID of the sensor that the measurement originates from.
  /// @param orRedundantPressureData Reference to an object being populated.
  static void populateRedundantPressureDataObject(const SBarometerMeasurement& korPressureData,
                                                  NFusionLibCommon::ESensorId eSensorId,
                                                  CRedundantPressureData& orRedundantPressureData);

#ifdef BAHRS_HW_V3
  static constexpr NFusionLibCommon::ESensorId skeBarometerId1_{NFusionLibCommon::ESensorId::eIcp20100};
  static constexpr NFusionLibCommon::ESensorId skeBarometerId2_{NFusionLibCommon::ESensorId::eBmp384};
  static constexpr NFusionLibCommon::ESensorId skeBarometerId3_{NFusionLibCommon::ESensorId::eLps22};
#elif defined(BAHRS_HW_V2)
  static constexpr NFusionLibCommon::ESensorId skeBarometerId1_{NFusionLibCommon::ESensorId::eBmp384};
  static constexpr NFusionLibCommon::ESensorId skeBarometerId2_{NFusionLibCommon::ESensorId::eIcm20789Baro1};
  static constexpr NFusionLibCommon::ESensorId skeBarometerId3_{NFusionLibCommon::ESensorId::eIcm20789Baro2};
#else
#error "BAHRS hardware version is not defined"
#endif
};

#endif /* C_BARO_MONITOR_SWC_H */
