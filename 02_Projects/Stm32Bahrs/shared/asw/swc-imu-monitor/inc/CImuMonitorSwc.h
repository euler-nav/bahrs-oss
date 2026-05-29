/**
 * @file CImuMonitorSwc.h
 * @brief Implementation of the IMU monitor software component
 * @author Fedor Baklanov
 * @date 6 December 2023
 * @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#ifndef C_IMU_MONITOR_SWC_H
#define C_IMU_MONITOR_SWC_H

#include "General/CSoftwareComponentBase.h"
#include "ImuMonitorApi.h"
#include "CRte.h"

#ifdef BAHRS_HW_V3
using CRedundantImuData = NImuMonitorApi::CRedundantInputDataBahrsV3;
#elif defined(BAHRS_HW_V2)
using CRedundantImuData = NImuMonitorApi::CRedundantInputDataBahrsV2;
#else
#error "BAHRS hardware version is not defined"
#endif

class CImuMonitorSwc : public CSoftwareComponent<CImuMonitorSwc, 1U>
{
  friend class CSoftwareComponent<CImuMonitorSwc, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CImuMonitorSwc)
  DECLARE_MANDATORY_APIS(CImuMonitorSwc)

public:
  /**
   * @brief Run IMU monitor.
  */
  void Run();

protected:

private:
  CImuMonitorSwc() = default;
  ~CImuMonitorSwc() = default;

  /**
   * @brief A helper function to populate redundant input data object.
   * @param korImuData Reference to an IMU measurement.
   * @param eSensorId ID of the sensor that the measurement originates from.
   * @param orRedundantImuData Reference to an object being populated.
   */
  static void populateRedundantImuDataObject(const SImuMeasurement& korImuData,
                                             NFusionLibCommon::ESensorId eSensorId,
                                             CRedundantImuData& orRedundantImuData);

#ifdef BAHRS_HW_V3
  static constexpr NFusionLibCommon::ESensorId skeImuId1_{NFusionLibCommon::ESensorId::eScha63T};
  static constexpr NFusionLibCommon::ESensorId skeImuId2_{NFusionLibCommon::ESensorId::eBmi270};
  static constexpr NFusionLibCommon::ESensorId skeImuId3_{NFusionLibCommon::ESensorId::eAsm330};
#elif defined(BAHRS_HW_V2)
  static constexpr NFusionLibCommon::ESensorId skeImuId1_{NFusionLibCommon::ESensorId::eScha63T};
  static constexpr NFusionLibCommon::ESensorId skeImuId2_{NFusionLibCommon::ESensorId::eIcm20789Imu1};
  static constexpr NFusionLibCommon::ESensorId skeImuId3_{NFusionLibCommon::ESensorId::eIcm20789Imu2};
#else
#error "BAHRS hardware version is not defined"
#endif
};

#endif /* C_IMU_MONITOR_SWC_H */
