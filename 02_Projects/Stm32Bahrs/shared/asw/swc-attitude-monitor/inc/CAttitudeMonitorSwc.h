/**
 * @file CAttitudeMonitorSwc.h
 * @brief Declaration of the attitude monitor software component.
 * @author Fedor Baklanov
 * @date 26 July 2024
 * @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#ifndef C_ATTITUDE_MONITOR_SWC_H
#define C_ATTITUDE_MONITOR_SWC_H

#include "General/CSoftwareComponentBase.h"
#include "AttitudeMonitorApi.h"
#include "CRte.h"

class CAttitudeMonitorSwc : public CSoftwareComponent<CAttitudeMonitorSwc, 1U>
{
  friend class CSoftwareComponent<CAttitudeMonitorSwc, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CAttitudeMonitorSwc)
  DECLARE_MANDATORY_APIS(CAttitudeMonitorSwc)

public:
  /**
   * @brief Run attitude monitor.
  */
  void Run();

protected:

private:
  CAttitudeMonitorSwc() = default;
  ~CAttitudeMonitorSwc() = default;

  /**
   * @brief Check if attitude dataset from a specified source is safe.
   * A dataset is safe if one of the following is fulfilled for both roll and pitch signals.
   * -# Fault detection is available AND the monitor did not raise an alarm
   * -# An alarm was raised, fault isolation is available AND succeeded, the query sensor is NOT faulty.
   * 
   * The function asserts if the query sensor is not supported by the monitor (configuration issue).
   * 
   * @param eSensorId Query sensor ID.
   * @param korMonitorOutput Result of fault detection and isolation.
   * @return True if the measurement is safe, false otherwise.
   */
  static bool isAttitudeDataSafe(NFusionLibCommon::ESensorId eSensorId, const NAttitudeMonitorApi::COutputData& korMonitorOutput);

  /**
   * @brief A helper function for converting related types.
   * If input attitude is marked invalid, then the function returns safe attitude with health status "unavailable". Otherwise
   * the function copies data fields from the input attitude object to the output safe attitude object and sets the health
   * status to provided value.
   */
  static SSafeAttitudeData convertAttitudeToSafeAttitude(const SAttitudeData& korAttitude, CSerialProtocol::ESignalHealthInfo eHealth);

  /**
   * @brief Construct attitude output when no safe data is available.
   * The function will set health status to "integrity risk", assign large standard deviations for roll and pitch, and populate
   * roll and pitch fields with any valid data from the monitor output struct.
   */
  static SSafeAttitudeData computeUnsafeAttitudeOutput(const NAttitudeMonitorApi::COutputData& korMonitorOutput,
                                                       const SAttitudeData& korAttitude1,
                                                       const SAttitudeData& korAttitude2,
                                                       const SAttitudeData& korAttitude3);

  /// @brief A helper function to populate redundant input data object.
  /// @param korAttitudeData Reference to an attitude data structure.
  /// @param eSensorId ID of the sensor that the measurement originates from.
  /// @param orRedundantInputData Reference to an object being populated.
  static void populateRedundantInputDataObject(const SAttitudeData& korAttitudeData,
                                               NFusionLibCommon::ESensorId eSensorId,
                                               NAttitudeMonitorApi::CRedundantInputData& orRedundantInputData);
};

#endif /* C_ATTITUDE_MONITOR_SWC_H */
