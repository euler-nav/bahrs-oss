/**
 * @file CVerticalChannelMonitorSwc.h
 * @brief Declaration of the vertical channel data (height and velocity downwards) monitor software component
 * @author Fedor Baklanov
 * @date 26 July 2024
 * @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#ifndef C_VERTICAL_CHANNEL_MONITOR_SWC_H
#define C_VERTICAL_CHANNEL_MONITOR_SWC_H

#include "General/CSoftwareComponentBase.h"
#include "VerticalChannelMonitorApi.h"
#include "CRte.h"

class CVerticalChannelMonitorSwc : public CSoftwareComponent<CVerticalChannelMonitorSwc, 1U>
{
  friend class CSoftwareComponent<CVerticalChannelMonitorSwc, 1U>;
  FORBID_CLASS_COPY_AND_MOVE(CVerticalChannelMonitorSwc)
  DECLARE_MANDATORY_APIS(CVerticalChannelMonitorSwc)

public:
  /**
   * @brief Run vertical channel monitor.
  */
  void Run();

protected:

private:
  CVerticalChannelMonitorSwc() = default;
  ~CVerticalChannelMonitorSwc() = default;

  /**
   * @brief Pick vertical channel data from BAHRS filter output.
   * @param korFilterOutput BAHRS filter output
   * @return Vertical channel data
   */
  static SVerticalChannelData pickVerticalChannelDataFromFilterState(const CBahrsFilterOutput& korFilterOutput);

  /**
   * @brief A helper function to populate redundant input data object.
   * @param korVerticalChannelData Reference to vertical channel data object.
   * @param eSensorId ID of the sensor that the data originates from.
   * @param orRedundantInputData Reference to an object being populated.
   */
  static void populateRedundantInputDataObject(const SVerticalChannelData& korVerticalChannelData,
                                               NFusionLibCommon::ESensorId eSensorId,
                                               NVerticalChannelMonitorApi::CRedundantInputData& orRedundantInputData);

  /**
   * @brief A helper method to check if vertical channel data from the query sensor is safe.
   * A dataset is safe if one of the following is fulfilled for both height and downwards velocity.
   * -# Fault detection is available AND the monitor did not raise an alarm
   * -# An alarm was raised, fault isolation is available AND succeeded, the query sensor is NOT faulty.
   * 
   * @param eSensorId Query sensor ID.
   * @param korMonitorOutput Reference to fault detection and isolation results.
   * @return True if height and velocity signals are safe, false otherwise.
   */
  static bool isDatasetSafe(NFusionLibCommon::ESensorId eSensorId, const NVerticalChannelMonitorApi::COutputData& korMonitorOutput);

  /**
   * @brief A helper function for converting related types.
   * If input vertical channel data is marked invalid, then the function returns safe vertical channel data with health status "unavailable". Otherwise
   * the function copies data fields from the input vertical channel data object to the output safe vertical channel data object and sets the health
   * status to provided value.
   */
  SSafeVerticalChannelData convertVerticalChannelDataToSafeVerticalChannelData(const SVerticalChannelData& korVerticalChannelData, CSerialProtocol::ESignalHealthInfo eHealth);

  /**
   * @brief Construct vertical channel output when no safe data is available.
   * The function will set health status to "integrity risk" and populate
   * vertical channel data fields with any valid data from the monitor output struct.
   */
  static SSafeVerticalChannelData computeUnsafeVerticalChannelOutput(const NVerticalChannelMonitorApi::COutputData& korMonitorOutput);

};

#endif /* C_VERTICAL_CHANNEL_MONITOR_SWC_H */
