/**
* @file CommonMonitorTypes.h
* @brief Type declaration common to all sensor monitors.
* @author Fedor Baklanov
* @date 11 December 2023
* @copyright Copyright 2023 AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef COMMON_MONITOR_TYPES_H
#define COMMON_MONITOR_TYPES_H

namespace NMonitorTypes
{
  /**
   * @brief Fault detection result
  */
  enum class EDetectionResult : int
  {
    eInvalid = 0, ///< Invalid (default) value
    eUnavailable, ///< Fault detection function is unavailable
    eGood,///< No failure detected
    eFailure ///< Failure detected
  };

  /**
   * @brief Fault isolation result
  */
  enum class EIsolationResult : int
  {
    eInvalid = 0, ///< Invalid (default) value
    eUnavailable, ///< Fault isolation function is unavailable
    eGood, ///< Fault isolation succeeded
    eFailed ///< Fault isolation failed
  };
}

#endif /* COMMON_MONITOR_TYPES_H */
