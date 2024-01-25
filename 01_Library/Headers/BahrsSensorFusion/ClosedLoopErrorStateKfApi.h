/**
* @file ClosedLoopErrorStateKfApi.h
* @brief Declaration of filter modes and high-level filter interface types.
* @author Fedor Baklanov
* @date 6 December 2023
* @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef CLOSED_LOOP_ERROR_STATE_KF_API_H
#define CLOSED_LOOP_ERROR_STATE_KF_API_H

namespace CClosedLoopErrorStateKfApi
{
  /**
   * @brief Enumeration of filter modes.
  */
  enum class EFilterModes : unsigned
  {
    IDLE = 0U, ///< Idle, i. e. does nothing.
    INIT, ///< The filter initializes.
    RUNNING ///< The filter is running.
  };
}

#endif /* CLOSED_LOOP_ERROR_STATE_KF_API_H */
