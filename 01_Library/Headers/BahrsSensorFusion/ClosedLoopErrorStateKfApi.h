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
