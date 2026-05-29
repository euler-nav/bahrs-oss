/// @file CMutex.h
/// @brief Declaration of a mutex wrapper class.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_MUTEX_H
#define C_MUTEX_H

#include "cmsis_os.h"

/// @brief A mutex wrapper class. Uses CMSIS-RTOS constructs for mutex management.
class CMutex
{
public:
  CMutex() = default;
  ~CMutex();

  CMutex(const CMutex&) = delete;
  CMutex& operator=(const CMutex&) = delete;
  CMutex(CMutex&&) = delete;
  CMutex& operator=(CMutex&&) = delete;

  /// @brief Creates and initializes the mutex.
  /// This function sets up the mutex for use in locking mechanisms. It must be called
  /// only in the initialization task.
  /// @return true if the mutex was successfully created and initialized, false otherwise.
  bool Create();

  /// @brief Checks if the mutex is initialized.
  /// @return true if the mutex is initialized, false otherwise.
  inline bool IsInitialized() const
  {
    return bIsInitialized_;
  }

  /// @brief Acquires the mutex, blocking the calling thread until the mutex is available or the timeout expires.
  /// @param uTimeoutInOsTicks The timeout duration in OS ticks to wait for the mutex.
  /// @return true if the mutex was successfully acquired within the timeout period; otherwise, false.
  bool Acquire(uint32_t uTimeoutInOsTicks);

  /// @brief Releases the mutex, allowing other threads to acquire it.
  void Release();

private:
  using CMutexControlBlock = StaticSemaphore_t;

  osMutexId_t pMutexHandle_{nullptr}; ///< Mutex ID.
  CMutexControlBlock oMutexControlBlock_; ///< Memory reserved for the mutex control block.
  const osMutexAttr_t oMutexAttributes_ { NULL, osMutexPrioInherit, &oMutexControlBlock_, sizeof(oMutexControlBlock_) }; ///< A structure with mutex attributes required for creation.
  bool bIsInitialized_{false};
};

#endif // C_MUTEX_H
