/// @file CStartupTaskBarrier.h
/// @brief Declaration of the startup task barrier.
/// @copyright Copyright 2026. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_STARTUP_TASK_BARRIER_H
#define C_STARTUP_TASK_BARRIER_H

#ifdef BAHRS_HW_V3
extern "C" void RunTaskInit(void* pArgs);
#else
extern "C" void TaskRoutineInit();
#endif // BAHRS_HW_V3

class CStartupTaskBarrier
{
  /// @brief Constructs an unused startup barrier.
  CStartupTaskBarrier() = delete;
  CStartupTaskBarrier(const CStartupTaskBarrier&) = delete;
  CStartupTaskBarrier& operator=(const CStartupTaskBarrier&) = delete;

  /// @brief Suspends all the threads except for the init task and the idle task.
  /// @return true on success; false on failure.
  static bool suspend();

  /// @brief Resume all previously suspended threads.
  /// @return true on success; false on failure.
  static bool resume();

#ifdef BAHRS_HW_V3
  friend void RunTaskInit(void*);
#else
  friend void TaskRoutineInit();
#endif // BAHRS_HW_V3
};

#endif // C_STARTUP_TASK_BARRIER_H
