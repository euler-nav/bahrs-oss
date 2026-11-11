/// @file tasks_asw.cpp
/// @brief Implementation of application software tasks shared between several deployments.
/// @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CRte.h"
#include "AmsAssert.h"
#include "GetMicroseconds.h"
#include "cmsis_os.h"

extern osThreadId_t TaskBahrsFilter1Handle;
extern osThreadId_t TaskBahrsFilter2Handle;
extern osThreadId_t TaskBahrsFilter3Handle;
extern osSemaphoreId_t SemTaskBahrsFilter1Handle;
extern osSemaphoreId_t SemTaskBahrsFilter2Handle;
extern osSemaphoreId_t SemTaskBahrsFilter3Handle;
extern osSemaphoreId_t SemTask5msHandle;
extern osEventFlagsId_t EventBahrsFiltersCompletedHandle;

struct SEventFlagsBahrsFilters
{
  static constexpr uint32_t skuCompletedFilter1_{0x01}; ///< Event flag indicating that BAHRS filter 1 finished an iteration
  static constexpr uint32_t skuCompletedFilter2_{0x02}; ///< Event flag indicating that BAHRS filter 2 finished an iteration
  static constexpr uint32_t skuCompletedFilter3_{0x04}; ///< Event flag indicating that BAHRS filter 3 finished an iteration

  /// Event flags indicating all BAHRS filters finished an iteration
  static constexpr uint32_t skuAllCompleted_{skuCompletedFilter1_ | skuCompletedFilter2_ | skuCompletedFilter3_};
};

extern "C" void RunTask5ms(void* arguments)
{
  static uint32_t suCounter = 0U;

  for (;;)
  {
    osStatus_t eStatus = osSemaphoreAcquire(SemTask5msHandle, 7U);

    if ((osOK == eStatus) || (osErrorTimeout == eStatus))
    {
      // Send the software version message one time.
      if (0U == suCounter)
      {
        CRs232OutputHandler::GetInstance().SendSoftwareVersionMessage();
        CRs232OutputHandler::GetInstance().SendHardwareVersionMessage();
      }

      CImuMonitorSwc::GetInstance().Run();
      COutputTransformer::GetInstance().TransformImuSignals();

      if ((suCounter % 8) == 0U)
      {
        // Run barometer monitor at 25Hz
        CBaroMonitorSwc::GetInstance().Run();

        // Provide pressure data to BAHRS filters
        for (uint32_t uFilterIndex = 0U; uFilterIndex < CBahrsFilterSwc::skuInstanceCount_; ++uFilterIndex)
        {
          CBahrsFilterSwc::GetInstance(uFilterIndex).SetPressureInput();
        }
      }

      // Send inertial data message
      CRs232OutputHandler::GetInstance().SendInertialDataMessage();

      // Send time of inertial data at 10Hz
      if (0U == (suCounter % 20U))
      {
        CRs232OutputHandler::GetInstance().SendTimeOfInertialDataMessage();
      }
    }

    ++suCounter;
  }
}

extern "C" void RunTask10ms(void* arguments)
{
  static uint32_t suCounter = 0U;

  for(;;)
  {
    uint32_t uFlags = osEventFlagsWait(EventBahrsFiltersCompletedHandle, SEventFlagsBahrsFilters::skuAllCompleted_, osFlagsWaitAll, 20U);

    if ((osFlagsErrorTimeout == uFlags) || (SEventFlagsBahrsFilters::skuAllCompleted_ == uFlags))
    {
      COutputTransformer::GetInstance().TransformOrientation();
      CAttitudeMonitorSwc::GetInstance().Run();
      CVerticalChannelMonitorSwc::GetInstance().Run();

      CMagneticHeadingFilterSwc::GetInstance(0U).SetAttitudeInput();
      CMagneticHeadingFilterSwc::GetInstance(1U).SetAttitudeInput();
      CMagneticHeadingFilterSwc::GetInstance(2U).SetAttitudeInput();

      CMagneticHeadingFilterSwc::GetInstance(0U).Step(GetMicroseconds());
      CMagneticHeadingFilterSwc::GetInstance(1U).Step(GetMicroseconds());
      CMagneticHeadingFilterSwc::GetInstance(2U).Step(GetMicroseconds());

      CMagneticHeadingMonitorSwc::GetInstance().Run();

      // Send navigation data message
      CRs232OutputHandler::GetInstance().SendNavigationDataMessage();

      // Send accuracy and time of navigation data at 10Hz
      if (0U == (suCounter % 10U))
      {
        CRs232OutputHandler::GetInstance().SendAccuracyDataMessage();
        CRs232OutputHandler::GetInstance().SendTimeOfNavigationDataMessage();
      }
    }

    ++suCounter;
  }
}

void TaskRoutineBahrsFilter(uint32_t uFilterIndex)
{
  AMS_HARD_ASSERT(uFilterIndex < CBahrsFilterSwc::skuInstanceCount_);
  AMS_HARD_ASSERT(SemTaskBahrsFilter1Handle != NULL);
  AMS_HARD_ASSERT(SemTaskBahrsFilter2Handle != NULL);
  AMS_HARD_ASSERT(SemTaskBahrsFilter3Handle != NULL);

  const std::array<osSemaphoreId_t*, CBahrsFilterSwc::skuInstanceCount_> koSemaphoreHandlePointers{ &SemTaskBahrsFilter1Handle,
                                                                                                    &SemTaskBahrsFilter2Handle,
                                                                                                    &SemTaskBahrsFilter3Handle };

  static constexpr std::array<uint32_t, CBahrsFilterSwc::skuInstanceCount_> skoFilterEvents{ SEventFlagsBahrsFilters::skuCompletedFilter1_,
                                                                                             SEventFlagsBahrsFilters::skuCompletedFilter2_,
                                                                                             SEventFlagsBahrsFilters::skuCompletedFilter3_};

  osStatus_t eStatus = osSemaphoreAcquire(*koSemaphoreHandlePointers[uFilterIndex], osWaitForever);

  if (osOK == eStatus)
  {
    CBahrsFilterSwc::GetInstance(uFilterIndex).Step(GetMicroseconds());
    osEventFlagsSet(EventBahrsFiltersCompletedHandle, skoFilterEvents[uFilterIndex]);
  }
}

extern "C" void RunTaskBahrsFilter1(void* argument)
{
  for (;;)
  {
    TaskRoutineBahrsFilter(0U);
  }
}

extern "C" void RunTaskBahrsFilter2(void* argument)
{
  for (;;)
  {
    TaskRoutineBahrsFilter(1U);
  }
}

extern "C" void RunTaskBahrsFilter3(void* argument)
{
  for (;;)
  {
    TaskRoutineBahrsFilter(2U);
  }
}


