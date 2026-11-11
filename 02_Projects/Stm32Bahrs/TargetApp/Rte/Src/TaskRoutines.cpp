/// @file TaskRoutines.cpp
/// @brief Implementation of FreeRTOS task routines that wrap C++ code into C-functions.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CRte.h"
#include "AmsAssert.h"
#include "cmsis_os.h"
#include "CStartupTaskBarrier.h"

namespace
{
  void transmitFrames()
  {
#ifdef USE_DRONECAN
    CCanHandler::GetInstance().TransmitFrames();
#endif // USE_DRONECAN
  }
} // namespace

extern osSemaphoreId_t SemTaskBahrsFilter1Handle;
extern osSemaphoreId_t SemTaskBahrsFilter2Handle;
extern osSemaphoreId_t SemTaskBahrsFilter3Handle;
extern osTimerId_t TimerCyclicTaskTriggerHandle;

extern "C" void TaskRoutine1ms()
{
  // Nothing to do
}

extern "C" void TaskRoutineCanSender()
{
  static uint32_t uMillisecondCounter = 0U;
  const auto& korHandles = CCanOutputHandler::GetFrameHandles();

  for (const auto& handle : korHandles)
  {
    if ((0U < handle.uFrameFrequency_) &&
        (0U == (uMillisecondCounter % CCanOutputHandler::HerzToMs(handle.uFrameFrequency_))))
    {
      handle.pSend_();
    }
  }

  transmitFrames();

  ++uMillisecondCounter;
}

extern "C" void TaskRoutineCanReceiver()
{
  CCanHandler::ReceiveCanFrames();
}

extern "C" void TaskRoutineReceiveScha63TData()
{
  static uint32_t suCounter{ 0U };

  CScha63TDriver::GetInstance().ConvertRawDataset();
  CImuPreFilterSwc::GetInstance(0U).Run();
  CBahrsFilterSwc::GetInstance(0U).SetImuInput();

  if ((suCounter % 2) == 0)
  {
    osSemaphoreRelease(SemTaskBahrsFilter1Handle);
  }

  ++suCounter;
}

extern "C" void TaskRoutinePollBmm(uint32_t uBmmSensorIndex)
{
  CBmm150Driver::GetInstance(uBmmSensorIndex).Bmm150ReadMagData();

  // Compensate BMM150 sensor: index 0 → instance 1, index 1 → instance 2
  CMagnetometerCompensator::GetInstance(uBmmSensorIndex + 1U).CompensateMeasurements();
  COutputTransformer::GetInstance().TransformMagnetometerInput(uBmmSensorIndex + 1U);
}

extern "C" void TaskRoutinePollBmp()
{
  CBmp384Driver::GetInstance().PollSensor();
  CPressureCompensator::GetInstance(0U).CompensateMeasurements();
}

extern "C" void TaskRoutinePollMmc()
{
  CMmc5983Driver::GetInstance().PollSensor();
  CMagnetometerCompensator::GetInstance(0U).CompensateMeasurements();
  COutputTransformer::GetInstance().TransformMagnetometerInput(0U);
}

extern "C" void TaskRoutineInit()
{
  bool bStatus{CStartupTaskBarrier::suspend()};

  AMS_HARD_ASSERT(bStatus);

  if (true == bStatus)
  {
    CRte::GetInstance().Init();
    CBmp384Driver::GetInstance().Init();
    bStatus = CBmp384Driver::GetInstance().IsInitialized();
  }

  if (true == bStatus)
  {
    CMmc5983Driver::GetInstance().Init();
    bStatus = CMmc5983Driver::GetInstance().IsInitialized();
  }

  if (true == bStatus)
  {
    CScha63TDriver::GetInstance().Init();
    bStatus = CScha63TDriver::GetInstance().IsInitialized();
  }

  if (true == bStatus)
  {
    CBmm150Driver::GetInstance(0).Init();
    bStatus = CBmm150Driver::GetInstance(0).IsInitialized();
  }

  if (true == bStatus)
  {
    CBmm150Driver::GetInstance(1).Init();
    bStatus = CBmm150Driver::GetInstance(1).IsInitialized();
  }

  if (true == bStatus)
  {
    CIcm20789Driver::GetInstance(0).Init();
    bStatus = CIcm20789Driver::GetInstance(0).IsInitialized();
  }

  if (true == bStatus)
  {
    CIcm20789Driver::GetInstance(1).Init();
    bStatus = CIcm20789Driver::GetInstance(1).IsInitialized();
  }

  if (true == bStatus)
  {
    COutputTransformer::GetInstance().Init();
    bStatus = COutputTransformer::GetInstance().IsInitialized();
  }

  if (true == bStatus)
  {
    for (uint32_t uIndex = 0U; uIndex < CPressureCompensator::skuInstanceCount_; ++uIndex)
    {
      CPressureCompensator::GetInstance(uIndex).Init();
      bStatus = CPressureCompensator::GetInstance(uIndex).IsInitialized();

      if (false == bStatus)
      {
        break;
      }
    }
  }

  if (true == bStatus)
  {
    for (uint32_t uIndex = 0U; uIndex < CImuPreFilterSwc::skuInstanceCount_; ++uIndex)
    {
      CImuPreFilterSwc::GetInstance(uIndex).Init();
      bStatus = CImuPreFilterSwc::GetInstance(uIndex).IsInitialized();

      if (false == bStatus)
      {
        break;
      }
    }
  }

  if (true == bStatus)
  {
    for (uint32_t uIndex = 0U; uIndex < CMagnetometerCompensator::skuInstanceCount_; ++uIndex)
    {
      CMagnetometerCompensator::GetInstance(uIndex).Init();
      bStatus = CMagnetometerCompensator::GetInstance(uIndex).IsInitialized();

      if (false == bStatus)
      {
        break;
      }
    }
  }

  if (true == bStatus)
  {
    CCanHandler::GetInstance().Init();
    bStatus = CCanHandler::GetInstance().IsInitialized();
  }

  if (true == bStatus)
  {
    CCanOutputHandler::GetInstance().Init();
    bStatus = CCanOutputHandler::GetInstance().IsInitialized();
  }

  if (true == bStatus)
  {
    CRs232OutputHandler::GetInstance().Init();
    bStatus = CRs232OutputHandler::GetInstance().IsInitialized();
  }

  if (bStatus)
  {
    // Warm up singleton filters before releasing their consumer threads.
    CBahrsFilterSwc::GetInstance(0).Step(0);
    CBahrsFilterSwc::GetInstance(1).Step(0);
    CBahrsFilterSwc::GetInstance(2).Step(0);

    bStatus = (osOK == osTimerStart(TimerCyclicTaskTriggerHandle, 1U)) && CStartupTaskBarrier::resume();
  }

  AMS_HARD_ASSERT(bStatus);
}

extern "C" void TaskRoutineProcessSyncPulse(uint64_t uTimestampUs)
{
  CSyncPulseHandler::ProcessSyncPulse(uTimestampUs);
}
