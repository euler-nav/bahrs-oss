/**
 * @file task_routines.cpp
 * @brief Implementation of the task functions.
 * @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#include "CRte.h"
#include "AmsAssert.h"
#include "CI2CHandler.h"
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

extern osThreadId_t TaskInitHandle;
extern osTimerId_t TimerCyclicTaskTriggerHandle;
extern osSemaphoreId_t asm330lhhInt1SemHandle;
extern osMessageQueueId_t QueueTaskAsm330TimeStampHandle;
extern osMessageQueueId_t QueueTaskBmp384TimeStampHandle;
extern osMessageQueueId_t QueueTaskBmi270TimeStampHandle;
extern osMessageQueueId_t QueueTaskLps22TimeStampHandle;
extern osMessageQueueId_t QueueTaskIcp20100TimeStampHandle;
extern osMessageQueueId_t QueueTaskMmc5983TimestampHandle;
extern osMessageQueueId_t QueueTaskLis3TimestampHandle;
extern osMessageQueueId_t QueueTaskBmm350TimestampHandle;
extern osMessageQueueId_t QueueTaskProcessSyncHandle;
extern osSemaphoreId_t SemBmp384ReceiveTaskHandle;
extern osSemaphoreId_t SemTaskReceiveScha63TDataHandle;
extern osSemaphoreId_t SemTaskCanSenderHandle;
extern osSemaphoreId_t SemTask5msHandle;
extern osSemaphoreId_t SemTaskBahrsFilter1Handle;
extern osSemaphoreId_t SemTaskBahrsFilter2Handle;
extern osSemaphoreId_t SemTaskBahrsFilter3Handle;
extern osSemaphoreId_t SemLis3mdlDrdyHandle;
extern osSemaphoreId_t SemI2c1RxCompleteHandle;
extern osSemaphoreId_t SemI2c2RxCompleteHandle;
extern osSemaphoreId_t SemI2c3RxCompleteHandle;


namespace
{

void drainSemaphore(osSemaphoreId_t SemHandle)
{
  osSemaphoreAcquire(SemHandle, 0U);
}

template<CI2CHandler::EBus eBus, class CSoftwareComponent, unsigned uInstance = 0U>
void recoverI2cBusAndReinitializeSwc()
{
  const osThreadId_t kopTaskId{osThreadGetId()};
  const osPriority_t keTaskPriority{osThreadGetPriority(kopTaskId)};

  if (osPriorityError != keTaskPriority)
  {
    osThreadSetPriority(kopTaskId, osPriorityBelowNormal);
  }

  bool bNeedToContinue = CI2CHandler::GetInstance().DeinitializeBus(eBus);

  if (bNeedToContinue)
  {
    osDelay(50U);
    bNeedToContinue = CI2CHandler::GetInstance().InitializeBus(eBus);
  }

  if (bNeedToContinue)
  {
    osDelay(50U);
    CSoftwareComponent::GetInstance(uInstance).Init();
  }

  if (osPriorityError != keTaskPriority)
  {
    osThreadSetPriority(kopTaskId, keTaskPriority);
  }
}

} // namespace

static constexpr uint32_t skuPeriod100msInTicks = static_cast<uint32_t>(0.1F * static_cast<float>(configTICK_RATE_HZ));
static constexpr uint32_t skuPeriod40msInTicks = static_cast<uint32_t>(0.04F * static_cast<float>(configTICK_RATE_HZ));
static constexpr uint32_t skuPeriod10msInTicks = static_cast<uint32_t>(0.01F * static_cast<float>(configTICK_RATE_HZ));
static constexpr uint32_t skuPeriod5msInTicks = static_cast<uint32_t>(0.005F * static_cast<float>(configTICK_RATE_HZ));

extern "C" void RunTaskInit(void *argument)
{
  bool bStatus = false;

  for(;;)
  {
    bStatus = CStartupTaskBarrier::suspend();

    AMS_HARD_ASSERT(bStatus);

    if (bStatus)
    {
      CRte::GetInstance().Init();
      CI2CHandler::GetInstance().Init();
      bStatus = CI2CHandler::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CSpiHandler::GetInstance().Init();
      bStatus = CSpiHandler::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CBmi270Driver::GetInstance().Init();
      bStatus = CBmi270Driver::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CAsm330lhhDriver::GetInstance().Init();
      bStatus = CAsm330lhhDriver::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CScha63TDriver::GetInstance().Init();
      bStatus = CScha63TDriver::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CLps22hhDriver::GetInstance().Init();
      bStatus = CLps22hhDriver::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CBmp384Driver::GetInstance().Init();
      bStatus = CBmp384Driver::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CIcp20100Driver::GetInstance().Init();
      bStatus = CIcp20100Driver::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CMmc5983Driver::GetInstance().Init();
      bStatus = CMmc5983Driver::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CLis3mdlDriver::GetInstance().Init();
      bStatus = CLis3mdlDriver::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CBmm350Driver::GetInstance().Init();
      bStatus = CBmm350Driver::GetInstance().IsInitialized();
    }

    if (bStatus)
    {
      CRs232OutputHandler::GetInstance().Init();
      bStatus = CRs232OutputHandler::GetInstance().IsInitialized();
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

    if (bStatus)
    {
      // Warm up singleton filters before releasing their consumer threads.
      CBahrsFilterSwc::GetInstance(0).Step(0);
      CBahrsFilterSwc::GetInstance(1).Step(0);
      CBahrsFilterSwc::GetInstance(2).Step(0);

      bStatus = (osOK == osTimerStart(TimerCyclicTaskTriggerHandle, 1U)) && CStartupTaskBarrier::resume();
    }

    AMS_HARD_ASSERT(bStatus);

    osThreadSuspend(TaskInitHandle);
  }
}

extern "C" void TimerCyclicTaskTriggerCallback(void* argument)
{
  static uint32_t suCounter{0U};

  // Trigger 1ms tasks
  osSemaphoreRelease(SemTaskCanSenderHandle);

  // Trigger 5ms tasks
  if (0U == (suCounter % skuPeriod5msInTicks))
  {
    osSemaphoreRelease(SemTask5msHandle);
  }

  // Trigger 100ms tasks
  if (0U == (suCounter % skuPeriod100msInTicks))
  {

  }

  ++suCounter;
}

extern "C" void RunTaskBmi270(void* argument)
{
  uint32_t uCounter{0U};
  static constexpr uint32_t skuBmi270DataReadyTimeout{50U};

  for(;;)
  {
    static uint64_t suTimestamp = 0U;

    osStatus_t eStatus = osMessageQueueGet(QueueTaskBmi270TimeStampHandle, &suTimestamp, 0U, skuBmi270DataReadyTimeout);

    if (osOK == eStatus)
    {
      CBmi270Driver::GetInstance().PollSensor(suTimestamp);
    }
    else
    {
      CBmi270Driver::GetInstance().InvalidatePortData();
    }

    CImuPreFilterSwc::GetInstance(1U).Run();
    CBahrsFilterSwc::GetInstance(1U).SetImuInput();

    if ((uCounter % 2) == 0)
    {
      osSemaphoreRelease(SemTaskBahrsFilter2Handle);
    }

    ++uCounter;
  }
}

extern "C" void RunTaskAsm330(void* argument)
{
  uint32_t uCounter{0U};
  static constexpr uint32_t skuAsm330lhhDataReadyTimeout{50U};

  for(;;)
  {
    static uint64_t suTimestamp = 0U;

    // Wait for gyroscope interrupt and timestamp
    osStatus_t eStatus = osMessageQueueGet(QueueTaskAsm330TimeStampHandle, &suTimestamp, 0U, skuAsm330lhhDataReadyTimeout);
  
    if (osOK == eStatus)
    {
      // Wait for accelerometer interrupt, gyro and accel interrupts are almost simultaneous
      eStatus = osSemaphoreAcquire(asm330lhhInt1SemHandle, 3U);

      if (osOK == eStatus)
      {
        CAsm330lhhDriver::GetInstance().PollSensor(suTimestamp);
      }
      else
      {
        CAsm330lhhDriver::GetInstance().InvalidatePortData();
      }
    }
    else
    {
      CAsm330lhhDriver::GetInstance().InvalidatePortData();
    }

    CImuPreFilterSwc::GetInstance(2U).Run();
    CBahrsFilterSwc::GetInstance(2U).SetImuInput();

    if ((uCounter % 2) == 0)
    {
      osSemaphoreRelease(SemTaskBahrsFilter3Handle);
    }

    ++uCounter;
  }
}

extern "C" void RunTaskLps22(void* argument)
{
  static constexpr uint32_t skuLps22DataReadyTimeout{150U};

  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    
    osStatus_t eStatus = osMessageQueueGet(QueueTaskLps22TimeStampHandle, &suTimestamp, 0U, skuLps22DataReadyTimeout);

    if (osOK == eStatus)
    {
      CLps22hhDriver::GetInstance().PollSensor(suTimestamp);
    }
    else
    {
      CLps22hhDriver::GetInstance().InvalidatePortData();
    }

    CPressureCompensator::GetInstance(2U).CompensateMeasurements();
  }
}

extern "C" void RunTaskBmp384(void *argument)
{
  static constexpr uint32_t skuBmp384DataReadyTimeout{150U};

  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskBmp384TimeStampHandle, &suTimestamp, 0U, skuBmp384DataReadyTimeout);

    if (osOK == eStatus)
    {
      CBmp384Driver::GetInstance().PollSensor(suTimestamp);
    }
    else
    {
      CBmp384Driver::GetInstance().InvalidatePortData();
    }

    CPressureCompensator::GetInstance(1U).CompensateMeasurements();
  }
}

extern "C" void RunTaskScha63T(void* argument)
{
  uint32_t uCounter{ 0U };

  for(;;)
  {
    osStatus_t eStatus = osSemaphoreAcquire(SemTaskReceiveScha63TDataHandle, osWaitForever);

    if (osOK == eStatus)
    {
        CScha63TDriver::GetInstance().ConvertRawDataset();
        CImuPreFilterSwc::GetInstance(0U).Run();
        CBahrsFilterSwc::GetInstance(0U).SetImuInput();

        if ((uCounter % 2) == 0)
        {
          osSemaphoreRelease(SemTaskBahrsFilter1Handle);
        }

        ++uCounter;
    }
  }
}

extern "C" void RunTaskIcp20100(void *argument)
{
  static constexpr uint32_t skuIcp20100DataReadyTimeout{150U};

  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskIcp20100TimeStampHandle, &suTimestamp, 0U, skuIcp20100DataReadyTimeout);

    if (osOK == eStatus)
    {
      CIcp20100Driver::GetInstance().PollSensor(suTimestamp);
    }
    else
    {
      CIcp20100Driver::GetInstance().InvalidatePortData();
    }

    CPressureCompensator::GetInstance(0U).CompensateMeasurements();
  }
}

extern "C" void RunTaskMmc5983(void* argument)
{
  static constexpr uint32_t skuMmc5983DataReadyTimeout{300U};
  static constexpr uint32_t skuMmc5983DmaTimeout{3U};

  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskMmc5983TimestampHandle, &suTimestamp, 0U, skuMmc5983DataReadyTimeout);
    bool bNeedToRecoverFromError{false};

    if (osOK == eStatus)
    {
      drainSemaphore(SemI2c1RxCompleteHandle);
      bool bStatus = CMmc5983Driver::GetInstance().PollSensorDma();

      if (bStatus)
      {
        eStatus = osSemaphoreAcquire(SemI2c1RxCompleteHandle, skuMmc5983DmaTimeout);

        if (osOK == eStatus)
        {
          CMmc5983Driver::GetInstance().ProcessReceivedBytesDma(suTimestamp);
        }
        else
        {
          drainSemaphore(SemI2c1RxCompleteHandle);
          CMmc5983Driver::GetInstance().InvalidatePortData();
          bNeedToRecoverFromError = true;
        }
      }
      else
      {
        CMmc5983Driver::GetInstance().InvalidatePortData();
        bNeedToRecoverFromError = true;
      }
    }
    else
    {
      CMmc5983Driver::GetInstance().InvalidatePortData();
      bNeedToRecoverFromError = true;
    }

    CMagnetometerCompensator::GetInstance(0U).CompensateMeasurements();
    COutputTransformer::GetInstance().TransformMagnetometerInput(0U);

    if (bNeedToRecoverFromError)
    {
      recoverI2cBusAndReinitializeSwc<CI2CHandler::EBus::eI2c1, CMmc5983Driver>();
    }
  }
}

extern "C" void RunTaskLis3(void* argument)
{
  static constexpr uint32_t skuLis3DataReadyTimeout{300U};
  static constexpr uint32_t skuLis3DmaTimeout{3U};

  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskLis3TimestampHandle, &suTimestamp, 0U, skuLis3DataReadyTimeout);
    bool bNeedToRecoverFromError{false};

    if (osOK == eStatus)
    {
      // Request sensor data over I2C DMA
      drainSemaphore(SemI2c3RxCompleteHandle);
      bool bStatus = CLis3mdlDriver::GetInstance().PollSensorDma();

      if (bStatus)
      {
        // Wait until I2C DMA is finished
        eStatus = osSemaphoreAcquire(SemI2c3RxCompleteHandle, skuLis3DmaTimeout);

        if (osOK == eStatus)
        {
          CLis3mdlDriver::GetInstance().ProcessReceivedBytesDma(suTimestamp);
        }
        else
        {
          drainSemaphore(SemI2c3RxCompleteHandle); // In case the DMA interrupt fired after timeout
          CLis3mdlDriver::GetInstance().InvalidatePortData();
          bNeedToRecoverFromError = true;
        }
      }
      else
      {
        CLis3mdlDriver::GetInstance().InvalidatePortData();
        bNeedToRecoverFromError = true;
      }
    }
    else
    {
      CLis3mdlDriver::GetInstance().InvalidatePortData();
      bNeedToRecoverFromError = true;
    }

    CMagnetometerCompensator::GetInstance(2U).CompensateMeasurements();
    COutputTransformer::GetInstance().TransformMagnetometerInput(2U);

    if (bNeedToRecoverFromError)
    {
      recoverI2cBusAndReinitializeSwc<CI2CHandler::EBus::eI2c3, CLis3mdlDriver>();
    }
  }
}

extern "C" void RunTaskBmm350(void* argument)
{
  static constexpr uint32_t skuBmm350DataReadyTimeout{300U};
  static constexpr uint32_t skuBmm350DmaTimeout{3U};

  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskBmm350TimestampHandle, &suTimestamp, 0U, skuBmm350DataReadyTimeout);
    bool bNeedToRecoverFromError{false};

    if (osOK == eStatus)
    {
      // Request sensor data over I2C DMA
      drainSemaphore(SemI2c2RxCompleteHandle);
      bool bStatus = CBmm350Driver::GetInstance().PollSensorDma();

      if (bStatus)
      {
        // Wait until I2C DMA is finished
        eStatus = osSemaphoreAcquire(SemI2c2RxCompleteHandle, skuBmm350DmaTimeout);

        if (osOK == eStatus)
        {
          CBmm350Driver::GetInstance().ProcessReceivedBytesDma(suTimestamp);
        }
        else
        {
          drainSemaphore(SemI2c2RxCompleteHandle);  // In case the DMA interrupt fired after timeout
          CBmm350Driver::GetInstance().InvalidatePortData();
          bNeedToRecoverFromError = true;
        }
      }
      else
      {
        CBmm350Driver::GetInstance().InvalidatePortData();
        bNeedToRecoverFromError = true;
      }
    }
    else
    {
      CBmm350Driver::GetInstance().InvalidatePortData();
      bNeedToRecoverFromError = true;
    }

    CMagnetometerCompensator::GetInstance(1U).CompensateMeasurements();
    COutputTransformer::GetInstance().TransformMagnetometerInput(1U);

    if (bNeedToRecoverFromError)
    {
      recoverI2cBusAndReinitializeSwc<CI2CHandler::EBus::eI2c2, CBmm350Driver>();
    }
  }
}

extern "C" void RunTaskCanSender(void* argument)
{
  static uint32_t uMillisecondCounter = 0U;
  const auto& korHandles = CCanOutputHandler::GetFrameHandles();

  for(;;)
  {
    const osStatus_t eStatus = osSemaphoreAcquire(SemTaskCanSenderHandle, osWaitForever);

    if (osOK == eStatus)
    {
      for (const auto& handle : korHandles)
      {
        if ((0U < handle.uFrameFrequency_) &&
            (0U == (uMillisecondCounter % CCanOutputHandler::HerzToMs(handle.uFrameFrequency_))))
        {
          handle.pSend_();
        }
      }

      transmitFrames();
    }

    ++uMillisecondCounter;
  }
}

extern "C" void RunTaskCanReceiver(void* argument)
{
  for (;;)
  {
    CCanHandler::ReceiveCanFrames();
  }
}

extern "C" void RunTaskProcessSyncPulse(void* argument)
{
  for (;;)
  {
    uint64_t uTimestampUs = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskProcessSyncHandle, &uTimestampUs, NULL, osWaitForever);

    if (osOK == eStatus)
    {
      CSyncPulseHandler::ProcessSyncPulse(uTimestampUs);
    }
  }
}
