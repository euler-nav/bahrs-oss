/**
 * @file task_routines.cpp
 * @brief Implementation of the task functions.
 * @copyright Copyright 2025. AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#include "CRte.h"
#include "AmsAssert.h"
#include "CI2CHandler.h"

extern osThreadId_t TaskInitHandle;
extern osSemaphoreId_t asm330lhhInt1SemHandle;
extern osMessageQueueId_t QueueTaskRs232SenderHandle;
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
extern osSemaphoreId_t SemLis3mdlDrdyHandle;

static constexpr uint32_t skuPeriod100msInTicks = static_cast<uint32_t>(0.1F * static_cast<float>(configTICK_RATE_HZ));
static constexpr uint32_t skuPeriod40msInTicks = static_cast<uint32_t>(0.04F * static_cast<float>(configTICK_RATE_HZ));
static constexpr uint32_t skuPeriod10msInTicks = static_cast<uint32_t>(0.01F * static_cast<float>(configTICK_RATE_HZ));
static constexpr uint32_t skuPeriod5msInTicks = static_cast<uint32_t>(0.005F * static_cast<float>(configTICK_RATE_HZ));

extern "C" void RunTaskInit(void *argument)
{
  bool bStatus = false;

  for(;;)
  {
    CRte::GetInstance().Init();

    CI2CHandler::GetInstance().Init();
    bStatus = CI2CHandler::GetInstance().IsInitialized();

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


    if (false == bStatus)
    {
      AMS_HARD_ASSERT(false);
    }

    // We call the lines below in order to force initialization of filters
    // declared as static objects within a singleton class. It must be done
    // before we start getting references to the static objects from multiple threads.
    CBahrsFilterSwc::GetInstance(0).Step(0);

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
  for(;;)
  {
    static uint64_t suTimestamp = 0U;

    osStatus_t eStatus = osMessageQueueGet(QueueTaskBmi270TimeStampHandle, &suTimestamp, 0U, osWaitForever);
    if (osOK == eStatus)
    {
      CBmi270Driver::GetInstance().PollSensor(suTimestamp);
    }
  }
}

extern "C" void RunTaskAsm330(void* argument)
{
  for(;;)
  {
    static uint64_t suTimestamp = 0U;

    // Wait for gyroscope interrupt and timestamp
    osStatus_t eStatus = osMessageQueueGet(QueueTaskAsm330TimeStampHandle, &suTimestamp, 0U, osWaitForever);
    if (osOK == eStatus)
    {
      // Wait for accelerometer interrupt
      eStatus = osSemaphoreAcquire(asm330lhhInt1SemHandle, osWaitForever);
      if (osOK == eStatus)
      {
        CAsm330lhhDriver::GetInstance().PollSensor(suTimestamp);
      }
    }
  }
}

extern "C" void RunTaskLps22(void* argument)
{
  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    
    osStatus_t eStatus = osMessageQueueGet(QueueTaskLps22TimeStampHandle, &suTimestamp, 0U, osWaitForever);
    if (osOK == eStatus)
    {
        CLps22hhDriver::GetInstance().PollSensor(suTimestamp);
        CPressureCompensator::GetInstance(2U).CompensateMeasurements();
    }
  }
}

extern "C" void RunTaskRs232Sender(void* argument)
{
  for(;;)
  {
    uint8_t uMessageId = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskRs232SenderHandle, &uMessageId, NULL, osWaitForever);

    if (osOK == eStatus)
    {
      const CSerialProtocol::EMessageIds eMessageId = static_cast<CSerialProtocol::EMessageIds>(uMessageId);

      switch (eMessageId)
      {
        case CSerialProtocol::EMessageIds::eInertialData:
          CRs232OutputHandler::GetInstance().SendInertialDataMessage();
          break;
        case CSerialProtocol::EMessageIds::eTimeOfInertialData:
          CRs232OutputHandler::GetInstance().SendTimeOfInertialDataMessage();
          break;
        case CSerialProtocol::EMessageIds::eNavigationData:
          CRs232OutputHandler::GetInstance().SendNavigationDataMessage();
          break;
        case CSerialProtocol::EMessageIds::eTimeOfNavigationData:
          CRs232OutputHandler::GetInstance().SendTimeOfNavigationDataMessage();
          break;
        case CSerialProtocol::EMessageIds::eAccuracy:
          CRs232OutputHandler::GetInstance().SendAccuracyDataMessage();
          break;
        case CSerialProtocol::EMessageIds::eTimeOfSyncPulse:
          CRs232OutputHandler::GetInstance().SendTimeOfLatestSyncPulseMessage();
          break;
        case CSerialProtocol::EMessageIds::eSoftwareVersion:
          CRs232OutputHandler::GetInstance().SendSoftwareVersionMessage();
          break;
        default:
          break;
      }
    }
  }
}

extern "C" void RunTaskBmp384(void *argument)
{
  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskBmp384TimeStampHandle, &suTimestamp, 0U, osWaitForever);

    if (osOK == eStatus)
    {
      CBmp384Driver::GetInstance().PollSensor(suTimestamp);
      CPressureCompensator::GetInstance(1U).CompensateMeasurements();
    }
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
  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskIcp20100TimeStampHandle, &suTimestamp, 0U, osWaitForever);

    if (osOK == eStatus)
    {
      CIcp20100Driver::GetInstance().PollSensor(suTimestamp);
      CPressureCompensator::GetInstance(0U).CompensateMeasurements();
    }
  }
}

extern "C" void RunTaskMmc5983(void* argument)
{
  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskMmc5983TimestampHandle, &suTimestamp, 0U, osWaitForever);
    if (osOK == eStatus)
    {
      CMmc5983Driver::GetInstance().PollSensor(suTimestamp);
    }
  }
}

extern "C" void RunTaskLis3(void* argument)
{
  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskLis3TimestampHandle, &suTimestamp, 0U, osWaitForever);
    if (osOK == eStatus)
    {
      CLis3mdlDriver::GetInstance().PollSensor(suTimestamp);
    }
  }
}

extern "C" void RunTaskBmm350(void* argument)
{
  for(;;)
  {
    static uint64_t suTimestamp = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskBmm350TimestampHandle, &suTimestamp, 0U, osWaitForever);
    if (osOK == eStatus)
    {
      CBmm350Driver::GetInstance().PollSensor(suTimestamp);
    }
  }
}

extern "C" void RunTaskCanSender(void* argument)
{
  for(;;)
  {
    osDelay(osWaitForever);
  }
}

extern "C" void RunTaskCanReceiver(void* argument)
{
  for (;;)
  {
    osDelay(osWaitForever);
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
