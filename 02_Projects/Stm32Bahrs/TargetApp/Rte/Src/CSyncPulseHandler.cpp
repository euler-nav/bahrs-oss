/**
 * @file CSyncPulseHandler.cpp
 * @brief Implementation of the CSyncPulseHandler software component.
 * @author Fedor Baklanov
 * @date 31 May 2023
 */

#include "cmsis_os2.h"
#include "CRte.h"
#include "GetMicroseconds.h"

extern osMessageQueueId_t QueueTaskProcessSyncHandle;

extern "C" void SyncPulseCallback()
{
  CSyncPulseHandler::SyncPulseCallback();
}

void CSyncPulseHandler::Init()
{
  // do nothing
}

bool CSyncPulseHandler::IsInitialized()
{
  return true;
}

CSyncPulseHandler& CSyncPulseHandler::getInstanceImpl(unsigned uInstanceIndex)
{
  // This function is required by the GetInstance() of the parent class.
  // However, this function shall never be used, because instance count is 0.
  static CSyncPulseHandler soInstance;
  assert(false);
  return soInstance;
}

void CSyncPulseHandler::SyncPulseCallback()
{
  static uint64_t suLastCallTimeUs = 0U;
  uint64_t uCurrentTimeUs = GetMicroseconds();

  // We ignore pulses if their rate is too high
  if ((uCurrentTimeUs - suLastCallTimeUs) >= 90000U)
  {
    osStatus_t oStatus = osMessageQueuePut(QueueTaskProcessSyncHandle, &uCurrentTimeUs, 0U, 0U);

    if (osErrorResource == oStatus)
    {
      // No space in the queue, so we need to replace data with the latest timestamp.
      // Note that we cannot use the function osMessageQueueReset() from an interrupt.
      uint64_t uTimestampTmp;
      oStatus = osMessageQueueGet(QueueTaskProcessSyncHandle, &uTimestampTmp, 0U, 0U);

      if (osOK == oStatus)
      {
        osMessageQueuePut(QueueTaskProcessSyncHandle, &uCurrentTimeUs, 0U, 0U);
      }
    }
  }

  suLastCallTimeUs = uCurrentTimeUs;
}

void CSyncPulseHandler::ProcessSyncPulse(uint64_t uTimestampUs)
{
  CRte::GetInstance().oPortSyncPulseTime_.Write(uTimestampUs);
  CRs232OutputHandler::GetInstance().QueueTransmissionRequest(CSerialProtocol::EMessageIds::eTimeOfSyncPulse);
}

