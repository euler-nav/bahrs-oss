/// @file TaskRoutines.h
/// @brief Declarations of FreeRTOS task routines that wrap C++ code into C-functions.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef TASK_ROUTINES_H
#define TASK_ROUTINES_H

#ifdef __cplusplus
extern "C"
{
#endif

void TaskRoutine1ms();

void TaskRoutine5ms();

void TaskRoutine10ms();

void TaskRoutineRs232Sender(uint8_t uMessageId);

#ifdef SEND_DEBUG_OUTPUT
void TaskRoutineRs232SenderDebugOutput();
unsigned TaskRoutineRs232SenderGetDebugMessageQueueSize();
#endif // SEND_DEBUG_OUTPUT

void TaskRoutineCanSender();

void TaskRoutineCanReceiver();

void TaskRoutineInit();

void TaskRoutineProcessSyncPulse(uint64_t uTimestampUs);

void TaskRoutineReceiveScha63TData();

void TaskRoutinePollBmm(uint32_t uBmmSensorIndex);

void TaskRoutinePollBmp();

void TaskRoutinePollMmc();

void TaskRoutineBahrsFilter(uint32_t uFilterIndex);

#ifdef __cplusplus
}
#endif

#endif // TASK_ROUTINES_H
