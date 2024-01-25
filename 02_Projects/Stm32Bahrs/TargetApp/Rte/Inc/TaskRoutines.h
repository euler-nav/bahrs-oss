/**
 * @file TaskRoutines.h
 * @brief Declarations of FreeRTOS task routines that wrap C++ code into C-functions.
 * @author Fedor Baklanov
 * @date 05 February 2022
 */

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

void InitializeSensors();

void TaskRoutineProcessSyncPulse(uint64_t uTimestampUs);

#ifdef __cplusplus
}
#endif

#endif /* TASK_ROUTINES_H */
