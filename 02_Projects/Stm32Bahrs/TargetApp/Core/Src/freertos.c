/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "RteWrapper.h"
#include "TaskRoutines.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static const uint32_t skuPeriod1msInTicks = (uint32_t) (0.001F * (float)configTICK_RATE_HZ);
static const uint32_t skuPeriod5msInTicks = (uint32_t) (0.005F * (float)configTICK_RATE_HZ);
static const uint32_t skuPeriod10msInTicks = (uint32_t) (0.01F * (float)configTICK_RATE_HZ);

/* USER CODE END Variables */
/* Definitions for Task5ms */
osThreadId_t Task5msHandle;
uint32_t Task5msBuffer[ 2056 ];
osStaticThreadDef_t Task5msControlBlock;
const osThreadAttr_t Task5ms_attributes = {
  .name = "Task5ms",
  .cb_mem = &Task5msControlBlock,
  .cb_size = sizeof(Task5msControlBlock),
  .stack_mem = &Task5msBuffer[0],
  .stack_size = sizeof(Task5msBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task10ms */
osThreadId_t Task10msHandle;
uint32_t Task10msBuffer[ 4096 ];
osStaticThreadDef_t Task10msControlBlock;
const osThreadAttr_t Task10ms_attributes = {
  .name = "Task10ms",
  .cb_mem = &Task10msControlBlock,
  .cb_size = sizeof(Task10msControlBlock),
  .stack_mem = &Task10msBuffer[0],
  .stack_size = sizeof(Task10msBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskRs232Sender */
osThreadId_t TaskRs232SenderHandle;
uint32_t TaskRs232SenderBuffer[ 256 ];
osStaticThreadDef_t TaskRs232SenderControlBlock;
const osThreadAttr_t TaskRs232Sender_attributes = {
  .name = "TaskRs232Sender",
  .cb_mem = &TaskRs232SenderControlBlock,
  .cb_size = sizeof(TaskRs232SenderControlBlock),
  .stack_mem = &TaskRs232SenderBuffer[0],
  .stack_size = sizeof(TaskRs232SenderBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TaskProcessSync */
osThreadId_t TaskProcessSyncHandle;
uint32_t TaskProcessSyncBuffer[ 128 ];
osStaticThreadDef_t TaskProcessSyncControlBlock;
const osThreadAttr_t TaskProcessSync_attributes = {
  .name = "TaskProcessSync",
  .cb_mem = &TaskProcessSyncControlBlock,
  .cb_size = sizeof(TaskProcessSyncControlBlock),
  .stack_mem = &TaskProcessSyncBuffer[0],
  .stack_size = sizeof(TaskProcessSyncBuffer),
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for QueueTaskRs232Sender */
osMessageQueueId_t QueueTaskRs232SenderHandle;
uint8_t QueueTaskRs232SenderBuffer[ 8 * sizeof( uint8_t ) ];
osStaticMessageQDef_t QueueTaskRs232SenderControlBlock;
const osMessageQueueAttr_t QueueTaskRs232Sender_attributes = {
  .name = "QueueTaskRs232Sender",
  .cb_mem = &QueueTaskRs232SenderControlBlock,
  .cb_size = sizeof(QueueTaskRs232SenderControlBlock),
  .mq_mem = &QueueTaskRs232SenderBuffer,
  .mq_size = sizeof(QueueTaskRs232SenderBuffer)
};
/* Definitions for QueueTaskProcessSync */
osMessageQueueId_t QueueTaskProcessSyncHandle;
uint8_t QueueTaskProcessSyncBuffer[ 1 * sizeof( uint64_t ) ];
osStaticMessageQDef_t QueueTaskProcessSyncControlBlock;
const osMessageQueueAttr_t QueueTaskProcessSync_attributes = {
  .name = "QueueTaskProcessSync",
  .cb_mem = &QueueTaskProcessSyncControlBlock,
  .cb_size = sizeof(QueueTaskProcessSyncControlBlock),
  .mq_mem = &QueueTaskProcessSyncBuffer,
  .mq_size = sizeof(QueueTaskProcessSyncBuffer)
};
/* Definitions for TimerCyclicTaskTrigger */
osTimerId_t TimerCyclicTaskTriggerHandle;
osStaticTimerDef_t TimerCyclicTaskTriggerControlBlock;
const osTimerAttr_t TimerCyclicTaskTrigger_attributes = {
  .name = "TimerCyclicTaskTrigger",
  .cb_mem = &TimerCyclicTaskTriggerControlBlock,
  .cb_size = sizeof(TimerCyclicTaskTriggerControlBlock),
};
/* Definitions for SemaphoreTask5ms */
osSemaphoreId_t SemaphoreTask5msHandle;
osStaticSemaphoreDef_t SemaphoreTask5msControlBlock;
const osSemaphoreAttr_t SemaphoreTask5ms_attributes = {
  .name = "SemaphoreTask5ms",
  .cb_mem = &SemaphoreTask5msControlBlock,
  .cb_size = sizeof(SemaphoreTask5msControlBlock),
};
/* Definitions for SemaphoreTask10ms */
osSemaphoreId_t SemaphoreTask10msHandle;
osStaticSemaphoreDef_t SemaphoreTask10msControlBlock;
const osSemaphoreAttr_t SemaphoreTask10ms_attributes = {
  .name = "SemaphoreTask10ms",
  .cb_mem = &SemaphoreTask10msControlBlock,
  .cb_size = sizeof(SemaphoreTask10msControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask5ms(void *argument);
void StartTask10ms(void *argument);
void StartTaskRs232Sender(void *argument);
void StartTaskProcessSyncPulse(void *argument);
void TimerCyclicTaskTriggerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreTask5ms */
  SemaphoreTask5msHandle = osSemaphoreNew(1, 0, &SemaphoreTask5ms_attributes);

  /* creation of SemaphoreTask10ms */
  SemaphoreTask10msHandle = osSemaphoreNew(1, 0, &SemaphoreTask10ms_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerCyclicTaskTrigger */
  TimerCyclicTaskTriggerHandle = osTimerNew(TimerCyclicTaskTriggerCallback, osTimerPeriodic, NULL, &TimerCyclicTaskTrigger_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  osTimerStart(TimerCyclicTaskTriggerHandle, 1U);
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueTaskRs232Sender */
  QueueTaskRs232SenderHandle = osMessageQueueNew (8, sizeof(uint8_t), &QueueTaskRs232Sender_attributes);

  /* creation of QueueTaskProcessSync */
  QueueTaskProcessSyncHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskProcessSync_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task5ms */
  Task5msHandle = osThreadNew(StartTask5ms, NULL, &Task5ms_attributes);

  /* creation of Task10ms */
  Task10msHandle = osThreadNew(StartTask10ms, NULL, &Task10ms_attributes);

  /* creation of TaskRs232Sender */
  TaskRs232SenderHandle = osThreadNew(StartTaskRs232Sender, NULL, &TaskRs232Sender_attributes);

  /* creation of TaskProcessSync */
  TaskProcessSyncHandle = osThreadNew(StartTaskProcessSyncPulse, NULL, &TaskProcessSync_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

  RteInit();
  InitializeSensors();

  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask5ms */
/**
  * @brief  Function implementing the Task5ms thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask5ms */
void StartTask5ms(void *argument)
{
  /* USER CODE BEGIN StartTask5ms */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemaphoreTask5msHandle, osWaitForever);
    TaskRoutine5ms();
  }
  /* USER CODE END StartTask5ms */
}

/* USER CODE BEGIN Header_StartTask10ms */
/**
* @brief Function implementing the Task10ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask10ms */
void StartTask10ms(void *argument)
{
  /* USER CODE BEGIN StartTask10ms */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemaphoreTask10msHandle, osWaitForever);
    TaskRoutine10ms();
  }
  /* USER CODE END StartTask10ms */
}

/* USER CODE BEGIN Header_StartTaskRs232Sender */
/**
* @brief Function implementing the TaskRs232Sender thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskRs232Sender */
void StartTaskRs232Sender(void *argument)
{
  /* USER CODE BEGIN StartTaskRs232Sender */
  /* Infinite loop */
  for(;;)
  {
    uint8_t uMessageId = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskRs232SenderHandle, &uMessageId, NULL, osWaitForever);

    if (osOK == eStatus)
    {
      TaskRoutineRs232Sender(uMessageId);
    }
  }
  /* USER CODE END StartTaskRs232Sender */
}

/* USER CODE BEGIN Header_StartTaskProcessSyncPulse */
/**
* @brief Function implementing the TaskProcessSync thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskProcessSyncPulse */
void StartTaskProcessSyncPulse(void *argument)
{
  /* USER CODE BEGIN StartTaskProcessSyncPulse */
  /* Infinite loop */
  for(;;)
  {
    uint64_t uTimestampUs = 0U;
    osStatus_t eStatus = osMessageQueueGet(QueueTaskProcessSyncHandle, &uTimestampUs, NULL, osWaitForever);

    if (osOK == eStatus)
    {
      TaskRoutineProcessSyncPulse(uTimestampUs);
    }
  }
  /* USER CODE END StartTaskProcessSyncPulse */
}

/* TimerCyclicTaskTriggerCallback function */
void TimerCyclicTaskTriggerCallback(void *argument)
{
  /* USER CODE BEGIN TimerCyclicTaskTriggerCallback */

  static uint32_t suCounter = 0U;

  // Trigger 5ms tasks
  if ( 0U == (suCounter % skuPeriod5msInTicks) )
  {
    osSemaphoreRelease(SemaphoreTask5msHandle);
  }

  // Trigger 10ms tasks
  if ( 0U == (suCounter % skuPeriod10msInTicks) )
  {
    osSemaphoreRelease(SemaphoreTask10msHandle);
  }

  ++suCounter;

  /* USER CODE END TimerCyclicTaskTriggerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

