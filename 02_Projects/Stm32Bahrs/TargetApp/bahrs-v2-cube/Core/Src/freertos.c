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
#include "CDiagnosticModeWrapper.h"
#include "Icm20789DriverCApi.h"
#include "BahrsFilterSwcCApi.h"
#include "PressureCompensatorSwcCApi.h"
#include "ImuPreFilterSwcCApi.h"
#include "GetMicroseconds.h"
#include "assert.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
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
static const uint32_t skuPeriod100msInTicks = (uint32_t) (0.1F * (float)configTICK_RATE_HZ);
static const uint32_t skuPeriod40msInTicks = (uint32_t) (0.04F * (float)configTICK_RATE_HZ);
static const uint32_t skuPeriod10sInTicks = (uint32_t) (10.0F * (float)configTICK_RATE_HZ);

#ifdef MEASURE_RESOURCES_USAGE

#define TASK_COUNT_FOR_DIAGNOSTICS (32U)
#define MIN_FREE_STACK_SPACE_FOR_ASSERT (4U)

typedef struct
{
  char acTaskName_[configMAX_TASK_NAME_LEN];
  uint32_t uFreeStackSpace_;
} STaskStackUsageData;

static TaskStatus_t aoTaskStatusArray[TASK_COUNT_FOR_DIAGNOSTICS];
static volatile STaskStackUsageData saoStackUsageData[TASK_COUNT_FOR_DIAGNOSTICS];

#endif /* MEASURE_RESOURCES_USAGE */

/* USER CODE END Variables */
/* Definitions for Task1ms */
osThreadId_t Task1msHandle;
uint32_t Task1msBuffer[ 128 ];
osStaticThreadDef_t Task1msControlBlock;
const osThreadAttr_t Task1ms_attributes = {
  .name = "Task1ms",
  .cb_mem = &Task1msControlBlock,
  .cb_size = sizeof(Task1msControlBlock),
  .stack_mem = &Task1msBuffer[0],
  .stack_size = sizeof(Task1msBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task5ms */
osThreadId_t Task5msHandle;
uint32_t Task5msBuffer[ 512 ];
osStaticThreadDef_t Task5msControlBlock;
const osThreadAttr_t Task5ms_attributes = {
  .name = "Task5ms",
  .cb_mem = &Task5msControlBlock,
  .cb_size = sizeof(Task5msControlBlock),
  .stack_mem = &Task5msBuffer[0],
  .stack_size = sizeof(Task5msBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for Task10ms */
osThreadId_t Task10msHandle;
uint32_t Task10msBuffer[ 1024 ];
osStaticThreadDef_t Task10msControlBlock;
const osThreadAttr_t Task10ms_attributes = {
  .name = "Task10ms",
  .cb_mem = &Task10msControlBlock,
  .cb_size = sizeof(Task10msControlBlock),
  .stack_mem = &Task10msBuffer[0],
  .stack_size = sizeof(Task10msBuffer),
  .priority = (osPriority_t) osPriorityNormal,
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
/* Definitions for TaskCanSender */
osThreadId_t TaskCanSenderHandle;
uint32_t TaskCanSenderBuffer[ 256 ];
osStaticThreadDef_t TaskCanSenderControlBlock;
const osThreadAttr_t TaskCanSender_attributes = {
  .name = "TaskCanSender",
  .cb_mem = &TaskCanSenderControlBlock,
  .cb_size = sizeof(TaskCanSenderControlBlock),
  .stack_mem = &TaskCanSenderBuffer[0],
  .stack_size = sizeof(TaskCanSenderBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for TaskCanReceiver */
osThreadId_t TaskCanReceiverHandle;
uint32_t TaskCanReceiverBuffer[ 256 ];
osStaticThreadDef_t TaskCanReceiverControlBlock;
const osThreadAttr_t TaskCanReceiver_attributes = {
  .name = "TaskCanReceiver",
  .cb_mem = &TaskCanReceiverControlBlock,
  .cb_size = sizeof(TaskCanReceiverControlBlock),
  .stack_mem = &TaskCanReceiverBuffer[0],
  .stack_size = sizeof(TaskCanReceiverBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskPollIcm1 */
osThreadId_t TaskPollIcm1Handle;
uint32_t TaskPollIcmBuffer1[ 256 ];
osStaticThreadDef_t TaskPollIcmControlBlock1;
const osThreadAttr_t TaskPollIcm1_attributes = {
  .name = "TaskPollIcm1",
  .cb_mem = &TaskPollIcmControlBlock1,
  .cb_size = sizeof(TaskPollIcmControlBlock1),
  .stack_mem = &TaskPollIcmBuffer1[0],
  .stack_size = sizeof(TaskPollIcmBuffer1),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskPollIcm2 */
osThreadId_t TaskPollIcm2Handle;
uint32_t TaskPollIcmBuffer2[ 256 ];
osStaticThreadDef_t TaskPollIcmControlBlock2;
const osThreadAttr_t TaskPollIcm2_attributes = {
  .name = "TaskPollIcm2",
  .cb_mem = &TaskPollIcmControlBlock2,
  .cb_size = sizeof(TaskPollIcmControlBlock2),
  .stack_mem = &TaskPollIcmBuffer2[0],
  .stack_size = sizeof(TaskPollIcmBuffer2),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskReceiveScha63TData */
osThreadId_t TaskReceiveScha63TDataHandle;
uint32_t TaskReceiveScha63TDataBuffer[ 512 ];
osStaticThreadDef_t TaskReceiveScha63TDataControlBlock;
const osThreadAttr_t TaskReceiveScha63TData_attributes = {
  .name = "TaskReceiveScha63TData",
  .cb_mem = &TaskReceiveScha63TDataControlBlock,
  .cb_size = sizeof(TaskReceiveScha63TDataControlBlock),
  .stack_mem = &TaskReceiveScha63TDataBuffer[0],
  .stack_size = sizeof(TaskReceiveScha63TDataBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskPollBmm1 */
osThreadId_t TaskPollBmm1Handle;
uint32_t TaskPollBmm1Buffer[ 256 ];
osStaticThreadDef_t TaskPollBmm1ControlBlock;
const osThreadAttr_t TaskPollBmm1_attributes = {
  .name = "TaskPollBmm1",
  .cb_mem = &TaskPollBmm1ControlBlock,
  .cb_size = sizeof(TaskPollBmm1ControlBlock),
  .stack_mem = &TaskPollBmm1Buffer[0],
  .stack_size = sizeof(TaskPollBmm1Buffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskPollBmm2 */
osThreadId_t TaskPollBmm2Handle;
uint32_t TaskPollBmm2Buffer[ 256 ];
osStaticThreadDef_t TaskPollBmm2ControlBlock;
const osThreadAttr_t TaskPollBmm2_attributes = {
  .name = "TaskPollBmm2",
  .cb_mem = &TaskPollBmm2ControlBlock,
  .cb_size = sizeof(TaskPollBmm2ControlBlock),
  .stack_mem = &TaskPollBmm2Buffer[0],
  .stack_size = sizeof(TaskPollBmm2Buffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskPollBmp */
osThreadId_t TaskPollBmpHandle;
uint32_t TaskPollBmpBuffer[ 256 ];
osStaticThreadDef_t TaskPollBmpControlBlock;
const osThreadAttr_t TaskPollBmp_attributes = {
  .name = "TaskPollBmp",
  .cb_mem = &TaskPollBmpControlBlock,
  .cb_size = sizeof(TaskPollBmpControlBlock),
  .stack_mem = &TaskPollBmpBuffer[0],
  .stack_size = sizeof(TaskPollBmpBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskPollMmc */
osThreadId_t TaskPollMmcHandle;
uint32_t TaskPollMmcBuffer[ 256 ];
osStaticThreadDef_t TaskPollMmcControlBlock;
const osThreadAttr_t TaskPollMmc_attributes = {
  .name = "TaskPollMmc",
  .cb_mem = &TaskPollMmcControlBlock,
  .cb_size = sizeof(TaskPollMmcControlBlock),
  .stack_mem = &TaskPollMmcBuffer[0],
  .stack_size = sizeof(TaskPollMmcBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskInit */
osThreadId_t TaskInitHandle;
uint32_t TaskInitBuffer[ 256 ];
osStaticThreadDef_t TaskInitControlBlock;
const osThreadAttr_t TaskInit_attributes = {
  .name = "TaskInit",
  .cb_mem = &TaskInitControlBlock,
  .cb_size = sizeof(TaskInitControlBlock),
  .stack_mem = &TaskInitBuffer[0],
  .stack_size = sizeof(TaskInitBuffer),
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for TaskMeasureResourcesUsage */
osThreadId_t TaskMeasureResourcesUsageHandle;
uint32_t TaskMeasureResourcesUsageBuffer[ 256 ];
osStaticThreadDef_t TaskMeasureResourcesUsagerControlBlock;
const osThreadAttr_t TaskMeasureResourcesUsage_attributes = {
  .name = "TaskMeasureResourcesUsage",
  .cb_mem = &TaskMeasureResourcesUsagerControlBlock,
  .cb_size = sizeof(TaskMeasureResourcesUsagerControlBlock),
  .stack_mem = &TaskMeasureResourcesUsageBuffer[0],
  .stack_size = sizeof(TaskMeasureResourcesUsageBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TaskBahrsFilter1 */
osThreadId_t TaskBahrsFilter1Handle;
uint32_t TaskBahrsFilter1Buffer[ 2048 ];
osStaticThreadDef_t TaskBahrsFilter1ControlBlock;
const osThreadAttr_t TaskBahrsFilter1_attributes = {
  .name = "TaskBahrsFilter1",
  .cb_mem = &TaskBahrsFilter1ControlBlock,
  .cb_size = sizeof(TaskBahrsFilter1ControlBlock),
  .stack_mem = &TaskBahrsFilter1Buffer[0],
  .stack_size = sizeof(TaskBahrsFilter1Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskBahrsFilter2 */
osThreadId_t TaskBahrsFilter2Handle;
uint32_t TaskBahrsFilter2Buffer[ 2048 ];
osStaticThreadDef_t TaskBahrsFilter2ControlBlock;
const osThreadAttr_t TaskBahrsFilter2_attributes = {
  .name = "TaskBahrsFilter2",
  .cb_mem = &TaskBahrsFilter2ControlBlock,
  .cb_size = sizeof(TaskBahrsFilter2ControlBlock),
  .stack_mem = &TaskBahrsFilter2Buffer[0],
  .stack_size = sizeof(TaskBahrsFilter2Buffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskBahrsFilter3 */
osThreadId_t TaskBahrsFilter3Handle;
uint32_t TaskBahrsFilter3Buffer[ 2048 ];
osStaticThreadDef_t TaskBahrsFilter3ControlBlock;
const osThreadAttr_t TaskBahrsFilter3_attributes = {
  .name = "TaskBahrsFilter3",
  .cb_mem = &TaskBahrsFilter3ControlBlock,
  .cb_size = sizeof(TaskBahrsFilter3ControlBlock),
  .stack_mem = &TaskBahrsFilter3Buffer[0],
  .stack_size = sizeof(TaskBahrsFilter3Buffer),
  .priority = (osPriority_t) osPriorityNormal,
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
/* Definitions for SemaphoreTask1ms */
osSemaphoreId_t SemaphoreTask1msHandle;
const osSemaphoreAttr_t SemaphoreTask1ms_attributes = {
  .name = "SemaphoreTask1ms"
};
/* Definitions for SemTask5ms */
osSemaphoreId_t SemTask5msHandle;
const osSemaphoreAttr_t SemTask5ms_attributes = {
  .name = "SemTask5ms"
};
/* Definitions for SemaphoreTaskCanSender */
osSemaphoreId_t SemaphoreTaskCanSenderHandle;
const osSemaphoreAttr_t SemaphoreTaskCanSender_attributes = {
  .name = "SemaphoreTaskCanSender"
};
/* Definitions for SemaphoreTdk1 */
osSemaphoreId_t SemaphoreTdk1Handle;
const osSemaphoreAttr_t SemaphoreTdk1_attributes = {
  .name = "SemaphoreTdk1"
};
/* Definitions for SemaphoreTdk2 */
osSemaphoreId_t SemaphoreTdk2Handle;
const osSemaphoreAttr_t SemaphoreTdk2_attributes = {
  .name = "SemaphoreTdk2"
};
/* Definitions for SemDataReceivedTdk1 */
osSemaphoreId_t SemDataReceivedTdk1Handle;
const osSemaphoreAttr_t SemDataReceivedTdk1_attributes = {
  .name = "SemDataReceivedTdk1"
};
/* Definitions for SemDataReceivedTdk2 */
osSemaphoreId_t SemDataReceivedTdk2Handle;
const osSemaphoreAttr_t SemDataReceivedTdk2_attributes = {
  .name = "SemDataReceivedTdk2"
};
/* Definitions for SemTaskReceiveScha63TData */
osSemaphoreId_t SemTaskReceiveScha63TDataHandle;
const osSemaphoreAttr_t SemTaskReceiveScha63TData_attributes = {
  .name = "SemTaskReceiveScha63TData"
};
/* Definitions for SemTaskPollBmm1 */
osSemaphoreId_t SemTaskPollBmm1Handle;
const osSemaphoreAttr_t SemTaskPollBmm1_attributes = {
  .name = "SemTaskPollBmm1"
};
/* Definitions for SemTaskPollBmm2 */
osSemaphoreId_t SemTaskPollBmm2Handle;
const osSemaphoreAttr_t SemTaskPollBmm2_attributes = {
  .name = "SemTaskPollBmm2"
};
/* Definitions for SemTaskPollBmp */
osSemaphoreId_t SemTaskPollBmpHandle;
const osSemaphoreAttr_t SemTaskPollBmp_attributes = {
  .name = "SemTaskPollBmp"
};
/* Definitions for SemTaskPollMmc */
osSemaphoreId_t SemTaskPollMmcHandle;
const osSemaphoreAttr_t SemTaskPollMmc_attributes = {
  .name = "SemTaskPollMmc"
};
/* Definitions for SemTaskMeasureResourcesUsage */
osSemaphoreId_t SemTaskMeasureResourcesUsageHandle;
const osSemaphoreAttr_t SemTaskMeasureResourcesUsage_attributes = {
  .name = "SemTaskMeasureResourcesUsage"
};
/* Definitions for SemTaskBahrsFilter1 */
osSemaphoreId_t SemTaskBahrsFilter1Handle;
const osSemaphoreAttr_t SemTaskBahrsFilter1_attributes = {
  .name = "SemTaskBahrsFilter1"
};
/* Definitions for SemTaskBahrsFilter2 */
osSemaphoreId_t SemTaskBahrsFilter2Handle;
const osSemaphoreAttr_t SemTaskBahrsFilter2_attributes = {
  .name = "SemTaskBahrsFilter2"
};
/* Definitions for SemTaskBahrsFilter3 */
osSemaphoreId_t SemTaskBahrsFilter3Handle;
const osSemaphoreAttr_t SemTaskBahrsFilter3_attributes = {
  .name = "SemTaskBahrsFilter3"
};
/* Definitions for EventBahrsFiltersCompleted */
osEventFlagsId_t EventBahrsFiltersCompletedHandle;
osStaticEventGroupDef_t EventBahrsFiltersCompletedControlBlock;
const osEventFlagsAttr_t EventBahrsFiltersCompleted_attributes = {
  .name = "EventBahrsFiltersCompleted",
  .cb_mem = &EventBahrsFiltersCompletedControlBlock,
  .cb_size = sizeof(EventBahrsFiltersCompletedControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask1ms(void *argument);
extern void RunTask5ms(void *argument);
extern void RunTask10ms(void *argument);
void StartTaskProcessSyncPulse(void *argument);
void StartTaskCanSender(void *argument);
void StartTaskCanReceiver(void *argument);
void StartTaskPollIcm1(void *argument);
void StartTaskPollIcm2(void *argument);
void StartTaskReceiveScha63TData(void *argument);
void StartTaskPollBmm1(void *argument);
void StartTaskPollBmm2(void *argument);
void StartTaskPollBmp(void *argument);
void StartTaskTaskPollMmc(void *argument);
void StartTaskInit(void *argument);
void StartTaskMeasureResourcesUsage(void *argument);
extern void RunTaskBahrsFilter1(void *argument);
extern void RunTaskBahrsFilter2(void *argument);
extern void RunTaskBahrsFilter3(void *argument);
void TimerCyclicTaskTriggerCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
  return GetMicroseconds();
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  assert(0);
}
/* USER CODE END 4 */

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
  /* creation of SemaphoreTask1ms */
  SemaphoreTask1msHandle = osSemaphoreNew(1, 0, &SemaphoreTask1ms_attributes);

  /* creation of SemTask5ms */
  SemTask5msHandle = osSemaphoreNew(1, 0, &SemTask5ms_attributes);

  /* creation of SemaphoreTaskCanSender */
  SemaphoreTaskCanSenderHandle = osSemaphoreNew(1, 0, &SemaphoreTaskCanSender_attributes);

  /* creation of SemaphoreTdk1 */
  SemaphoreTdk1Handle = osSemaphoreNew(1, 0, &SemaphoreTdk1_attributes);

  /* creation of SemaphoreTdk2 */
  SemaphoreTdk2Handle = osSemaphoreNew(1, 0, &SemaphoreTdk2_attributes);

  /* creation of SemDataReceivedTdk1 */
  SemDataReceivedTdk1Handle = osSemaphoreNew(1, 0, &SemDataReceivedTdk1_attributes);

  /* creation of SemDataReceivedTdk2 */
  SemDataReceivedTdk2Handle = osSemaphoreNew(1, 0, &SemDataReceivedTdk2_attributes);

  /* creation of SemTaskReceiveScha63TData */
  SemTaskReceiveScha63TDataHandle = osSemaphoreNew(1, 0, &SemTaskReceiveScha63TData_attributes);

  /* creation of SemTaskPollBmm1 */
  SemTaskPollBmm1Handle = osSemaphoreNew(1, 0, &SemTaskPollBmm1_attributes);

  /* creation of SemTaskPollBmm2 */
  SemTaskPollBmm2Handle = osSemaphoreNew(1, 0, &SemTaskPollBmm2_attributes);

  /* creation of SemTaskPollBmp */
  SemTaskPollBmpHandle = osSemaphoreNew(1, 0, &SemTaskPollBmp_attributes);

  /* creation of SemTaskPollMmc */
  SemTaskPollMmcHandle = osSemaphoreNew(1, 0, &SemTaskPollMmc_attributes);

  /* creation of SemTaskMeasureResourcesUsage */
  SemTaskMeasureResourcesUsageHandle = osSemaphoreNew(1, 0, &SemTaskMeasureResourcesUsage_attributes);

  /* creation of SemTaskBahrsFilter1 */
  SemTaskBahrsFilter1Handle = osSemaphoreNew(1, 0, &SemTaskBahrsFilter1_attributes);

  /* creation of SemTaskBahrsFilter2 */
  SemTaskBahrsFilter2Handle = osSemaphoreNew(1, 0, &SemTaskBahrsFilter2_attributes);

  /* creation of SemTaskBahrsFilter3 */
  SemTaskBahrsFilter3Handle = osSemaphoreNew(1, 0, &SemTaskBahrsFilter3_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerCyclicTaskTrigger */
  TimerCyclicTaskTriggerHandle = osTimerNew(TimerCyclicTaskTriggerCallback, osTimerPeriodic, NULL, &TimerCyclicTaskTrigger_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueTaskProcessSync */
  QueueTaskProcessSyncHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskProcessSync_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task1ms */
  Task1msHandle = osThreadNew(StartTask1ms, NULL, &Task1ms_attributes);

  /* creation of Task5ms */
  Task5msHandle = osThreadNew(RunTask5ms, NULL, &Task5ms_attributes);

  /* creation of Task10ms */
  Task10msHandle = osThreadNew(RunTask10ms, NULL, &Task10ms_attributes);

  /* creation of TaskProcessSync */
  TaskProcessSyncHandle = osThreadNew(StartTaskProcessSyncPulse, NULL, &TaskProcessSync_attributes);

  /* creation of TaskCanSender */
  TaskCanSenderHandle = osThreadNew(StartTaskCanSender, NULL, &TaskCanSender_attributes);

  /* creation of TaskCanReceiver */
  TaskCanReceiverHandle = osThreadNew(StartTaskCanReceiver, NULL, &TaskCanReceiver_attributes);

  /* creation of TaskPollIcm1 */
  TaskPollIcm1Handle = osThreadNew(StartTaskPollIcm1, NULL, &TaskPollIcm1_attributes);

  /* creation of TaskPollIcm2 */
  TaskPollIcm2Handle = osThreadNew(StartTaskPollIcm2, NULL, &TaskPollIcm2_attributes);

  /* creation of TaskReceiveScha63TData */
  TaskReceiveScha63TDataHandle = osThreadNew(StartTaskReceiveScha63TData, NULL, &TaskReceiveScha63TData_attributes);

  /* creation of TaskPollBmm1 */
  TaskPollBmm1Handle = osThreadNew(StartTaskPollBmm1, NULL, &TaskPollBmm1_attributes);

  /* creation of TaskPollBmm2 */
  TaskPollBmm2Handle = osThreadNew(StartTaskPollBmm2, NULL, &TaskPollBmm2_attributes);

  /* creation of TaskPollBmp */
  TaskPollBmpHandle = osThreadNew(StartTaskPollBmp, NULL, &TaskPollBmp_attributes);

  /* creation of TaskPollMmc */
  TaskPollMmcHandle = osThreadNew(StartTaskTaskPollMmc, NULL, &TaskPollMmc_attributes);

  /* creation of TaskInit */
  TaskInitHandle = osThreadNew(StartTaskInit, NULL, &TaskInit_attributes);

  /* creation of TaskMeasureResourcesUsage */
  TaskMeasureResourcesUsageHandle = osThreadNew(StartTaskMeasureResourcesUsage, NULL, &TaskMeasureResourcesUsage_attributes);

  /* creation of TaskBahrsFilter1 */
  TaskBahrsFilter1Handle = osThreadNew(RunTaskBahrsFilter1, NULL, &TaskBahrsFilter1_attributes);

  /* creation of TaskBahrsFilter2 */
  TaskBahrsFilter2Handle = osThreadNew(RunTaskBahrsFilter2, NULL, &TaskBahrsFilter2_attributes);

  /* creation of TaskBahrsFilter3 */
  TaskBahrsFilter3Handle = osThreadNew(RunTaskBahrsFilter3, NULL, &TaskBahrsFilter3_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of EventBahrsFiltersCompleted */
  EventBahrsFiltersCompletedHandle = osEventFlagsNew(&EventBahrsFiltersCompleted_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask1ms */
/**
* @brief Function implementing the Task1ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask1ms */
void StartTask1ms(void *argument)
{
  /* USER CODE BEGIN StartTask1ms */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemaphoreTask1msHandle, osWaitForever);
    TaskRoutine1ms();
  }
  /* USER CODE END StartTask1ms */
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

/* USER CODE BEGIN Header_StartTaskCanSender */
/**
* @brief Function implementing the TaskCanSender thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCanSender */
void StartTaskCanSender(void *argument)
{
  /* USER CODE BEGIN StartTaskCanSender */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemaphoreTaskCanSenderHandle, osWaitForever);
    TaskRoutineCanSender();
  }
  /* USER CODE END StartTaskCanSender */
}

/* USER CODE BEGIN Header_StartTaskCanReceiver */
/**
* @brief Function implementing the TaskCanReceiver thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCanReceiver */
void StartTaskCanReceiver(void *argument)
{
  /* USER CODE BEGIN StartTaskCanReceiver */
  /* Infinite loop */
  for(;;)
  {
    TaskRoutineCanReceiver();
  }
  /* USER CODE END StartTaskCanReceiver */
}

/* USER CODE BEGIN Header_StartTaskPollIcm1 */
/**
* @brief Function implementing the TaskPollIcm1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPollIcm1 */
void StartTaskPollIcm1(void *argument)
{
  /* USER CODE BEGIN StartTaskPollIcm1 */
  static const uint32_t skuSensorInstance = 0U;
  static uint32_t uCounter = 0U;
  static const uint32_t skuPressurePollingPeriod = 8U;
  uint8_t uIsWaitingForPressure = 0U;
  uint8_t uWaitingForPressureCounter = 0U;

  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemaphoreTdk1Handle, osWaitForever);

    if (0 == Icm20789ImuDataDmaRequest(skuSensorInstance))
    {
      osStatus_t eStatus = osSemaphoreAcquire(SemDataReceivedTdk1Handle, skuPeriod5msInTicks);

      if (osOK == eStatus)
      {
        Icm20789ParseImuDataDma(skuSensorInstance);
      }
      else
      {
        Icm20789InvalidateImuOutputPort(skuSensorInstance);
      }
    }
    else
    {
      Icm20789InvalidateImuOutputPort(skuSensorInstance);
    }

    ImuPreFilterSwcRun(1U);
    BahrsFilterSwcSetImuInput(1U);

    if ((uCounter % 2) == 0)
    {
      osSemaphoreRelease(SemTaskBahrsFilter2Handle);
    }

    // Trigger pressure measurement at 25Hz
    if (0U == (uCounter % skuPressurePollingPeriod))
    {
      if (0 == uIsWaitingForPressure)
      {
        if(0 == Icm20789TriggerPressureMeasurement(skuSensorInstance))
        {
          uIsWaitingForPressure = 1U;
          uWaitingForPressureCounter = 0U;
        }
        else
        {
          Icm20789InvalidatePressureOutputPort(skuSensorInstance);
          PressureCompensatorSwcCompensate(1U);
        }
      }
    }

    if (1 == uIsWaitingForPressure)
    {
      // We wait for a while to let the pressure sensor finish the measurement,
      // and then receive data in DMA mode.
      if ((skuPressurePollingPeriod / 2 + 1) == uWaitingForPressureCounter)
      {
        if (0 == Icm20789PressureDataDmaRequest(skuSensorInstance))
        {
          osStatus_t eStatus = osSemaphoreAcquire(SemDataReceivedTdk1Handle, skuPeriod5msInTicks);

          if (osOK == eStatus)
          {
            Icm20789ParsePressureDataDma(skuSensorInstance);
          }
          else
          {
            Icm20789InvalidatePressureOutputPort(skuSensorInstance);
          }
        }
        else
        {
          Icm20789InvalidatePressureOutputPort(skuSensorInstance);
        }

        PressureCompensatorSwcCompensate(1U);

        uIsWaitingForPressure = 0;
      }

      ++uWaitingForPressureCounter;
    }

    ++uCounter;
  }
  /* USER CODE END StartTaskPollIcm1 */
}

/* USER CODE BEGIN Header_StartTaskPollIcm2 */
/**
* @brief Function implementing the TaskPollIcm2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPollIcm2 */
void StartTaskPollIcm2(void *argument)
{
  /* USER CODE BEGIN StartTaskPollIcm2 */
  static const uint32_t skuSensorInstance = 1U;
  static uint32_t uCounter = 0U;
  static const uint32_t skuPressurePollingPeriod = 8U;
  uint8_t uIsWaitingForPressure = 0U;
  uint8_t uWaitingForPressureCounter = 0U;

  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemaphoreTdk2Handle, osWaitForever);

    if (0 == Icm20789ImuDataDmaRequest(skuSensorInstance))
    {
      osStatus_t eStatus = osSemaphoreAcquire(SemDataReceivedTdk2Handle, skuPeriod5msInTicks);

      if (osOK == eStatus)
      {
        Icm20789ParseImuDataDma(skuSensorInstance);
      }
      else
      {
        Icm20789InvalidateImuOutputPort(skuSensorInstance);
      }
    }
    else
    {
      Icm20789InvalidateImuOutputPort(skuSensorInstance);
    }

    ImuPreFilterSwcRun(2U);
    BahrsFilterSwcSetImuInput(2U);

    if ((uCounter % 2) == 0)
    {
      osSemaphoreRelease(SemTaskBahrsFilter3Handle);
    }

    // Trigger pressure measurement at 25Hz
    if (0U == (uCounter % skuPressurePollingPeriod))
    {
      if (0 == uIsWaitingForPressure)
      {
        if(0 == Icm20789TriggerPressureMeasurement(skuSensorInstance))
        {
          uIsWaitingForPressure = 1U;
          uWaitingForPressureCounter = 0U;
        }
        else
        {
          Icm20789InvalidatePressureOutputPort(skuSensorInstance);
          PressureCompensatorSwcCompensate(2U);
        }
      }
    }

    if (1 == uIsWaitingForPressure)
    {
      // We wait for a while to let the pressure sensor finish the measurement,
      // and then receive data in DMA mode.
      if ((skuPressurePollingPeriod / 2 + 1) == uWaitingForPressureCounter)
      {
        if (0 == Icm20789PressureDataDmaRequest(skuSensorInstance))
        {
          osStatus_t eStatus = osSemaphoreAcquire(SemDataReceivedTdk2Handle, skuPeriod5msInTicks);

          if (osOK == eStatus)
          {
            Icm20789ParsePressureDataDma(skuSensorInstance);
          }
          else
          {
            Icm20789InvalidatePressureOutputPort(skuSensorInstance);
          }
        }
        else
        {
          Icm20789InvalidatePressureOutputPort(skuSensorInstance);
        }

        PressureCompensatorSwcCompensate(2U);

        uIsWaitingForPressure = 0;
      }

      ++uWaitingForPressureCounter;
    }

    ++uCounter;
  }
  /* USER CODE END StartTaskPollIcm2 */
}

/* USER CODE BEGIN Header_StartTaskReceiveScha63TData */
/**
* @brief Function implementing the TaskReceiveScha63TData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskReceiveScha63TData */
void StartTaskReceiveScha63TData(void *argument)
{
  /* USER CODE BEGIN StartTaskReceiveScha63TData */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemTaskReceiveScha63TDataHandle, osWaitForever);
    TaskRoutineReceiveScha63TData();
  }
  /* USER CODE END StartTaskReceiveScha63TData */
}

/* USER CODE BEGIN Header_StartTaskPollBmm1 */
/**
* @brief Function implementing the TaskPollBmm1 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPollBmm1 */
void StartTaskPollBmm1(void *argument)
{
  /* USER CODE BEGIN StartTaskPollBmm1 */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemTaskPollBmm1Handle, osWaitForever);
    TaskRoutinePollBmm(0U);
  }
  /* USER CODE END StartTaskPollBmm1 */
}

/* USER CODE BEGIN Header_StartTaskPollBmm2 */
/**
* @brief Function implementing the TaskPollBmm2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPollBmm2 */
void StartTaskPollBmm2(void *argument)
{
  /* USER CODE BEGIN StartTaskPollBmm2 */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemTaskPollBmm2Handle, osWaitForever);
    TaskRoutinePollBmm(1U);
  }
  /* USER CODE END StartTaskPollBmm2 */
}

/* USER CODE BEGIN Header_StartTaskPollBmp */
/**
* @brief Function implementing the TaskPollBmp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPollBmp */
void StartTaskPollBmp(void *argument)
{
  /* USER CODE BEGIN StartTaskPollBmp */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemTaskPollBmpHandle, osWaitForever);
    TaskRoutinePollBmp();
  }
  /* USER CODE END StartTaskPollBmp */
}

/* USER CODE BEGIN Header_StartTaskTaskPollMmc */
/**
* @brief Function implementing the TaskPollMmc thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTaskPollMmc */
void StartTaskTaskPollMmc(void *argument)
{
  /* USER CODE BEGIN StartTaskTaskPollMmc */
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemTaskPollMmcHandle, osWaitForever);
    TaskRoutinePollMmc();
  }
  /* USER CODE END StartTaskTaskPollMmc */
}

/* USER CODE BEGIN Header_StartTaskInit */
/**
* @brief Function implementing the TaskInit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskInit */
void StartTaskInit(void *argument)
{
  /* USER CODE BEGIN StartTaskInit */
  /* Infinite loop */
  for(;;)
  {
    TaskRoutineInit();
    osThreadSuspend(TaskInitHandle);
  }
  /* USER CODE END StartTaskInit */
}

/* USER CODE BEGIN Header_StartTaskMeasureResourcesUsage */
/**
* @brief Function implementing the TaskMeasureResourcesUsage thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMeasureResourcesUsage */
void StartTaskMeasureResourcesUsage(void *argument)
{
  /* USER CODE BEGIN StartTaskMeasureResourcesUsage */
#ifdef MEASURE_RESOURCES_USAGE
  static volatile float sfTotalCpuLoadInPercent = 0.0F;
  static volatile float sfCpuLoadTask10msInPercent = 0.0F;
  static volatile float sfCpuLoadTask5msInPercent = 0.0F;
  static uint32_t uIdleTaskCounterPrev = 0U;
  static uint32_t uTask10msCounterPrev = 0U;
  static uint32_t uTask5msCounterPrev = 0U;
  static uint32_t uTickCountPrev = 0U;
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreAcquire(SemTaskMeasureResourcesUsageHandle, osWaitForever);

    uint32_t uIdleTaskCounter = ulTaskGetIdleRunTimeCounter();
    uint32_t uTickCount = xTaskGetTickCount();
    uint32_t uTickDiff = uTickCount - uTickCountPrev;

    if (uTickDiff > 0U)
    {
      sfTotalCpuLoadInPercent = 100.0F - 100.0F * ((0.001F * (float)(uIdleTaskCounter - uIdleTaskCounterPrev)) / ((float)uTickDiff));
    }

    uint32_t uTaskCount = uxTaskGetNumberOfTasks();
    assert(uTaskCount <= TASK_COUNT_FOR_DIAGNOSTICS);

    uTaskCount = uxTaskGetSystemState(aoTaskStatusArray, uTaskCount, NULL);
    memset((void*)saoStackUsageData, 0, sizeof(saoStackUsageData));

    for (uint32_t uInd = 0U; uInd < uTaskCount; ++uInd)
    {
      if (aoTaskStatusArray[uInd].xHandle == Task10msHandle)
      {
        if (uTickDiff > 0U)
        {
          sfCpuLoadTask10msInPercent = 100.0F * ((0.001F * (float)(aoTaskStatusArray[uInd].ulRunTimeCounter - uTask10msCounterPrev)) / ((float)uTickDiff));
        }

        uTask10msCounterPrev = aoTaskStatusArray[uInd].ulRunTimeCounter;
      }
      else if (aoTaskStatusArray[uInd].xHandle == Task5msHandle)
      {
        if (uTickDiff > 0U)
        {
          sfCpuLoadTask5msInPercent = 100.0F * ((0.001F * (float)(aoTaskStatusArray[uInd].ulRunTimeCounter - uTask5msCounterPrev)) / ((float)uTickDiff));
        }

        uTask5msCounterPrev = aoTaskStatusArray[uInd].ulRunTimeCounter;
      }
      else
      {
        // do nothing
      }

      saoStackUsageData[uInd].uFreeStackSpace_ = osThreadGetStackSpace(aoTaskStatusArray[uInd].xHandle);
      strcpy((char*)saoStackUsageData[uInd].acTaskName_, aoTaskStatusArray[uInd].pcTaskName);

      assert(saoStackUsageData[uInd].uFreeStackSpace_ > MIN_FREE_STACK_SPACE_FOR_ASSERT);
    }

    uIdleTaskCounterPrev = uIdleTaskCounter;
    uTickCountPrev = uTickCount;
  }
#else
  for(;;)
  {
    osThreadSuspend(TaskMeasureResourcesUsageHandle);
  }
#endif /* MEASURE_RESOURCES_USAGE */
  /* USER CODE END StartTaskMeasureResourcesUsage */
}

/* TimerCyclicTaskTriggerCallback function */
void TimerCyclicTaskTriggerCallback(void *argument)
{
  /* USER CODE BEGIN TimerCyclicTaskTriggerCallback */

  static uint32_t suCounter = 0U;

  // Trigger 1ms tasks
  osSemaphoreRelease(SemaphoreTaskCanSenderHandle);

  // Trigger 5ms tasks
  if ( 0U == (suCounter % skuPeriod5msInTicks) )
  {
    osSemaphoreRelease(SemTask5msHandle);
    osSemaphoreRelease(SemaphoreTdk1Handle);
    osSemaphoreRelease(SemaphoreTdk2Handle);
  }

  // Trigger 40ms tasks
  if ( 0U == (suCounter % skuPeriod40msInTicks) )
  {
    osSemaphoreRelease(SemTaskPollBmpHandle);
  }

  // Trigger 100ms tasks. We use phase shift to avoid
  // runtime bottle neck.
  if ( 0U == ((suCounter + 3U) % skuPeriod100msInTicks) )
  {
    osSemaphoreRelease(SemTaskPollBmm1Handle);
    osSemaphoreRelease(SemTaskPollBmm2Handle);
    osSemaphoreRelease(SemTaskPollMmcHandle);
  }

#ifdef MEASURE_RESOURCES_USAGE
  // Trigger run time monitor every 10 seconds
  if ( 0U == (suCounter % skuPeriod10sInTicks) )
  {
    osSemaphoreRelease(SemTaskMeasureResourcesUsageHandle);
  }
#endif /* MEASURE_RESOURCES_USAGE */

  ++suCounter;

  /* USER CODE END TimerCyclicTaskTriggerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
