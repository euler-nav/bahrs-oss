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
#include "Icm20789DriverCApi.h"

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
static const uint32_t skuPeriod100msInTicks = (uint32_t) (0.1F * (float)configTICK_RATE_HZ);
static const uint32_t skuPeriod40msInTicks = (uint32_t) (0.04F * (float)configTICK_RATE_HZ);

/* USER CODE END Variables */
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
  .priority = (osPriority_t) osPriorityAboveNormal,
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
/* Definitions for TaskPollIcm1 */
osThreadId_t TaskPollIcm1Handle;
uint32_t TaskPollIcmBuffer1[ 128 ];
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
uint32_t TaskPollIcmBuffer2[ 128 ];
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
uint32_t TaskPollBmm1Buffer[ 128 ];
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
uint32_t TaskPollBmm2Buffer[ 128 ];
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
uint32_t TaskPollBmpBuffer[ 128 ];
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
uint32_t TaskPollMmcBuffer[ 128 ];
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
/* Definitions for SemaphoreTdk1 */
osSemaphoreId_t SemaphoreTdk1Handle;
osStaticSemaphoreDef_t SemaphoreTdk1ControlBlock;
const osSemaphoreAttr_t SemaphoreTdk1_attributes = {
  .name = "SemaphoreTdk1",
  .cb_mem = &SemaphoreTdk1ControlBlock,
  .cb_size = sizeof(SemaphoreTdk1ControlBlock),
};
/* Definitions for SemaphoreTdk2 */
osSemaphoreId_t SemaphoreTdk2Handle;
osStaticSemaphoreDef_t SemaphoreTdk2ControlBlock;
const osSemaphoreAttr_t SemaphoreTdk2_attributes = {
  .name = "SemaphoreTdk2",
  .cb_mem = &SemaphoreTdk2ControlBlock,
  .cb_size = sizeof(SemaphoreTdk2ControlBlock),
};
/* Definitions for SemDataReceivedTdk1 */
osSemaphoreId_t SemDataReceivedTdk1Handle;
osStaticSemaphoreDef_t SemDataReceivedTdk1ControlBlock;
const osSemaphoreAttr_t SemDataReceivedTdk1_attributes = {
  .name = "SemDataReceivedTdk1",
  .cb_mem = &SemDataReceivedTdk1ControlBlock,
  .cb_size = sizeof(SemDataReceivedTdk1ControlBlock),
};
/* Definitions for SemDataReceivedTdk2 */
osSemaphoreId_t SemDataReceivedTdk2Handle;
osStaticSemaphoreDef_t SemDataReceivedTdk2ControlBlock;
const osSemaphoreAttr_t SemDataReceivedTdk2_attributes = {
  .name = "SemDataReceivedTdk2",
  .cb_mem = &SemDataReceivedTdk2ControlBlock,
  .cb_size = sizeof(SemDataReceivedTdk2ControlBlock),
};
/* Definitions for SemTaskReceiveScha63TData */
osSemaphoreId_t SemTaskReceiveScha63TDataHandle;
osStaticSemaphoreDef_t SemTaskReceiveScha63TDataControlBlock;
const osSemaphoreAttr_t SemTaskReceiveScha63TData_attributes = {
  .name = "SemTaskReceiveScha63TData",
  .cb_mem = &SemTaskReceiveScha63TDataControlBlock,
  .cb_size = sizeof(SemTaskReceiveScha63TDataControlBlock),
};
/* Definitions for SemTaskPollBmm1 */
osSemaphoreId_t SemTaskPollBmm1Handle;
osStaticSemaphoreDef_t SemTaskPollBmm1ControlBlock;
const osSemaphoreAttr_t SemTaskPollBmm1_attributes = {
  .name = "SemTaskPollBmm1",
  .cb_mem = &SemTaskPollBmm1ControlBlock,
  .cb_size = sizeof(SemTaskPollBmm1ControlBlock),
};
/* Definitions for SemTaskPollBmm2 */
osSemaphoreId_t SemTaskPollBmm2Handle;
osStaticSemaphoreDef_t SemTaskPollBmm2ControlBlock;
const osSemaphoreAttr_t SemTaskPollBmm2_attributes = {
  .name = "SemTaskPollBmm2",
  .cb_mem = &SemTaskPollBmm2ControlBlock,
  .cb_size = sizeof(SemTaskPollBmm2ControlBlock),
};
/* Definitions for SemTaskPollBmp */
osSemaphoreId_t SemTaskPollBmpHandle;
osStaticSemaphoreDef_t SemTaskPollBmpControlBlock;
const osSemaphoreAttr_t SemTaskPollBmp_attributes = {
  .name = "SemTaskPollBmp",
  .cb_mem = &SemTaskPollBmpControlBlock,
  .cb_size = sizeof(SemTaskPollBmpControlBlock),
};
/* Definitions for SemTaskPollMmc */
osSemaphoreId_t SemTaskPollMmcHandle;
osStaticSemaphoreDef_t SemTaskPollMmcControlBlock;
const osSemaphoreAttr_t SemTaskPollMmc_attributes = {
  .name = "SemTaskPollMmc",
  .cb_mem = &SemTaskPollMmcControlBlock,
  .cb_size = sizeof(SemTaskPollMmcControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask5ms(void *argument);
void StartTask10ms(void *argument);
void StartTaskRs232Sender(void *argument);
void StartTaskProcessSyncPulse(void *argument);
void StartTaskPollIcm1(void *argument);
void StartTaskPollIcm2(void *argument);
void StartTaskReceiveScha63TData(void *argument);
void StartTaskPollBmm1(void *argument);
void StartTaskPollBmm2(void *argument);
void StartTaskPollBmp(void *argument);
void StartTaskTaskPollMmc(void *argument);
void StartTaskInit(void *argument);
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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
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

/* TimerCyclicTaskTriggerCallback function */
void TimerCyclicTaskTriggerCallback(void *argument)
{
  /* USER CODE BEGIN TimerCyclicTaskTriggerCallback */

  static uint32_t suCounter = 0U;

  // Trigger 5ms tasks
  if ( 0U == (suCounter % skuPeriod5msInTicks) )
  {
    osSemaphoreRelease(SemaphoreTask5msHandle);
    osSemaphoreRelease(SemTaskReceiveScha63TDataHandle);
    osSemaphoreRelease(SemaphoreTdk1Handle);
    osSemaphoreRelease(SemaphoreTdk2Handle);
  }

  // Trigger 10ms tasks
  if ( 0U == (suCounter % skuPeriod10msInTicks) )
  {
    osSemaphoreRelease(SemaphoreTask10msHandle);
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

  ++suCounter;

  /* USER CODE END TimerCyclicTaskTriggerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

