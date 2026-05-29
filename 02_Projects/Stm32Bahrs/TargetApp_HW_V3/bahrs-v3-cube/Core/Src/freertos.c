/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "AmsAssert.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticTimer_t osStaticTimerDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
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
osEventFlagsId_t eventGroupAsm330;

/* USER CODE END Variables */
/* Definitions for TaskInit */
osThreadId_t TaskInitHandle;
uint32_t taskInitBuffer[ 256 ];
osStaticThreadDef_t taskInitControlBlock;
const osThreadAttr_t TaskInit_attributes = {
  .name = "TaskInit",
  .cb_mem = &taskInitControlBlock,
  .cb_size = sizeof(taskInitControlBlock),
  .stack_mem = &taskInitBuffer[0],
  .stack_size = sizeof(taskInitBuffer),
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for TaskBmi270 */
osThreadId_t TaskBmi270Handle;
uint32_t TaskBmi270Buffer[ 256 ];
osStaticThreadDef_t TaskBmi270ControlBlock;
const osThreadAttr_t TaskBmi270_attributes = {
  .name = "TaskBmi270",
  .cb_mem = &TaskBmi270ControlBlock,
  .cb_size = sizeof(TaskBmi270ControlBlock),
  .stack_mem = &TaskBmi270Buffer[0],
  .stack_size = sizeof(TaskBmi270Buffer),
  .priority = (osPriority_t) osPriorityHigh,
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
/* Definitions for TaskAsm330 */
osThreadId_t TaskAsm330Handle;
uint32_t TaskAsm330Buffer[ 256 ];
osStaticThreadDef_t TaskAsm330ControlBlock;
const osThreadAttr_t TaskAsm330_attributes = {
  .name = "TaskAsm330",
  .cb_mem = &TaskAsm330ControlBlock,
  .cb_size = sizeof(TaskAsm330ControlBlock),
  .stack_mem = &TaskAsm330Buffer[0],
  .stack_size = sizeof(TaskAsm330Buffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskBmp384 */
osThreadId_t TaskBmp384Handle;
uint32_t TaskBmp384Buffer[ 256 ];
osStaticThreadDef_t TaskBmp384ControlBlock;
const osThreadAttr_t TaskBmp384_attributes = {
  .name = "TaskBmp384",
  .cb_mem = &TaskBmp384ControlBlock,
  .cb_size = sizeof(TaskBmp384ControlBlock),
  .stack_mem = &TaskBmp384Buffer[0],
  .stack_size = sizeof(TaskBmp384Buffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskLps22 */
osThreadId_t TaskLps22Handle;
uint32_t TaskLps22Buffer[ 256 ];
osStaticThreadDef_t TaskLps22ControlBlock;
const osThreadAttr_t TaskLps22_attributes = {
  .name = "TaskLps22",
  .cb_mem = &TaskLps22ControlBlock,
  .cb_size = sizeof(TaskLps22ControlBlock),
  .stack_mem = &TaskLps22Buffer[0],
  .stack_size = sizeof(TaskLps22Buffer),
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
/* Definitions for TaskIcp20100 */
osThreadId_t TaskIcp20100Handle;
uint32_t TaskIcp20100Buffer[ 256 ];
osStaticThreadDef_t TaskIcp20100ControlBlock;
const osThreadAttr_t TaskIcp20100_attributes = {
  .name = "TaskIcp20100",
  .cb_mem = &TaskIcp20100ControlBlock,
  .cb_size = sizeof(TaskIcp20100ControlBlock),
  .stack_mem = &TaskIcp20100Buffer[0],
  .stack_size = sizeof(TaskIcp20100Buffer),
  .priority = (osPriority_t) osPriorityHigh,
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
/* Definitions for TaskMmc5983 */
osThreadId_t TaskMmc5983Handle;
uint32_t TaskMmc5983Buffer[ 256 ];
osStaticThreadDef_t TaskMmc5983ControlBlock;
const osThreadAttr_t TaskMmc5983_attributes = {
  .name = "TaskMmc5983",
  .cb_mem = &TaskMmc5983ControlBlock,
  .cb_size = sizeof(TaskMmc5983ControlBlock),
  .stack_mem = &TaskMmc5983Buffer[0],
  .stack_size = sizeof(TaskMmc5983Buffer),
  .priority = (osPriority_t) osPriorityHigh,
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
/* Definitions for TaskLis3 */
osThreadId_t TaskLis3Handle;
uint32_t TaskLis3Buffer[ 256 ];
osStaticThreadDef_t TaskLis3ControlBlock;
const osThreadAttr_t TaskLis3_attributes = {
  .name = "TaskLis3",
  .cb_mem = &TaskLis3ControlBlock,
  .cb_size = sizeof(TaskLis3ControlBlock),
  .stack_mem = &TaskLis3Buffer[0],
  .stack_size = sizeof(TaskLis3Buffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskBmm350 */
osThreadId_t TaskBmm350Handle;
uint32_t TaskBmm350Buffer[ 256 ];
osStaticThreadDef_t TaskBmm350ControlBlock;
const osThreadAttr_t TaskBmm350_attributes = {
  .name = "TaskBmm350",
  .cb_mem = &TaskBmm350ControlBlock,
  .cb_size = sizeof(TaskBmm350ControlBlock),
  .stack_mem = &TaskBmm350Buffer[0],
  .stack_size = sizeof(TaskBmm350Buffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskProcessSyncPulse */
osThreadId_t TaskProcessSyncPulseHandle;
uint32_t TaskProcessSyncPulseBuffer[ 128 ];
osStaticThreadDef_t TaskProcessSyncPulseControlBlock;
const osThreadAttr_t TaskProcessSyncPulse_attributes = {
  .name = "TaskProcessSyncPulse",
  .cb_mem = &TaskProcessSyncPulseControlBlock,
  .cb_size = sizeof(TaskProcessSyncPulseControlBlock),
  .stack_mem = &TaskProcessSyncPulseBuffer[0],
  .stack_size = sizeof(TaskProcessSyncPulseBuffer),
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
/* Definitions for QueueTaskAsm330TimeStamp */
osMessageQueueId_t QueueTaskAsm330TimeStampHandle;
uint8_t QueueTaskAsm330TimeStampBuffer[ 1 * sizeof( uint64_t ) ];
osStaticMessageQDef_t QueueTaskAsm330TimeStampControlBlock;
const osMessageQueueAttr_t QueueTaskAsm330TimeStamp_attributes = {
  .name = "QueueTaskAsm330TimeStamp",
  .cb_mem = &QueueTaskAsm330TimeStampControlBlock,
  .cb_size = sizeof(QueueTaskAsm330TimeStampControlBlock),
  .mq_mem = &QueueTaskAsm330TimeStampBuffer,
  .mq_size = sizeof(QueueTaskAsm330TimeStampBuffer)
};
/* Definitions for QueueTaskBmi270TimeStamp */
osMessageQueueId_t QueueTaskBmi270TimeStampHandle;
uint8_t QueueTaskBmi270TimeStampBuffer[ 1 * sizeof( uint64_t ) ];
osStaticMessageQDef_t QueueTaskBmi270TimeStampControlBlock;
const osMessageQueueAttr_t QueueTaskBmi270TimeStamp_attributes = {
  .name = "QueueTaskBmi270TimeStamp",
  .cb_mem = &QueueTaskBmi270TimeStampControlBlock,
  .cb_size = sizeof(QueueTaskBmi270TimeStampControlBlock),
  .mq_mem = &QueueTaskBmi270TimeStampBuffer,
  .mq_size = sizeof(QueueTaskBmi270TimeStampBuffer)
};
/* Definitions for QueueTaskBmp384TimeStamp */
osMessageQueueId_t QueueTaskBmp384TimeStampHandle;
uint8_t QueueTaskBmp384TimeStampBuffer[ 1 * sizeof( uint64_t ) ];
osStaticMessageQDef_t QueueTaskBmp384TimeStampControlBlock;
const osMessageQueueAttr_t QueueTaskBmp384TimeStamp_attributes = {
  .name = "QueueTaskBmp384TimeStamp",
  .cb_mem = &QueueTaskBmp384TimeStampControlBlock,
  .cb_size = sizeof(QueueTaskBmp384TimeStampControlBlock),
  .mq_mem = &QueueTaskBmp384TimeStampBuffer,
  .mq_size = sizeof(QueueTaskBmp384TimeStampBuffer)
};
/* Definitions for QueueTaskLps22TimeStamp */
osMessageQueueId_t QueueTaskLps22TimeStampHandle;
uint8_t QueueTaskLps22TimeStampBuffer[ 1 * sizeof( uint64_t ) ];
osStaticMessageQDef_t QueueTaskLps22TimeStampControlBlock;
const osMessageQueueAttr_t QueueTaskLps22TimeStamp_attributes = {
  .name = "QueueTaskLps22TimeStamp",
  .cb_mem = &QueueTaskLps22TimeStampControlBlock,
  .cb_size = sizeof(QueueTaskLps22TimeStampControlBlock),
  .mq_mem = &QueueTaskLps22TimeStampBuffer,
  .mq_size = sizeof(QueueTaskLps22TimeStampBuffer)
};
/* Definitions for QueueTaskIcp20100TimeStamp */
osMessageQueueId_t QueueTaskIcp20100TimeStampHandle;
uint8_t QueueTaskIcp20100TimeStampBuffer[ 1 * sizeof( uint64_t ) ];
osStaticMessageQDef_t QueueTaskIcp20100TimeStampControlBlock;
const osMessageQueueAttr_t QueueTaskIcp20100TimeStamp_attributes = {
  .name = "QueueTaskIcp20100TimeStamp",
  .cb_mem = &QueueTaskIcp20100TimeStampControlBlock,
  .cb_size = sizeof(QueueTaskIcp20100TimeStampControlBlock),
  .mq_mem = &QueueTaskIcp20100TimeStampBuffer,
  .mq_size = sizeof(QueueTaskIcp20100TimeStampBuffer)
};
/* Definitions for QueueTaskMmc5983Timestamp */
osMessageQueueId_t QueueTaskMmc5983TimestampHandle;
uint8_t QueueTaskMmc5983TimestampBuffer[ 1 * sizeof( uint64_t ) ];
osStaticMessageQDef_t QueueTaskMmc5983TimestampControlBlock;
const osMessageQueueAttr_t QueueTaskMmc5983Timestamp_attributes = {
  .name = "QueueTaskMmc5983Timestamp",
  .cb_mem = &QueueTaskMmc5983TimestampControlBlock,
  .cb_size = sizeof(QueueTaskMmc5983TimestampControlBlock),
  .mq_mem = &QueueTaskMmc5983TimestampBuffer,
  .mq_size = sizeof(QueueTaskMmc5983TimestampBuffer)
};
/* Definitions for QueueTaskLis3Timestamp */
osMessageQueueId_t QueueTaskLis3TimestampHandle;
uint8_t QueueTaskLis3TimestampBuffer[ 1 * sizeof( uint64_t ) ];
osStaticMessageQDef_t QueueTaskLis3TimestampControlBlock;
const osMessageQueueAttr_t QueueTaskLis3Timestamp_attributes = {
  .name = "QueueTaskLis3Timestamp",
  .cb_mem = &QueueTaskLis3TimestampControlBlock,
  .cb_size = sizeof(QueueTaskLis3TimestampControlBlock),
  .mq_mem = &QueueTaskLis3TimestampBuffer,
  .mq_size = sizeof(QueueTaskLis3TimestampBuffer)
};
/* Definitions for QueueTaskBmm350Timestamp */
osMessageQueueId_t QueueTaskBmm350TimestampHandle;
uint8_t QueueTaskBmm350TimestampBuffer[ 1 * sizeof( uint64_t ) ];
osStaticMessageQDef_t QueueTaskBmm350TimestampControlBlock;
const osMessageQueueAttr_t QueueTaskBmm350Timestamp_attributes = {
  .name = "QueueTaskBmm350Timestamp",
  .cb_mem = &QueueTaskBmm350TimestampControlBlock,
  .cb_size = sizeof(QueueTaskBmm350TimestampControlBlock),
  .mq_mem = &QueueTaskBmm350TimestampBuffer,
  .mq_size = sizeof(QueueTaskBmm350TimestampBuffer)
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
/* Definitions for asm330lhhInt1Sem */
osSemaphoreId_t asm330lhhInt1SemHandle;
osStaticSemaphoreDef_t asm330lhhInt1SemControlBlock;
const osSemaphoreAttr_t asm330lhhInt1Sem_attributes = {
  .name = "asm330lhhInt1Sem",
  .cb_mem = &asm330lhhInt1SemControlBlock,
  .cb_size = sizeof(asm330lhhInt1SemControlBlock),
};
/* Definitions for SemTaskReceiveScha63TData */
osSemaphoreId_t SemTaskReceiveScha63TDataHandle;
osStaticSemaphoreDef_t SemTaskReceiveScha63TDataControlBlock;
const osSemaphoreAttr_t SemTaskReceiveScha63TData_attributes = {
  .name = "SemTaskReceiveScha63TData",
  .cb_mem = &SemTaskReceiveScha63TDataControlBlock,
  .cb_size = sizeof(SemTaskReceiveScha63TDataControlBlock),
};
/* Definitions for SemTaskCanSender */
osSemaphoreId_t SemTaskCanSenderHandle;
osStaticSemaphoreDef_t SemTaskCanSenderControlBlock;
const osSemaphoreAttr_t SemTaskCanSender_attributes = {
  .name = "SemTaskCanSender",
  .cb_mem = &SemTaskCanSenderControlBlock,
  .cb_size = sizeof(SemTaskCanSenderControlBlock),
};
/* Definitions for SemTask5ms */
osSemaphoreId_t SemTask5msHandle;
osStaticSemaphoreDef_t SemTask5msControlBlock;
const osSemaphoreAttr_t SemTask5ms_attributes = {
  .name = "SemTask5ms",
  .cb_mem = &SemTask5msControlBlock,
  .cb_size = sizeof(SemTask5msControlBlock),
};
/* Definitions for SemTaskBahrsFilter1 */
osSemaphoreId_t SemTaskBahrsFilter1Handle;
osStaticSemaphoreDef_t SemTaskBahrsFilter1ControlBlock;
const osSemaphoreAttr_t SemTaskBahrsFilter1_attributes = {
  .name = "SemTaskBahrsFilter1",
  .cb_mem = &SemTaskBahrsFilter1ControlBlock,
  .cb_size = sizeof(SemTaskBahrsFilter1ControlBlock),
};
/* Definitions for SemTaskBahrsFilter2 */
osSemaphoreId_t SemTaskBahrsFilter2Handle;
osStaticSemaphoreDef_t SemTaskBahrsFilter2ControlBlock;
const osSemaphoreAttr_t SemTaskBahrsFilter2_attributes = {
  .name = "SemTaskBahrsFilter2",
  .cb_mem = &SemTaskBahrsFilter2ControlBlock,
  .cb_size = sizeof(SemTaskBahrsFilter2ControlBlock),
};
/* Definitions for SemTaskBahrsFilter3 */
osSemaphoreId_t SemTaskBahrsFilter3Handle;
osStaticSemaphoreDef_t SemTaskBahrsFilter3ControlBlock;
const osSemaphoreAttr_t SemTaskBahrsFilter3_attributes = {
  .name = "SemTaskBahrsFilter3",
  .cb_mem = &SemTaskBahrsFilter3ControlBlock,
  .cb_size = sizeof(SemTaskBahrsFilter3ControlBlock),
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

void RunTaskInit(void *argument);
extern void RunTaskBmi270(void *argument);
extern void RunTaskRs232Sender(void *argument);
extern void RunTaskAsm330(void *argument);
extern void RunTaskBmp384(void *argument);
extern void RunTaskLps22(void *argument);
extern void RunTaskScha63T(void *argument);
extern void RunTaskIcp20100(void *argument);
extern void RunTaskCanSender(void *argument);
extern void RunTaskCanReceiver(void *argument);
extern void RunTask10ms(void *argument);
extern void RunTaskMmc5983(void *argument);
extern void RunTaskBahrsFilter1(void *argument);
extern void RunTaskBahrsFilter2(void *argument);
extern void RunTaskBahrsFilter3(void *argument);
extern void RunTask5ms(void *argument);
extern void RunTaskLis3(void *argument);
extern void RunTaskBmm350(void *argument);
extern void RunTaskProcessSyncPulse(void *argument);
extern void TimerCyclicTaskTriggerCallback(void *argument);

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
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */

static void uartPutChar(USART_TypeDef* opUart, char cChar)
{
  while ((opUart->SR & USART_SR_TXE) == 0U)
  {
    // wait
  }

  opUart->DR = (uint8_t)cChar;
}

static void uartPutString(USART_TypeDef* opUart, const char* kcpString, uint32_t uMaxLen)
{
  for (uint32_t i = 0; (kcpString != NULL) && (kcpString[i] != '\0') && (i < uMaxLen); ++i)
  {
    uartPutChar(opUart, kcpString[i]);
  }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName)
{
  __disable_irq();

  USART_TypeDef* opUart = USART2;

  uartPutString(opUart, "\r\nSTACK OVERFLOW: ", 32);
  uartPutString(opUart, (const char*)pcTaskName, 32);
  uartPutString(opUart, "\r\n", 2);

  for (;;)
  {
    __NOP();
  }
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
  /* creation of asm330lhhInt1Sem */
  asm330lhhInt1SemHandle = osSemaphoreNew(1, 0, &asm330lhhInt1Sem_attributes);

  /* creation of SemTaskReceiveScha63TData */
  SemTaskReceiveScha63TDataHandle = osSemaphoreNew(1, 0, &SemTaskReceiveScha63TData_attributes);

  /* creation of SemTaskCanSender */
  SemTaskCanSenderHandle = osSemaphoreNew(1, 0, &SemTaskCanSender_attributes);

  /* creation of SemTask5ms */
  SemTask5msHandle = osSemaphoreNew(1, 0, &SemTask5ms_attributes);

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
  osTimerStart(TimerCyclicTaskTriggerHandle, 1U);

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueTaskRs232Sender */
  QueueTaskRs232SenderHandle = osMessageQueueNew (8, sizeof(uint8_t), &QueueTaskRs232Sender_attributes);

  /* creation of QueueTaskAsm330TimeStamp */
  QueueTaskAsm330TimeStampHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskAsm330TimeStamp_attributes);

  /* creation of QueueTaskBmi270TimeStamp */
  QueueTaskBmi270TimeStampHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskBmi270TimeStamp_attributes);

  /* creation of QueueTaskBmp384TimeStamp */
  QueueTaskBmp384TimeStampHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskBmp384TimeStamp_attributes);

  /* creation of QueueTaskLps22TimeStamp */
  QueueTaskLps22TimeStampHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskLps22TimeStamp_attributes);

  /* creation of QueueTaskIcp20100TimeStamp */
  QueueTaskIcp20100TimeStampHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskIcp20100TimeStamp_attributes);

  /* creation of QueueTaskMmc5983Timestamp */
  QueueTaskMmc5983TimestampHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskMmc5983Timestamp_attributes);

  /* creation of QueueTaskLis3Timestamp */
  QueueTaskLis3TimestampHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskLis3Timestamp_attributes);

  /* creation of QueueTaskBmm350Timestamp */
  QueueTaskBmm350TimestampHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskBmm350Timestamp_attributes);

  /* creation of QueueTaskProcessSync */
  QueueTaskProcessSyncHandle = osMessageQueueNew (1, sizeof(uint64_t), &QueueTaskProcessSync_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskInit */
  TaskInitHandle = osThreadNew(RunTaskInit, NULL, &TaskInit_attributes);

  /* creation of TaskBmi270 */
  TaskBmi270Handle = osThreadNew(RunTaskBmi270, NULL, &TaskBmi270_attributes);

  /* creation of TaskRs232Sender */
  TaskRs232SenderHandle = osThreadNew(RunTaskRs232Sender, NULL, &TaskRs232Sender_attributes);

  /* creation of TaskAsm330 */
  TaskAsm330Handle = osThreadNew(RunTaskAsm330, NULL, &TaskAsm330_attributes);

  /* creation of TaskBmp384 */
  TaskBmp384Handle = osThreadNew(RunTaskBmp384, NULL, &TaskBmp384_attributes);

  /* creation of TaskLps22 */
  TaskLps22Handle = osThreadNew(RunTaskLps22, NULL, &TaskLps22_attributes);

  /* creation of TaskReceiveScha63TData */
  TaskReceiveScha63TDataHandle = osThreadNew(RunTaskScha63T, NULL, &TaskReceiveScha63TData_attributes);

  /* creation of TaskIcp20100 */
  TaskIcp20100Handle = osThreadNew(RunTaskIcp20100, NULL, &TaskIcp20100_attributes);

  /* creation of TaskCanSender */
  TaskCanSenderHandle = osThreadNew(RunTaskCanSender, NULL, &TaskCanSender_attributes);

  /* creation of TaskCanReceiver */
  TaskCanReceiverHandle = osThreadNew(RunTaskCanReceiver, NULL, &TaskCanReceiver_attributes);

  /* creation of Task10ms */
  Task10msHandle = osThreadNew(RunTask10ms, NULL, &Task10ms_attributes);

  /* creation of TaskMmc5983 */
  TaskMmc5983Handle = osThreadNew(RunTaskMmc5983, NULL, &TaskMmc5983_attributes);

  /* creation of TaskBahrsFilter1 */
  TaskBahrsFilter1Handle = osThreadNew(RunTaskBahrsFilter1, NULL, &TaskBahrsFilter1_attributes);

  /* creation of TaskBahrsFilter2 */
  TaskBahrsFilter2Handle = osThreadNew(RunTaskBahrsFilter2, NULL, &TaskBahrsFilter2_attributes);

  /* creation of TaskBahrsFilter3 */
  TaskBahrsFilter3Handle = osThreadNew(RunTaskBahrsFilter3, NULL, &TaskBahrsFilter3_attributes);

  /* creation of Task5ms */
  Task5msHandle = osThreadNew(RunTask5ms, NULL, &Task5ms_attributes);

  /* creation of TaskLis3 */
  TaskLis3Handle = osThreadNew(RunTaskLis3, NULL, &TaskLis3_attributes);

  /* creation of TaskBmm350 */
  TaskBmm350Handle = osThreadNew(RunTaskBmm350, NULL, &TaskBmm350_attributes);

  /* creation of TaskProcessSyncPulse */
  TaskProcessSyncPulseHandle = osThreadNew(RunTaskProcessSyncPulse, NULL, &TaskProcessSyncPulse_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  eventGroupAsm330 = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of EventBahrsFiltersCompleted */
  EventBahrsFiltersCompletedHandle = osEventFlagsNew(&EventBahrsFiltersCompleted_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_RunTaskInit */
/**
  * @brief  Function implementing the TaskInit thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RunTaskInit */
__weak void RunTaskInit(void *argument)
{
  /* USER CODE BEGIN RunTaskInit */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RunTaskInit */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

