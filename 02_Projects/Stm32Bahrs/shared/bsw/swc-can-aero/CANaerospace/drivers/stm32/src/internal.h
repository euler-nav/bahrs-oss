/*
 * STM32 CAN driver for the CANaerospace library
 * The application should not include this header
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifdef CAN_DRIVER_STM32CL_INTERNAL_H_
#   error "Your application should never include this file"
#else
#   define CAN_DRIVER_STM32CL_INTERNAL_H_
#endif

#include <stdint.h>
#include "stm32f4xx.h"

/**
 * @defgroup These configuration parameters should be overriden by preprocessor definitions.
 * @{
 */
#ifndef CAN_RX_QUEUE_LEN
#   define CAN_RX_QUEUE_LEN 20
#endif
#ifndef CAN_TX_QUEUE_LEN
#   define CAN_TX_QUEUE_LEN 20
#endif

#ifndef CAN_IRQ_PRIORITY
   // Lowest possible by default. Notes about NVIC and FreeRTOS: http://www.freertos.org/RTOS-Cortex-M3-M4.html
#   define CAN_IRQ_PRIORITY 15
#endif
#ifndef CAN_IRQ_SUBPRIORITY
   // Assuming that subpriorities are not available
#   define CAN_IRQ_SUBPRIORITY 0
#endif

#if !CAN_TIMER_EMULATED
#   ifndef CAN_TIMER_NUMBER
/**
 * Any General-Purpose timer is okay (TIM2, TIM3, TIM4, TIM5)
 */
#       define CAN_TIMER_NUMBER 3
#   endif
#endif

#ifndef CAN_INTERFACE_NUMBER
   // Select either CAN1 or CAN2
#   define CAN_INTERFACE_NUMBER 2
#endif
/**
 * @}
 */

#if CAN_CHIBIOS
#   define CAN_IRQ_HANDLER(id)  CH_IRQ_HANDLER(id)
#   define CAN_IRQ_PROLOGUE()   CH_IRQ_PROLOGUE()
#   define CAN_IRQ_EPILOGUE()   CH_IRQ_EPILOGUE()
#else
#   define CAN_IRQ_HANDLER(id)  void id(void)
#   define CAN_IRQ_PROLOGUE()
#   define CAN_IRQ_EPILOGUE()
#endif

int canSelftest(CAN_TypeDef* CANx);

int canTimerInit(void);
void canTimerDeinit(void);
void canTimerSet(int iface, int usec);
void canTimerStop(int iface);
void canTimerIrq(int iface);           ///< Callback
#if CAN_TIMER_EMULATED
void canTimerEmulIncrementIrq(int usec);
#endif

int canFilterInit(void);
