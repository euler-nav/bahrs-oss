/*
 * Timing stuff
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include "can_driver.h"
#include "internal.h"

#if !CAN_TIMER_EMULATED

#include <stdint.h>

#define GLUE2_(A, B) A##B
#define GLUE2(A, B) GLUE2_(A, B)
#define GLUE3_(A, B, C) A##B##C
#define GLUE3(A, B, C) GLUE3_(A, B, C)

// macro instantiation:
#define RCC_APB1ENR_TIMxEN  GLUE3(RCC_APB1ENR_TIM, CAN_TIMER_NUMBER, EN)
#define TIMx                GLUE2(TIM, CAN_TIMER_NUMBER)
#define TIMx_IRQn           GLUE3(TIM, CAN_TIMER_NUMBER, _IRQn)
#define TIMx_IRQHandler     GLUE3(TIM, CAN_TIMER_NUMBER, _IRQHandler)
#define TIMx_Handler        GLUE2(htim,CAN_TIMER_NUMBER)

#define TIMER_USEC_PER_TICK 100
#define TIMER_CLOCK_HZ      (1000000 / TIMER_USEC_PER_TICK)

extern TIM_HandleTypeDef TIMx_Handler;
TIM_HandleTypeDef *htim_can = &TIMx_Handler;

/**
  * @brief  Output Compare callback in non-blocking mode
  * @param  htim TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if( htim->Instance == TIMx )
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
	        canTimerIrq(0);
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
	        canTimerIrq(1);
		}
	}
}

/**
  * @brief  Enable the can timer and the corresponding interrupts
  * @param  None
  * @retval None
  */
int canTimerInit(void)
{
    //Timer configuration already done by CubeMX
	HAL_TIM_Base_MspInit(htim_can);
    return HAL_OK;
}

/**
  * @brief  Disable the can timer and the corresponding interrupts
  * @param  None
  * @retval None
  */
void canTimerDeinit(void)
{
	HAL_TIM_Base_MspDeInit(htim_can);
}

/**
  * @brief  Start the usec timeout on the selected interface
  * @param  iface Interface where to configure the timeout
  * @retval usec  Timeout time in microseconds
  */
void canTimerSet(int iface, int usec)
{
    if (iface != 0 && iface != 1)
        return;

    HAL_TIM_OC_Stop_IT(htim_can, (iface == 0) ? TIM_CHANNEL_1 : TIM_CHANNEL_2);

    int clocks = usec / TIMER_USEC_PER_TICK;
    if (clocks > 0xFFFF)
        clocks = 0xFFFF;
    if (clocks < 2)
        clocks = 2;

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_TIMING;
    oc.Pulse = TIMx->CNT + clocks;
    HAL_TIM_OC_ConfigChannel(htim_can, &oc, (iface == 0) ? TIM_CHANNEL_1 : TIM_CHANNEL_2);

    HAL_TIM_OC_Start_IT(htim_can, (iface == 0) ? TIM_CHANNEL_1 : TIM_CHANNEL_2);
}

/**
  * @brief  Cancel the usec timeout on the selected interface
  * @param  iface Interface where to cancel the timeout
  * @retval None
  */
void canTimerStop(int iface)
{
    if (iface != 0 && iface != 1)
        return;
    HAL_TIM_OC_Stop_IT(htim_can, (iface == 0) ? TIM_CHANNEL_1 : TIM_CHANNEL_2);
}

#else  // if CAN_TIMER_EMULATED is actually true

static int _remained_usec[CAN_IFACE_COUNT];

int canTimerInit(void)
{
    canTimerDeinit();
    return 0;
}

void canTimerDeinit(void)
{
    for (int i = 0; i < CAN_IFACE_COUNT; i++)
        _remained_usec[i] = -1;
}

void canTimerSet(int iface, int usec)
{
    if (iface < 0 || iface >= CAN_IFACE_COUNT)
        return;
    _remained_usec[iface] = usec;
}

void canTimerStop(int iface)
{
    if (iface < 0 || iface >= CAN_IFACE_COUNT)
        return;
    _remained_usec[iface] = -1;
}

void canTimerEmulIncrementIrq(int usec)
{
    for (int i = 0; i < CAN_IFACE_COUNT; i++)
    {
        if (_remained_usec[i] < 0)
            continue;
        _remained_usec[i] -= usec;
        if (_remained_usec[i] <= 0)
            canTimerIrq(i);
    }
}

#endif
