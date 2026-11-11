/*
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdio.h>
#include <stdlib.h>
#include <FreeRTOS.h>
#include <task.h>
#include "srvport/srvport.h"
#include "sys.h"
#include "util.h"

#include "main.h"

unsigned long ulRunTimeStatsClock = 0;

static uint64_t _sys_time_ms = 0;

void sysInit(void)
{
    // Service stuff
    ASSERT_ALWAYS(srvportInit() == 0);
    ASSERT_ALWAYS((1000 % configTICK_RATE_HZ) == 0);
}

uint64_t sysTimestampMicros(void)
{
    register uint64_t ret = 0;
    vPortEnterCritical();
    ret = _sys_time_ms;
    vPortExitCritical();
    return ret * 1000ul;
}

__WEAK void vApplicationIdleHook(void)
{
    __asm volatile("wfi");
}

__WEAK void vApplicationTickHook(void)
{
    _sys_time_ms += portTICK_RATE_MS;
}

__WEAK void vApplicationMallocFailedHook(void)
{
    const char* pstr[] = { "OUT OF MEMORY ", NULL };
    srvportDieWithHonour(pstr);
}

__WEAK void vApplicationStackOverflowHook(xTaskHandle xTask, signed portCHAR *pcTaskName)
{
    (void)xTask;
    const char* pstr[] = { "STACK OVERFLOW ", (const char*)pcTaskName, NULL };
    srvportDieWithHonour(pstr);
}

void assert_failed(const char* file, int line)
{
    char buf[13];
    const char* pstr[] = { "ASSERT ", file, ":", itoa(line, buf, 10), NULL };
    srvportDieWithHonour(pstr);
}
