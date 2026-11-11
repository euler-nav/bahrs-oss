/*
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include "srvport/srvport.h"
#include "util.h"
#include "sys.h"

#define REENT_PRINTF_BUF_SIZE 512

void reentPrintf(const char* fmt, ...)
{
    static char buf[REENT_PRINTF_BUF_SIZE];
    static xSemaphoreHandle mutex = NULL;

    va_list vl;
    va_start(vl, fmt);

    if (mutex == NULL)
    {
        mutex = xSemaphoreCreateMutex();
        ASSERT_ALWAYS(mutex != NULL);
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY))
    {
        vsnprintf(buf, REENT_PRINTF_BUF_SIZE, fmt, vl);
        srvportPrint(buf);
        xSemaphoreGive(mutex);
    }
    va_end(vl);
}
