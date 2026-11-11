/*
 * CANaerospace launch test
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <string.h>
#include <can_driver.h>
#include <canaerospace/canaerospace.h>
#include <canaerospace/param_id/nod_default.h>
#include <canaerospace/services/std_identification.h>
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "srvport/srvport.h"
#include "sys.h"
#include "util.h"

#define STATIC_MALLOC_POOL_SIZE  1024

static const int MY_NODE_ID      = 42;
static const int MY_REDUND_CHAN  = 0;
static const int MY_SERVICE_CHAN = 8;

static const int MY_HARDWARE_REVISION = 0xab;
static const int MY_SOFTWARE_REVISION = 0xcd;

/**
 * Platform-specific data
 */
typedef struct
{
    uint8_t malloc_pool[STATIC_MALLOC_POOL_SIZE] __attribute__((aligned(__BIGGEST_ALIGNMENT__)));
    char dump_buf[CANAS_DUMP_BUF_LEN];
    int malloc_offset;
    xSemaphoreHandle rmutex;
} CanasPlatform;

static CanasPlatform _canas_platform;
static CanasInstance _canas_instance;

static void* _staticMalloc(CanasInstance* pi, int size)
{
    ASSERT(pi);
    ASSERT(size > 0);
    CanasPlatform* ppl = (CanasPlatform*)pi->pthis;
    ASSERT(ppl);

    // Proper alignment is absolutely vital:
    ASSERT(((size_t)ppl->malloc_pool) % __BIGGEST_ALIGNMENT__ == 0);
    if (size % __BIGGEST_ALIGNMENT__ != 0)
        size += __BIGGEST_ALIGNMENT__ - size % __BIGGEST_ALIGNMENT__;

    if (size >= STATIC_MALLOC_POOL_SIZE - ppl->malloc_offset)
    {
        srvportPrint("Not enough memory in pool\n");
        return NULL;
    }
    void* ptr = ppl->malloc_pool + ppl->malloc_offset;
    ppl->malloc_offset += size;
    reentPrintf("Static malloc: %i requested, %i in use, %i left\n",
                size, ppl->malloc_offset, STATIC_MALLOC_POOL_SIZE - ppl->malloc_offset);
    return ptr;
}

static uint64_t _getTimestamp(CanasInstance* pi)
{
    (void)pi;
    return sysTimestampMicros();
}

static void _delayMs(int timeout_ms)
{
    vTaskDelay(timeout_ms / (1000 / configTICK_RATE_HZ));
}

static void _paramCallback(CanasInstance* pi, CanasParamCallbackArgs* pargs)
{
    ASSERT(pi);
    ASSERT(pargs);
    reentPrintf("PARAM %i[%i]   %s\n", pargs->message_id, pargs->redund_channel_id,
        canasDumpMessage(&pargs->message, _canas_platform.dump_buf));
}

static void _taskSpin(void* parg)
{
    (void)parg;
    srvportPrint("Spinner started\n");
    static const int UPDATE_INTERVAL_USEC = 10 * 1000;
    while (1)
    {
        CanasCanFrame frm;
        int iface = -1;
        /*
         * Note that CAN driver is fully reentrant, and it is possible to perform
         * blocking canReceive() and canSend() (or whatever) at the same time.
         */
        int ret = canReceive(&iface, &frm, UPDATE_INTERVAL_USEC);
        if (ret < 0)
        {
            reentPrintf("Reception failed: %i\n", ret);
            continue;
        }
        if (xSemaphoreTakeRecursive(_canas_platform.rmutex, portMAX_DELAY))
        {
            if (ret == 0)
                ret = canasUpdate(&_canas_instance, -1, NULL);     // No data received
            else
            {
                ASSERT(iface >= 0 && iface < CAN_IFACE_COUNT);
                ret = canasUpdate(&_canas_instance, iface, &frm);  // Pass a new frame
            }
            if (ret != 0)
                reentPrintf("Temporary failure in canasUpdate(): %i\n", ret);
            xSemaphoreGiveRecursive(_canas_platform.rmutex);
        }
        else
            srvportPrint("Failed to lock the mutex\n");
    }
    vTaskDelete(NULL);
}

static void _taskPublisherTemperature(void* parg)
{
    (void)parg;
    srvportPrint("Temperature publisher started\n");
    while (1)
    {
        _delayMs(20000);
        if (xSemaphoreTakeRecursive(_canas_platform.rmutex, portMAX_DELAY))
        {
            CanasMessageData msgd;
            msgd.type = CANAS_DATATYPE_FLOAT;
            //msgd.container.FLOAT = sensReadTemperatureKelvin();
            msgd.container.FLOAT = 1.23;
            /*
             * One does not simply publish a parameter.
             * It's necessary to advertise it first, and we did it indeed in _init().
             */
            ASSERT_ALWAYS(0 == canasParamPublish(&_canas_instance, CANAS_NOD_DEF_OUTSIDE_AIR_TEMPERATURE, &msgd, 0));
            xSemaphoreGiveRecursive(_canas_platform.rmutex);
        }
        else
            srvportPrint("Failed to lock the mutex\n");
    }
    vTaskDelete(NULL);
}

static void _taskPublisherVoltage(void* parg)
{
    (void)parg;
    srvportPrint("Voltage publisher started\n");
    while (1)
    {
        _delayMs(10000);
        if (xSemaphoreTakeRecursive(_canas_platform.rmutex, portMAX_DELAY))
        {
            CanasMessageData msgd;
            msgd.type = CANAS_DATATYPE_FLOAT;
            //msgd.container.FLOAT = sensReadVoltage();
            msgd.container.FLOAT = 4.56;
            ASSERT_ALWAYS(0 == canasParamPublish(&_canas_instance, CANAS_NOD_DEF_DC_SYSTEM_1_VOLTAGE, &msgd, 0));
            xSemaphoreGiveRecursive(_canas_platform.rmutex);
        }
        else
            srvportPrint("Failed to lock the mutex\n");
    }
    vTaskDelete(NULL);
}

static void _taskMonitor(void* parg)
{
    (void)parg;
    srvportPrint("Monitor started\n");
    while (1)
    {
        _delayMs(2000);
        if (xSemaphoreTakeRecursive(_canas_platform.rmutex, portMAX_DELAY))
        {
            for (int i = 0; i < 2; i++)
            {
                const unsigned int errmask = canYieldErrors(i);
                if (errmask)
                {
                    reentPrintf("CAN%i errmask %04x\n", i, errmask);
                }
            }
            xSemaphoreGiveRecursive(_canas_platform.rmutex);
        }
        else
            srvportPrint("Failed to lock the mutex\n");
    }
    vTaskDelete(NULL);
}

static int _drvSend(CanasInstance* pi, int iface, const CanasCanFrame* pframe)
{
    ASSERT(pi);
    ASSERT(iface >= 0 && iface < CAN_IFACE_COUNT);
    ASSERT(pframe);
    return canSend(iface, pframe);
}

static int _drvFilter(CanasInstance* pi, int iface, const CanasCanFilterConfig* pfilters, int nfilters)
{
    ASSERT(pi);
    ASSERT(iface >= 0 && iface < CAN_IFACE_COUNT);
    ASSERT(pfilters);
    ASSERT(nfilters > 0);
    return canFilterSetup(iface, pfilters, nfilters);
}

void canas_launch_test(void)
{
    sysInit();
    srvportPrint("Hello\n");
    ASSERT_ALWAYS(0 == canInit(500000, 4000));

    CanasConfig cfg = canasMakeConfig();
    cfg.filters_per_iface = CAN_FILTERS_PER_IFACE;
    cfg.iface_count = CAN_IFACE_COUNT;
    cfg.fn_send     = _drvSend;
    cfg.fn_filter   = _drvFilter;
    cfg.fn_malloc   = _staticMalloc;        // No dynamic memory needed. Thus, free() is not necessary too.
    cfg.fn_timestamp = _getTimestamp;
    // Each function pointer is initialized to NULL, so no need to assign them by hand.
    // These settings are application-defined:
    cfg.node_id           = MY_NODE_ID;
    cfg.redund_channel_id = MY_REDUND_CHAN;
    cfg.service_channel   = MY_SERVICE_CHAN;

    memset(&_canas_platform, 0, sizeof(_canas_platform));
    _canas_platform.rmutex = xSemaphoreCreateRecursiveMutex();
    ASSERT_ALWAYS(_canas_platform.rmutex != NULL);

    ASSERT_ALWAYS(0 == canasInit(&_canas_instance, &cfg, &_canas_platform));

    // IDS service (it is mandatory for every node):
    CanasSrvIdsPayload ids_selfdescr;
    ids_selfdescr.hardware_revision = MY_HARDWARE_REVISION;
    ids_selfdescr.software_revision = MY_SOFTWARE_REVISION;
    ids_selfdescr.id_distribution   = CANAS_SRV_IDS_ID_DISTRIBUTION_STD;  // These two are standard
    ids_selfdescr.header_type       = CANAS_SRV_IDS_HEADER_TYPE_STD;
    ASSERT_ALWAYS(0 == canasSrvIdsInit(&_canas_instance, &ids_selfdescr, 0));

    // Parameter subscriptions and advertisements:
    ASSERT_ALWAYS(0 == canasParamSubscribe(&_canas_instance, CANAS_NOD_DEF_UTC, 1, _paramCallback, NULL));
    ASSERT_ALWAYS(0 == canasParamSubscribe(&_canas_instance, CANAS_NOD_DEF_DATE, 1, _paramCallback, NULL));
    ASSERT_ALWAYS(0 == canasParamAdvertise(&_canas_instance, CANAS_NOD_DEF_OUTSIDE_AIR_TEMPERATURE, false));
    ASSERT_ALWAYS(0 == canasParamAdvertise(&_canas_instance, CANAS_NOD_DEF_DC_SYSTEM_1_VOLTAGE, false));

    // FreeRTOS hooks
    ASSERT_ALWAYS(xTaskCreate(_taskSpin, "spin",
        configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL));

    ASSERT_ALWAYS(xTaskCreate(_taskMonitor, "monitor",
        configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 1, NULL));

    ASSERT_ALWAYS(xTaskCreate(_taskPublisherTemperature, "pub_t",
        configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL));

    ASSERT_ALWAYS(xTaskCreate(_taskPublisherVoltage, "pub_v",
        configMINIMAL_STACK_SIZE * 4, NULL, tskIDLE_PRIORITY + 2, NULL));
}

