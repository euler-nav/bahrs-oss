/*
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef SYS_H_
#define SYS_H_

#include <stdint.h>
#include <FreeRTOS.h>

#ifdef __cplusplus
extern "C" {
#endif

void sysInit(void);

uint64_t sysTimestampMicros(void);

void assert_failed(const char* file, int line);

#define ASSERT(x) configASSERT(x)

/// This assert will not be disabled even in release builds
#define ASSERT_ALWAYS(x) \
    do { \
        if ((x) == 0) assert_failed(__FILE__, __LINE__); \
    } while (0)

#ifdef __cplusplus
}
#endif
#endif /* SYS_H_ */
