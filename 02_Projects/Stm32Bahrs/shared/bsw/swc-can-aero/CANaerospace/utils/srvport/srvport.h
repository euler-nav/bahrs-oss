/*
 * Service Port
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef SRVPORT_H_
#define SRVPORT_H_

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes the service port in 8-N-1 mode.
 */
int srvportInit(void);
int srvportIsAttached(void);

int srvportWrite(const uint8_t* pdata, int len);
int srvportRead(uint8_t* pdata, int maxlen);
int srvportGetChar(void);

void srvportFlush(void);

void srvportDieWithHonour(const char* const* compound_message) __attribute__((noreturn));

static inline void srvportPrint(const char* str)
{
    srvportWrite((const uint8_t*)str, strlen(str));
}

#ifdef __cplusplus
}
#endif
#endif
