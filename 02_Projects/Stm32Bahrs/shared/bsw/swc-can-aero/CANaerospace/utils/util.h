/*
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef UTIL_H_
#define UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * We can't use usual printf() because it is not reentrant
 */
void reentPrintf(const char* fmt, ...);

#ifdef __cplusplus
}
#endif
#endif
