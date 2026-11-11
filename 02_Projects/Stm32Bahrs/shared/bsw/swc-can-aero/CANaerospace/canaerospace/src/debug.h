/*
 * Debug stuff, should only be used for library development
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_DEBUG_H_
#define CANAEROSPACE_DEBUG_H_

#if CANAEROSPACE_DEBUG

#include <stdio.h>
#include <canaerospace/canaerospace.h>

// Debugging can only work in single-threaded environment
static char _canas_debug_strbuf_dumpframe[CANAS_DUMP_BUF_LEN] __attribute__((unused));

#  define CANAS_DUMPFRAME(pframe) canasDumpCanFrame(pframe, _canas_debug_strbuf_dumpframe)
#  define CANAS_TRACE(...) printf(">> "__VA_ARGS__)

#else

#  define CANAS_DUMPFRAME(pframe) ((void)0)
#  define CANAS_TRACE(...) ((void)0)

#endif

#endif
