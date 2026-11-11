/*
 * Message payload conversions
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_MARSHAL_H_
#define CANAEROSPACE_MARSHAL_H_

#include <stdbool.h>
#include <canaerospace/canaerospace.h>

#ifdef __cplusplus
extern "C" {
#endif

int canasHostToNetwork(uint8_t* pdata, const CanasMessageData* phost);
int canasNetworkToHost(CanasMessageData* phost, const uint8_t* pdata, uint8_t datalen, uint8_t datatype);

#ifdef __cplusplus
}
#endif
#endif
