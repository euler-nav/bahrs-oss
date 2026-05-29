#ifndef INC_CRC32_H_
#define INC_CRC32_H_

#include <stdint.h>

typedef uint32_t Crc32_t;

Crc32_t crc32(void* pData, uint32_t uSize, uint32_t uCRC32=0xFFFFFFFF);

#endif /* INC_CRC32_H_ */
