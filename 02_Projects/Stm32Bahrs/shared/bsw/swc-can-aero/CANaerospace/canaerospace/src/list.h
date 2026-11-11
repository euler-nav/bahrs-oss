/*
 * Generic singly-linked list
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#ifndef CANAEROSPACE_LIST_H_
#define CANAEROSPACE_LIST_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CanasListEntryStruct
{
    struct CanasListEntryStruct* pnext;
} CanasListEntry;

void canasListInsert(CanasListEntry** pproot, void* pentry);
void* canasListRemove(CanasListEntry** pproot, void* pentry);

#ifdef __cplusplus
}
#endif
#endif
