/*
 * Generic singly-linked list
 * Pavel Kirienko, 2013 (pavel.kirienko@gmail.com)
 */

#include <stdlib.h>
#include "list.h"

void canasListInsert(CanasListEntry** pproot, void* pentry)
{
    CanasListEntry* entry = (CanasListEntry*)pentry;

    if (pproot == NULL || entry == NULL)       // (*root) can be NULL, but (root) can't
        return;

    entry->pnext = *pproot;         // New entry becoming root
    *pproot = entry;                // Root will be the second entry
}

void* canasListRemove(CanasListEntry** pproot, void* pentry)
{
    if (pproot == NULL || pentry == NULL)
        return NULL;

    if (*pproot == NULL)
        return NULL;            // List is empty

    if (*pproot == (CanasListEntry*)pentry)
    {
        *pproot = (*pproot)->pnext;
        return pentry;
    }
    else
    {
        CanasListEntry* p = *pproot;
        while (p->pnext != NULL)
        {
            if (p->pnext == (CanasListEntry*)pentry)
            {
                p->pnext = p->pnext->pnext;
                return pentry;
            }
            p = p->pnext;
        }
    }
    return NULL;
}
