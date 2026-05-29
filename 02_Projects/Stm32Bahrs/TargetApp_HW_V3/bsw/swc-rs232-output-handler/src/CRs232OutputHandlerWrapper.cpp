#include "CRs232OutputHandlerWrapper.h"
#include "CRs232OutputHandler.h"

extern "C" void Rs232TxInterruptHandler(uint16_t uSize)
{
  CRs232OutputHandler::GetInstance().UARTTxInterruptHandler(uSize);
}
