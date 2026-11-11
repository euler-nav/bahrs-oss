/**
 * @file WriteToRtePort.cpp
 * @brief Fill RTE ports with debug output data.
 * 
 * THIS FILE WAS AUTOMATICALLY GENERATED FROM
 * A TEMPLATE AND A JSON CONFIGURATION FILE
 *
 * @date {DATE_GENERATED}
 */

#include "WriteToRtePort.h"

void WriteToRtePort(CRte::EPortIds ePortId, const std::vector<uint8_t>& korBytes)
{
  switch (ePortId)
  {
{SWITCH_WRITE_TO_PORT_BY_ID}
  default:
    WriteToUserDefinedRtePort(ePortId, korBytes);
    break;
  }
}
