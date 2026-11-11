/// @file WriteToRtePort.h
/// @brief Declaration of RTE port filler function.
/// @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef WRITE_TO_RTE_PORT_H
#define WRITE_TO_RTE_PORT_H

#include "CRte.h"

void WriteToRtePort(CRte::EPortIds ePortId, const std::vector<uint8_t>& korBytes);
void WriteToUserDefinedRtePort(CRte::EPortIds ePortId, const std::vector<uint8_t>& korBytes);

#endif // WRITE_TO_RTE_PORT_H
