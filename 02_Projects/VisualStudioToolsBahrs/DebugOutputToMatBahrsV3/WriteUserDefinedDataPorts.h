/// @file WriteUserDefinedDataPorts.h
/// @brief Declaration of functions for exporting user-defined data ports to MAT file.
/// @copyright Copyright 2024. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef WRITE_USER_DEFINED_RTE_PORTS_H
#define WRITE_USER_DEFINED_RTE_PORTS_H

#include "CRte.h"
#include "tinymatwriter.h"

void BufferDataFromUserDefinedPorts(CRte::EPortIds ePortId, const std::vector<uint8_t>& orBytes);
void WriteUserDefinedPortsToMat(TinyMATWriterFile* opMatWriter);

#endif // WRITE_USER_DEFINED_RTE_PORTS_H
