/// @file CMagneticHeadingCsvWriter.h
/// @brief Declaration of the CSV writer for magnetic heading output.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_MAGNETIC_HEADING_CSV_WRITER_H
#define C_MAGNETIC_HEADING_CSV_WRITER_H

#include "CCsvWriter.h"
#include "MagneticHeadingFilterApi.h"
#include "RteTypesGenerated.h"

class CMagneticHeadingCsvWriter : public CCsvWriter
{
public:
  CMagneticHeadingCsvWriter() = delete;
  CMagneticHeadingCsvWriter(const string& korFilename);

  void Write(const SMagneticHeading& korOutput);

protected:

private:
  static const vector<string> koColumnNames_;

};

#endif // C_MAGNETIC_HEADING_CSV_WRITER_H
