/// @file CBahrsCsvWriter.h
/// @brief Declaration of the CSV writer for BAHRS filter output.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_BAHRS_CSV_WRITER_H
#define C_BAHRS_CSV_WRITER_H

#include "CCsvWriter.h"
#include "BahrsFilterApi.h"

class CBahrsCsvWriter : public CCsvWriter
{
public:
  CBahrsCsvWriter() = delete;
  CBahrsCsvWriter(const string& korFilename);

  void Write(const NBahrsFilterApi::SOutputData& korOutput);

protected:

private:
  static const vector<string> koColumnNames_;

};

#endif // C_BAHRS_CSV_WRITER_H
