/// @file CCsvWriter.h
/// @brief Declaration of the CSV writer base class.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#ifndef C_CSV_WRITER_H
#define C_CSV_WRITER_H

#include <fstream>
#include <string>
#include <vector>

using namespace std;

class CCsvWriter
{
public:
  CCsvWriter() = delete;
  CCsvWriter(const string& korFilename, const vector<string>& korColumnNames);
  ~CCsvWriter();

  bool IsOpen();

protected:
  ofstream oFile_;

private:

};

#endif // C_CSV_WRITER_H
