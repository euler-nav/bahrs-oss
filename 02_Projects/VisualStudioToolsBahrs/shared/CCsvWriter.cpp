/// @file CCsvWriter.cpp
/// @brief Implementation of the CSV writer base class.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CCsvWriter.h"
#include <iostream>

CCsvWriter::CCsvWriter(const string& korFilename, const vector<string>& korColumnNames)
{
  oFile_.open(korFilename, ios_base::out | ios_base::trunc);

  if (true == oFile_.is_open())
  {
    for (const auto& korColumnName : korColumnNames)
    {
      oFile_ << korColumnName << ';';
    }

    oFile_ << endl;
  }
  else
  {
    cout << endl << "Failed to open " << korFilename << " for writing. Error!!!" << endl;
  }
}

CCsvWriter::~CCsvWriter()
{
  if (true == oFile_.is_open())
  {
    oFile_.close();
  }
}

bool CCsvWriter::IsOpen()
{
  return oFile_.is_open();
}
