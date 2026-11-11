/// @file CMagneticHeadingCsvWriter.cpp
/// @brief Implementation of the CSV writer for magnetic heading output.
/// @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CMagneticHeadingCsvWriter.h"


const vector<string> CMagneticHeadingCsvWriter::koColumnNames_{ "Time, [us]",
                                                                "Valid",
                                                                "Magnetic Heading, [rad]" };

CMagneticHeadingCsvWriter::CMagneticHeadingCsvWriter(const string& korFilename) : CCsvWriter(korFilename, koColumnNames_)
{
}

void CMagneticHeadingCsvWriter::Write(const SMagneticHeading& korOutput)
{
  oFile_ << static_cast<int>(korOutput.uValid_)  << ';';
  oFile_ << korOutput.uTimestampUs_ << ';';
  oFile_ << korOutput.fMagneticHeading_ << ';';

  oFile_ << endl;
}