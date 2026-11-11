/// @file CBahrsCsvWriter.cpp
/// @brief Implementation of the CSV writer for BAHRS filter output.
/// @copyright Copyright 2022. AMS Advanced Air Mobility Sensors UG. All rights reserved.

#include "CBahrsCsvWriter.h"


const vector<string> CBahrsCsvWriter::koColumnNames_{ "Time, [us]",
                                                      "Filter Mode",
                                                      "Height, [m]",
                                                      "Velocity Down, [m/s]",
                                                      "Quat Ned To Body W",
                                                      "Quat Ned To Body X",
                                                      "Quat Ned To Body Y",
                                                      "Quat Ned To Body Z",
                                                      "Accel Bias X, [m/s^2]",
                                                      "Accel Bias Y, [m/s^2]",
                                                      "Accel Bias Z, [m/s^2]",
                                                      "Gyro Bias X, [rad/s]",
                                                      "Gyro Bias Y, [rad/s]",
                                                      "Gyro Bias Z, [rad/s]",
                                                      "Height Std, [m]",
                                                      "Velocity Down Std, [m/s]",
                                                      "Attitude Std 1, [rad]",
                                                      "Attitude Std 2, [rad]",
                                                      "Attitude Std 3, [rad]",
                                                      "Accel Bias Std X, [m/s^2]",
                                                      "Accel Bias Std Y, [m/s^2]",
                                                      "Accel Bias Std Z, [m/s^2]",
                                                      "Gyro Bias Std X, [rad/s]",
                                                      "Gyro Bias Std Y, [rad/s]",
                                                      "Gyro Bias Std Z, [rad/s]" };

CBahrsCsvWriter::CBahrsCsvWriter(const string& korFilename) : CCsvWriter(korFilename, koColumnNames_)
{
}

void CBahrsCsvWriter::Write(const NBahrsFilterApi::SOutputData& korOutput)
{
  oFile_ << korOutput.uTimestampUs_ << ';';
  oFile_ << static_cast<int>(korOutput.eFilterMode_) << ';';

  oFile_ << korOutput.oState_.fHeight_ << ';';
  oFile_ << korOutput.oState_.fVelocityDown_ << ';';
  oFile_ << korOutput.oState_.oQuaternionBodyToNed_.W() << ';';
  oFile_ << korOutput.oState_.oQuaternionBodyToNed_.X() << ';';
  oFile_ << korOutput.oState_.oQuaternionBodyToNed_.Y() << ';';
  oFile_ << korOutput.oState_.oQuaternionBodyToNed_.Z() << ';';
  oFile_ << korOutput.oState_.oAccelerometerBias_(0) << ';';
  oFile_ << korOutput.oState_.oAccelerometerBias_(1) << ';';
  oFile_ << korOutput.oState_.oAccelerometerBias_(2) << ';';
  oFile_ << korOutput.oState_.oGyroscopeBias_(0) << ';';
  oFile_ << korOutput.oState_.oGyroscopeBias_(1) << ';';
  oFile_ << korOutput.oState_.oGyroscopeBias_(2) << ';';

  oFile_ << korOutput.oStateStd_.fHeight_ << ';';
  oFile_ << korOutput.oStateStd_.fVelocityDown_ << ';';
  oFile_ << korOutput.oStateStd_.oAttitude_(0) << ';';
  oFile_ << korOutput.oStateStd_.oAttitude_(1) << ';';
  oFile_ << korOutput.oStateStd_.oAttitude_(2) << ';';
  oFile_ << korOutput.oStateStd_.oAccelerometerBias_(0) << ';';
  oFile_ << korOutput.oStateStd_.oAccelerometerBias_(1) << ';';
  oFile_ << korOutput.oStateStd_.oAccelerometerBias_(2) << ';';
  oFile_ << korOutput.oStateStd_.oGyroscopeBias_(0) << ';';
  oFile_ << korOutput.oStateStd_.oGyroscopeBias_(1) << ';';
  oFile_ << korOutput.oStateStd_.oGyroscopeBias_(2) << ';';

  oFile_ << endl;
}
