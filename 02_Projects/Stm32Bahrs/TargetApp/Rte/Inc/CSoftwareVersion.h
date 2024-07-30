/**
 * @file CSoftwareVersion.h
 * @brief A class storing software version code.
 * @author Fedor Baklanov
 * @date 21 January 2024
 * @copyright Copyright 2024 AMS Advanced Air Mobility Sensors UG. All rights reserved.
 */

#ifndef C_SOFTWARE_VERSION_H
#define C_SOFTWARE_VERSION_H

#include <stdint.h>
#include "General/HelperSoftwareComponentMacros.h"

/**
 * @brief A class to store software version constants.
 */
class CSoftwareVersion
{
public:
  CSoftwareVersion() = delete;
  FORBID_CLASS_COPY_AND_MOVE(CSoftwareVersion)

  static constexpr char skacProjectCode_[3]{'O', 'S', 'S'}; ///< Project code, max 3 characters.
  static constexpr uint16_t skuMajor_{ 1U }; ///< Major version number
  static constexpr uint16_t skuMinor_{ 3U }; ///< Minor version number
};

#endif // C_SOFTWARE_VERSION_H
