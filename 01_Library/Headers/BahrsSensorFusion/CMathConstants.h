/**
* @file CMathConstants.h
* @brief Definitions of different mathematical constants.
* @author Fedor Baklanov
* @date 29 Dec 2022
* @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef C_MATH_CONSTANTS
#define C_MATH_CONSTANTS

namespace NNavigationUtilities
{
  class CMathConstants
  {
  public:
    static constexpr float skfPi_ { 3.1415926F }; ///< Definition of float pi.
    static constexpr double skdPi_ { 3.14159265358979 }; ///< Definition of double pi.
    static constexpr float skfHalfPi_ { skfPi_ / 2.0F }; ///< Definition of float pi / 2.
    static constexpr double skdHalfPi_{ skdPi_ / 2.0 }; ///< Definition of double pi / 2.
    static constexpr float skfTwoPi_{ skfPi_ * 2.0F }; ///< Definition of float 2 pi.
    static constexpr double skdTwoPi_{ skdPi_ * 2.0 }; ///< Definition of double 2 pi.

  protected:

  private:
  };
}

#endif
