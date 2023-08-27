#ifndef C_GEO_CONSTANTS_H
#define C_GEO_CONSTANTS_H

namespace NNavigationUtilities
{
  class CGeoConstants
  {
  public:
    static constexpr float skfBeta = -6.5e-3F; ///< Temperature gradient below 10000m from "ICAO Standard Atmosphere"
    static constexpr float skfEarthRadius = 6356766.0F; ///< Earth radius from "ICAO Standard Atmosphere"
    static constexpr float skfR = 287.05287F; ///< Gas constant constant from "ICAO Standard Atmosphere"
    static constexpr float skfReferencePressure = 1.01325e5F; ///< Pressure at sea level from "ICAO Standard Atmosphere"
    static constexpr float skfReferenceTemperature = 288.15F; ///< Temperature at sea level from "ICAO Standard Atmosphere"
    static constexpr float skfGravity = 9.80665F; ///< Gravity constant from "ICAO Standard Atmosphere"

  protected:

  private:
  };
}

#endif
