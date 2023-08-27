/**
* @file NavigationUtilities.h
* @brief Different functions related to navigational mathematics.
* @author Fedor Baklanov
* @date 11 Apr 2022
*/
#ifndef NAVIGATION_UTILITIES_H
#define NAVIGATION_UTILITIES_H

#include <type_traits>
#include "Eigen/Dense"
#include "CQuaternion.h"

namespace NNavigationUtilities
{
  /**
   * @brief Compute height above mean sea level from pressure.
   * Valid input range [22700.0 127780.0]. The range corresponds to height from approximately -2000m to 11000m.
   * @param fPressurePascals Measured pressure.
   * @return Height above mean sea level when input is from the valid range, quiet NaN otherwise.
  */
  float HeightFromPressure(float fPressurePascals);

  /**
   * @brief Compute a partial derivative of height above mean sea level with respect to pressure.
   * Valid input range [22700.0 127780.0]. The range corresponds to height from approximately -2000m to 11000m.
   * @param fPressurePascals Measured pressure.
   * @return A value of a partial derivative of height with respect to pressure when input is from the valid range, quiet NaN otherwise.
  */
  float PartialDerivativeHeightWrtPressure(float fPressurePascals);

  /**
   * @brief Compute Euler angles (roll and pitch) from specific force measurements.
   * The function does not check for static conditions. Aerospace sequence of Euler angles is assumed.
   * @param korSpecificForce The specific force measurement.
   * @param frRoll Output roll angle.
   * @param frPitch Output pitch angle.
  */
  void EulerFromSpecificForce(const Eigen::Vector3f& korSpecificForce, float& frRoll, float& frPitch);

  /**
   * @brief Convert a quaternion to Euler angles.
   * Quaternion's norm shall be 1, otherwise usage of this function does not make sense.
   * The function does not vefify the input, it shall be done by a caller.
   * @param korQuaternion A quaternion to be converted.
   * @param frRoll Output roll angle, [rad], from -pi to pi
   * @param frPitch Output pitch angle, [rad], from -pi/2 to pi/2
   * @param frYaw Output yaw angle, [rad], from -pi to pi
  */
  void EulerFromQuaternion(const CQuaternion& korQuaternion, float& frRoll, float& frPitch, float& frYaw);

  /**
   * @brief Compute magnetic heading given known attitude and a magnetometer measurement.
   *
   * Roll must be between +-pi, pitch must be between +-pi/2. The function returns NaN if inputs are not
   * in the expected range. Rturns NaN in gimbal lock, i. e. when pitch is exactly +pi/2 or -pi/2
   * 
   * @param korMagneticFieldVectorInGauss Magnetometer measurement.
   * @param fRoll Roll angle, [rad], must belong to the closed interval [-pi, pi]
   * @param fPitch Pitch angle, [rad]
   * @return Magnetic heading in radians, from -pi to pi.
  */
  float HeadingFromMagnetometer(const Eigen::Vector3f& korMagneticFieldVectorInGauss, float fRoll, float fPitch);

  /**
   * @brief Convert Euler angles to a quaternion.
   * @param fRoll Roll angle.
   * @param fPitch Pitch angle.
   * @param fYaw Yaw angle.
   * @return Quaternion.
  */
  CQuaternion QuaternionFromEuler(float fRoll, float fPitch, float fYaw);

  /**
   * @brief Convert a quaternion to a rotation matrix.
   * @param korQuaternion Input quaternion.
   * @return Rotation matrix.
  */
  Eigen::Matrix3f DcmFromQuaternion(const CQuaternion& korQuaternion);

  /**
   * @brief Computes a signed difference of two unsigned numbers.
   * The function asserts if the diff is greater than the max value of the return type.
   * @tparam tUnsignedType Unsigned type
   * @param[in] uTimestamp1
   * @param[in] uTimestamp2 
   * @return uTimestamp1 minus uTimastamp2
  */
  template<typename tUnsignedType> inline int32_t DiffOfUnsigned(tUnsignedType uTimestamp1, tUnsignedType uTimestamp2)
  {
    static_assert(std::is_unsigned<tUnsignedType>::value == true);
    tUnsignedType uDiff;
    int32_t iResult;

    if (uTimestamp1 >= uTimestamp2)
    {
      uDiff = uTimestamp1 - uTimestamp2;
      iResult = static_cast<int32_t>(uDiff);
    }
    else
    {
      uDiff = uTimestamp2 - uTimestamp1;
      iResult = -static_cast<int32_t>(uDiff);
    }

    using WiderType_t = std::conditional_t<(sizeof(tUnsignedType) > sizeof(uint32_t)), tUnsignedType, uint32_t>;

    assert(static_cast<WiderType_t>(uDiff) <= static_cast<WiderType_t>(std::numeric_limits<int32_t>::max()));

    return iResult;
  }
}

#endif
