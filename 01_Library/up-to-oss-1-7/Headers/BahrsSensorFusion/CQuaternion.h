/**
* @file CQuaternion.h
* @brief Declaration of the quaternion class.
* @author Fedor Baklanov
* @date 6 December 2023
* @copyright Copyright 2023. AMS Advanced Air Mobility Sensors UG. All rights reserved.
*/
#ifndef C_QUATERNION_H
#define C_QUATERNION_H

#include "Eigen/Dense"

class CQuaternion
{
public:
#ifdef UNIT_TESTS
  friend class CQuaternionAccessor;
#endif

  CQuaternion();
  CQuaternion(const Eigen::Vector3f& oVector);
  CQuaternion(float fW, float fX, float fY, float fZ);

  CQuaternion& operator+=(const CQuaternion& korRight);
  CQuaternion operator+(const CQuaternion& korRight) const;
  CQuaternion& operator-=(const CQuaternion& korRight);
  CQuaternion operator-(const CQuaternion& korRight) const;
  CQuaternion operator*(const CQuaternion& korRight) const;
  CQuaternion& operator*=(float fScalar);
  CQuaternion operator*(float fScalar) const;
  CQuaternion& operator*=(double dScalar);
  CQuaternion operator*(double dScalar) const;

  float& W();
  float W() const;
  float& X();
  float X() const;
  float& Y();
  float Y() const;
  float& Z();
  float Z() const;

  void Normalize();


protected:

private:
  Eigen::Vector4f oVector_;
  
};

CQuaternion operator*(float fScalar, const CQuaternion& korRight);
CQuaternion operator*(double fScalar, const CQuaternion& korRight);

#endif /* C_QUATERNION_H */
