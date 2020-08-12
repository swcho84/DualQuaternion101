#ifndef DUAL_QUAT_101_QUATERNION_OPERATIONS_H
#define DUAL_QUAT_101_QUATERNION_OPERATIONS_H

// using common headers for quaternion
#include "global_header.h"

using namespace std;
using namespace ros;
using namespace Eigen;

template <typename T>
T sinc(T x)
{
  return (x == 0) ? 1 : std::sin(x) / x;
}

class QuaternionOperation
{
public:
  QuaternionOperation();
  ~QuaternionOperation();

  // North(x-axis, fwd(+)), East(y-axis, right-side(+)), Down(z-axis, downward(+))
  // Euler angle(3-2-1 transformation)
  Quaterniond GetAddQuaternions(Quaterniond q1, Quaterniond q2);
  Quaterniond GetSubQuaternions(Quaterniond q1, Quaterniond q2);
  Quaterniond GetConjugateQuaternion(Quaterniond q);
  Quaterniond GetScalarMultiplyQuaternion(double scalar, Quaterniond q);
  Quaterniond GetMultiplyQuaternions(Quaterniond q1, Quaterniond q2);
  Quaterniond GetHalfQuaternion(Quaterniond q);
  Quaterniond GetExpQuaternion(Quaterniond q);
  Quaterniond GetLogQuaternion(Quaterniond q);
  double GetNormQuaternion(Quaterniond q);
  Matrix3d GetQuaternionRotationMatrix(Quaterniond q);
  Matrix3d GetQuaternionSkewMatrix(Quaterniond q);
  Matrix3d GetQuaternionSymmSkewMatrix(Quaterniond q);

  Quaterniond GetSingleQuaternion(double qx, double qy, double qz, double qw);
  Vector3d GetYPREulerAngFromQuaternion(Quaterniond q);
  Quaterniond GetQuaternionFromYPREulerAng(Vector3d euler);
  Vector4d GetAxisAngFromQuaternion(Quaterniond q);
  Vector3d GetAxisMagFromQuaternion(Vector4d axisAng);

  float wrapF(float angle);
  double wrapD(double angle);

private:
  // North(x-axis, fwd(+)), East(y-axis, right-side(+)), Down(z-axis, downward(+))
  // Euler angle(3-2-1 transformation)
  Quaterniond CalcAddQuaternions(Quaterniond q1, Quaterniond q2);
  Quaterniond CalcSubQuaternions(Quaterniond q1, Quaterniond q2);
  Quaterniond CalcConjugateQuaternion(Quaterniond q);
  Quaterniond CalcScalarMultiplyQuaternion(double scalar, Quaterniond q);
  Quaterniond CalcMultiplyQuaternions(Quaterniond q1, Quaterniond q2);
  Quaterniond CalcHalfQuaternion(Quaterniond q);
  Quaterniond CalcExpQuaternion(Quaterniond q);
  Quaterniond CalcLogQuaternion(Quaterniond q);
  double CalcNormQuaternion(Quaterniond q);
  Matrix3d CalcQuaternionRotationMatrix(Quaterniond q);
  Matrix3d CalcQuaternionSkewMatrix(Quaterniond q);
  Matrix3d CalcQuaternionSymmSkewMatrix(Quaterniond q);

  Quaterniond CalcSingleQuaternion(double qx, double qy, double qz, double qw);
  Vector3d CalcYPREulerAngFromQuaternion(Quaterniond q);
  Quaterniond CalcQuaternionFromYPREulerAng(Vector3d euler);
  Vector4d CalcAxisAngFromQuaternion(Quaterniond q);
  Vector3d CalcAxisMagFromQuaternion(Vector4d axisAng);
};

#endif  // DUAL_QUAT_101_QUATERNION_OPERATIONS_H