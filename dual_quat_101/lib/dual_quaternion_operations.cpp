#include "dual_quaternion_operations.h"

using namespace std;
using namespace ros;
using namespace Eigen;

DualQuaternionOperation::DualQuaternionOperation() : quatOper_()
{
}

DualQuaternionOperation::~DualQuaternionOperation()
{
}

void DualQuaternionOperation::MainLoop()
{
}

DualQuaternion DualQuaternionOperation::CalcAttPos2Quaternion(Quaterniond qAtt, Vector3d pos)
{
  DualQuaternion result;
  Quaterniond qPos;  // virtual quaternion using the NED order position
  qPos.x() = pos(0);
  qPos.y() = pos(1);
  qPos.z() = pos(2);
  qPos.w() = 0.0;
  result.qReal = qAtt;
  result.qDual = quatOper_.GetScalarMultiplyQuaternion(0.5, quatOper_.GetMultiplyQuaternions(qPos, qAtt));
  return result;
}

QuatPos DualQuaternionOperation::CalcDualQuaternion2AttPos(DualQuaternion dualQuat)
{
  Quaterniond qPos;  // virtual quaternion using the NED order position
  qPos = quatOper_.GetScalarMultiplyQuaternion(
      2.0, quatOper_.GetMultiplyQuaternions(dualQuat.qDual, quatOper_.GetConjugateQuaternion(dualQuat.qReal)));
  QuatPos result;
  result.qAtt = dualQuat.qReal;
  result.pos(0) = qPos.x();
  result.pos(1) = qPos.y();
  result.pos(2) = qPos.z();
  return result;
}

DualQuaternion DualQuaternionOperation::CalcConjugateDualQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetConjugateQuaternion(dualQuat.qReal);
  result.qDual = quatOper_.GetConjugateQuaternion(dualQuat.qDual);
  return result;
}

DualQuaternion DualQuaternionOperation::CalcMultiplyDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetMultiplyQuaternions(dualQuat1.qReal, dualQuat2.qReal);
  result.qDual = quatOper_.GetAddQuaternions(quatOper_.GetMultiplyQuaternions(dualQuat1.qReal, dualQuat2.qDual),
                                             quatOper_.GetMultiplyQuaternions(dualQuat1.qDual, dualQuat2.qReal));
  return result;
}

DualQuaternion DualQuaternionOperation::CalcAddDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetAddQuaternions(dualQuat1.qReal, dualQuat2.qReal);
  result.qDual = quatOper_.GetAddQuaternions(dualQuat1.qDual, dualQuat2.qDual);
  return result;
}

DualQuaternion DualQuaternionOperation::CalcSubDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetSubQuaternions(dualQuat1.qReal, dualQuat2.qReal);
  result.qDual = quatOper_.GetSubQuaternions(dualQuat1.qDual, dualQuat2.qDual);
  return result;
}

DualQuaternion DualQuaternionOperation::CalcPureRotationDualQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  result.qReal = dualQuat.qReal;
  result.qDual.x() = 0.0;
  result.qDual.y() = 0.0;
  result.qDual.z() = 0.0;
  result.qDual.w() = 0.0;
  return result;
}

DualQuaternion DualQuaternionOperation::CalcExponentialDualQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetExpQuaternion(dualQuat.qReal);
  result.qDual = quatOper_.GetMultiplyQuaternions(result.qReal, dualQuat.qDual);
  return result;
}

DualQuaternion DualQuaternionOperation::CalcLogDualQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetLogQuaternion(dualQuat.qReal);
  result.qDual = quatOper_.GetMultiplyQuaternions(quatOper_.GetConjugateQuaternion(dualQuat.qReal), dualQuat.qDual);
  double scale = (1.0) / (dualQuat.qReal.squaredNorm());
  result.qDual.coeffs() *= scale;
  return result;
}

DualQuaternion DualQuaternionOperation::CalcInverseQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  double sqrLen0 = dualQuat.qReal.squaredNorm();
  double sqrLenE = (2.0) * (dualQuat.qReal.coeffs().dot(dualQuat.qDual.coeffs()));

  if (sqrLen0 > 0.0)
  {
    double invSqrLen0 = (1.0) / (sqrLen0);
    double invSqrLenE = ((-1.0) * (sqrLenE)) / ((sqrLen0) * (sqrLen0));
    result.qReal.coeffs() = (invSqrLen0) * ((quatOper_.GetConjugateQuaternion(dualQuat.qReal)).coeffs());
    result.qDual.coeffs() = (invSqrLen0) * ((quatOper_.GetConjugateQuaternion(dualQuat.qDual)).coeffs()) +
                            (invSqrLenE) * ((quatOper_.GetConjugateQuaternion(dualQuat.qReal)).coeffs());
  }
  else
  {
    result.qReal.x() = 0.0;
    result.qReal.y() = 0.0;
    result.qReal.z() = 0.0;
    result.qReal.w() = 0.0;
    result.qDual.x() = 0.0;
    result.qDual.y() = 0.0;
    result.qDual.z() = 0.0;
    result.qDual.w() = 0.0;
  }

  return result;
}

DualQuaternion DualQuaternionOperation::CalcNormalizeQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  double length = dualQuat.qReal.norm();
  double lengthSqr = dualQuat.qReal.squaredNorm();
  result.qReal.coeffs() = (dualQuat.qReal.coeffs()) / (length);
  result.qDual.coeffs() = (dualQuat.qDual.coeffs()) / (length);
  result.qDual.coeffs() -=
      (dualQuat.qReal.coeffs().dot(dualQuat.qDual.coeffs()) * (lengthSqr)) * (dualQuat.qReal.coeffs());
  return result;
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------
DualQuaternion DualQuaternionOperation::GetAttPos2DualQuaternion(Quaterniond qAtt, Vector3d pos)
{
  return CalcAttPos2Quaternion(qAtt, pos);
}

QuatPos DualQuaternionOperation::GetDualQuaternion2AttPos(DualQuaternion dualQuat)
{
  return CalcDualQuaternion2AttPos(dualQuat);
}

DualQuaternion DualQuaternionOperation::GetConjugateDualQuaternion(DualQuaternion dualQuat)
{
  return CalcConjugateDualQuaternion(dualQuat);
}
DualQuaternion DualQuaternionOperation::GetMultiplyDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  return CalcMultiplyDualQuaternions(dualQuat1, dualQuat2);
}

DualQuaternion DualQuaternionOperation::GetAddDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  return CalcAddDualQuaternions(dualQuat1, dualQuat2);
}

DualQuaternion DualQuaternionOperation::GetSubDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  return CalcSubDualQuaternions(dualQuat1, dualQuat2);
}

DualQuaternion DualQuaternionOperation::GetPureRotationDualQuaternion(DualQuaternion dualQuat)
{
  return CalcPureRotationDualQuaternion(dualQuat);
}

DualQuaternion DualQuaternionOperation::GetExponentialDualQuaternion(DualQuaternion dualQuat)
{
  return CalcExponentialDualQuaternion(dualQuat);
}

DualQuaternion DualQuaternionOperation::GetLogDualQuaternion(DualQuaternion dualQuat)
{
  return CalcLogDualQuaternion(dualQuat);
}

DualQuaternion DualQuaternionOperation::GetInverseDualQuaternion(DualQuaternion dualQuat)
{
  return CalcInverseQuaternion(dualQuat);
}

DualQuaternion DualQuaternionOperation::GetNormalizeQuaternion(DualQuaternion dualQuat)
{
  return CalcNormalizeQuaternion(dualQuat);
}
// -------------------------------------------------------------------------------------------------------------------------------------------------------