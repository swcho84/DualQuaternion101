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
// -------------------------------------------------------------------------------------------------------------------------------------------------------