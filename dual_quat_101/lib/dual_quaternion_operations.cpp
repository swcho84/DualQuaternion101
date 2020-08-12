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

DualQuaternion DualQuaternionOperation::GetAttPos2DualQuaternion(Quaterniond qAtt, Vector3d pos)
{
  return CalcAttPos2Quaternion(qAtt, pos);
}

QuatPos DualQuaternionOperation::GetDualQuaternion2AttPos(DualQuaternion dualQuat)
{
  return CalcDualQuaternion2AttPos(dualQuat);
}