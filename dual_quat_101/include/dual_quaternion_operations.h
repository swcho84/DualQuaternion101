#ifndef DUAL_QUAT_101_DUAL_QUATERNION_OPERATIONS_H
#define DUAL_QUAT_101_DUAL_QUATERNION_OPERATIONS_H

// for using common headers for quaternion
#include "global_header.h"

// for using quaternion operation
#include "quaternion_operations.h"

using namespace std;
using namespace ros;
using namespace Eigen;

class DualQuaternionOperation
{
public:
  DualQuaternionOperation();
  ~DualQuaternionOperation();

  void MainLoop();

  DualQuaternion GetAttPos2DualQuaternion(Quaterniond qAtt, Vector3d pos);
  QuatPos GetDualQuaternion2AttPos(DualQuaternion dualQuat);
  DualQuaternion GetAxisAnglePos2DualQuaternion(Vector4d axisAngle, Vector3d pos);
  DualQuaternion GetDualQuaternionFromPureRotation(Quaterniond q);
  DualQuaternion GetDualQuaternionFromPurePosTrn(Vector3d u, int nCase);
  DualQuaternion GetDualQuaternionFromMatHomo(MatHomoGen matHomoP);
  DualQuaternion GetConjugateDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetMultiplyDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetAddDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetSubDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetExponentialDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetLogDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetInverseDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetNormalizeDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetSgnDualQuaternion(DualQuaternion dualQuat);

private:
  QuaternionOperation quatOper_;

  DualQuaternion CalcAttPos2DualQuaternion(Quaterniond qAtt, Vector3d pos);
  QuatPos CalcDualQuaternion2AttPos(DualQuaternion dualQuat);
  DualQuaternion CalcAxisAnglePos2DualQuaternion(Vector4d axisAngle, Vector3d pos);
  DualQuaternion CalcDualQuaternionUsingPlucker(double theta, Vector3d axis, Vector3d s0);
  DualQuaternion CalcDualQuaternionFromPureRotation(Quaterniond q);
  DualQuaternion CalcDualQuaternionFromPurePosTrn(Vector3d u, int nCase);
  DualQuaternion CalcDualQuaternionFromMatHomo(MatHomoGen matHomoP);
  DualQuaternion CalcConjugateDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcMultiplyDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcAddDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcSubDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcExponentialDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcLogDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcInverseQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcNormalizeDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcSgnDualQuaternion(DualQuaternion dualQuat);
};

#endif  // DUAL_QUAT_101_DUAL_QUATERNION_OPERATIONS_H