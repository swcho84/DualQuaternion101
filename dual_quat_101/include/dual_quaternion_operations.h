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
  DualQuaternion GetConjugateDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetMultiplyDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetAddDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetSubDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetPureRotationDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetExponentialDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetLogDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetInverseDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetNormalizeQuaternion(DualQuaternion dualQuat);

private:
  QuaternionOperation quatOper_;

  DualQuaternion CalcAttPos2Quaternion(Quaterniond qAtt, Vector3d pos);
  QuatPos CalcDualQuaternion2AttPos(DualQuaternion dualQuat);
  DualQuaternion CalcConjugateDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcMultiplyDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcAddDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcSubDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcPureRotationDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcExponentialDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcLogDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcInverseQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcNormalizeQuaternion(DualQuaternion dualQuat);
};

#endif  // DUAL_QUAT_101_DUAL_QUATERNION_OPERATIONS_H