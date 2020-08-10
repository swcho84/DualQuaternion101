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

private:
  QuaternionOperation quatOper_;

  DualQuaternion CalcAttPos2Quaternion(Quaterniond qAtt, Vector3d pos);
  QuatPos CalcDualQuaternion2AttPos(DualQuaternion dualQuat);
};

#endif  // DUAL_QUAT_101_DUAL_QUATERNION_OPERATIONS_H