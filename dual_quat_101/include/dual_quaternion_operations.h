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
  DualQuaternion GetLineInfo2DualQuaternion(Vector3d vecDir, Vector3d vecPosOnLine);
  DualQuaternion GetPlaneInfo2DualQuaternion(Vector3d vecNormalDir, double distBtwOrigin);
  DualQuaternion GetDualQuaternionFromPureRotation(Quaterniond q);
  DualQuaternion GetDualQuaternionFromPurePosTrn(Vector3d u, int nCase);
  DualQuaternion GetDualQuaternionFromMatHomo(MatHomoGen matHomoP);
  MatHomoGen GetMatHomoFromDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetConjugateDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetMultiplyDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetAddDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetSubDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetExponentialDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetLogDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetInverseDualQuaternion(DualQuaternion dualQuat);
  Vector2d GetNormsDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetNormalizeDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetSgnDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion GetConjF1gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetConjF2gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetConjF3gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion GetConjF4gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  int GetDualQuaternionEqual(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  int GetDualQuaternionEqualThreshold(DualQuaternion dualQuat1, DualQuaternion dualQuat2, double threshold);
  int GetDualQuaternionUnit(DualQuaternion dualQuat);
  int GetDualQuaternionOnPlane(DualQuaternion dualQuat1, DualQuaternion dualQuat2);

private:
  QuaternionOperation quatOper_;

  DualQuaternion CalcAttPos2DualQuaternion(Quaterniond qAtt, Vector3d pos);
  QuatPos CalcDualQuaternion2AttPos(DualQuaternion dualQuat);
  DualQuaternion CalcAxisAnglePos2DualQuaternion(Vector4d axisAngle, Vector3d pos);
  DualQuaternion CalcDualQuaternionUsingRotPlucker(double theta, Vector3d axis, Vector3d s0);
  DualQuaternion CalcLineInfo2DualQuaternion(Vector3d vecDir, Vector3d vecPosOnLine);
  DualQuaternion CalcDualQuaternionUsingLinePlucker(Vector3d vecDir, Vector3d s0);
  DualQuaternion CalcPlaneInfo2DualQuaternion(Vector3d vecNormalDir, double distBtwOrigin);
  DualQuaternion CalcDualQuaternionFromPureRotation(Quaterniond q);
  DualQuaternion CalcDualQuaternionFromPurePosTrn(Vector3d u, int nCase);
  DualQuaternion CalcDualQuaternionFromMatHomo(MatHomoGen matHomoP);
  MatHomoGen CalcMatHomoFromDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcConjugateDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcMultiplyDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcAddDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcSubDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcExponentialDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcLogDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcInverseDualQuaternion(DualQuaternion dualQuat);
  Vector2d CalcNormsDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcNormalizeDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcSgnDualQuaternion(DualQuaternion dualQuat);
  DualQuaternion CalcConjF1gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcConjF2gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcConjF3gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  DualQuaternion CalcConjF4gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  int CalcDualQuaternionEqual(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
  int CalcDualQuaternionEqualThreshold(DualQuaternion dualQuat1, DualQuaternion dualQuat2, double threshold);
  int CalcDualQuaternionUnit(DualQuaternion dualQuat);
  int CalcDualQuaternionOnPlane(DualQuaternion dualQuat1, DualQuaternion dualQuat2);
};

#endif  // DUAL_QUAT_101_DUAL_QUATERNION_OPERATIONS_H