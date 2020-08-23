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

  void MainLoop();

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
  Vector3d GetEulerNed(Vector3d euler);
  Quaterniond GetQuaternionFromYPREulerAng(Vector3d euler);
  Vector4d GetAxisAngFromQuaternion(Quaterniond q);
  Vector3d GetAxisMagFromQuaternion(Vector4d axisAng);

  double GetVec3dDot(Vector3d u, Vector3d v);
  Vector3d GetVec3dCross(Vector3d u, Vector3d v);
  Vector3d GetVec3dAdd(Vector3d u, Vector3d v);
  Vector3d GetVec3dSub(Vector3d u, Vector3d v);
  Vector3d GetVec3dSgn(Vector3d u);
  double GetVec3dNorm(Vector3d u);
  Vector3d GetVec3dNormalize(Vector3d u);
  double GetVec3dDist(Vector3d u, Vector3d v);
  int GetVec3dEqual(Vector3d u, Vector3d v);
  int GetVec3dEqualThreshold(Vector3d u, Vector3d v, double threshold);

  Matrix3d GetMat3dIdentity();
  double GetMat3dDeterminant(Matrix3d matP);
  Matrix3d GetMat3dAdd(Matrix3d matP, Matrix3d matQ);
  Matrix3d GetMat3dSub(Matrix3d matP, Matrix3d matQ);
  Matrix3d GetMat3dMultiply(Matrix3d matP, Matrix3d matQ);
  Vector3d GetMat3dVec3dMultiply(Matrix3d matP, Vector3d u);
  Matrix3d GetMat3dInv(Matrix3d matP);
  Vector3d GetMat3dVec3dSolve(Matrix3d matA, Vector3d vecB);
  int GetMat3dEqual(Matrix3d matP, Matrix3d matQ);
  int GetMat3dEqualThreshold(Matrix3d matP, Matrix3d matQ, double threshold);
  void GetMat3dPrint(Matrix3d matP);
  void GetMat3dsPrint(Matrix3d matP, Matrix3d matQ);

  MatHomoGen GetMatHomoFromRotTranInfo(RotTrnInfo rotTrnInfo);
  RotTrnInfo GetRotTranInfoFromMatHomo(MatHomoGen matHomoP);
  MatHomoGen GetMatHomoMultiply(MatHomoGen matHomoP, MatHomoGen matHomoQ);
  VecHomoGen GetMatHomoVecHomoMultiply(MatHomoGen matHomoP, VecHomoGen vecHomoU);
  int GetMatHomoEqual(MatHomoGen matHomoP, MatHomoGen matHomoQ);
  int GetMatHomoEqualThreshold(MatHomoGen matHomoP, MatHomoGen matHomoQ, double threshold);
  void GetMatHomoPrint(MatHomoGen matHomoP);
  void GetMatHomosPrint(MatHomoGen matHomoP, MatHomoGen matHomoQ);

  Matrix3d CalcDcmNtoB(Vector3d eulerAtt);
  Matrix3d CalcDcmEuler321(Vector3d eulerAtt);
  Vector3d ConvertPosFromEnuToNed(Vector3d posEnu);

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

  double CalcVec3dDot(Vector3d u, Vector3d v);
  Vector3d CalcVec3dCross(Vector3d u, Vector3d v);
  Vector3d CalcVec3dAdd(Vector3d u, Vector3d v);
  Vector3d CalcVec3dSub(Vector3d u, Vector3d v);
  Vector3d CalcVec3dSgn(Vector3d u);
  double CalcVec3dNorm(Vector3d u);
  Vector3d CalcVec3dNormalize(Vector3d u);
  double CalcVec3dDist(Vector3d u, Vector3d v);
  int CalcVec3dEqual(Vector3d u, Vector3d v);
  int CalcVec3dEqualThreshold(Vector3d u, Vector3d v, double threshold);

  Matrix3d CalcMat3dIdentity();
  double CalcMat3dDeterminant(Matrix3d matP);
  Matrix3d CalcMat3dAdd(Matrix3d matP, Matrix3d matQ);
  Matrix3d CalcMat3dSub(Matrix3d matP, Matrix3d matQ);
  Matrix3d CalcMat3dMultiply(Matrix3d matP, Matrix3d matQ);
  Vector3d CalcMat3dVec3dMultiply(Matrix3d matP, Vector3d u);
  Matrix3d CalcMat3dInv(Matrix3d matP);
  Vector3d CalcMat3dVec3dSolve(Matrix3d matA, Vector3d vecB);
  int CalcMat3dEqual(Matrix3d matP, Matrix3d matQ);
  int CalcMat3dEqualThreshold(Matrix3d matP, Matrix3d matQ, double threshold);

  MatHomoGen CalcMatHomoFromRotTranInfo(RotTrnInfo rotTrnInfo);
  RotTrnInfo CalcRotTranInfoFromMatHomo(MatHomoGen matHomoP);
  MatHomoGen CalcMatHomoMultiply(MatHomoGen matHomoP, MatHomoGen matHomoQ);
  VecHomoGen CalcMatHomoVecHomoMultiply(MatHomoGen matHomoP, VecHomoGen vecHomoU);
  int CalcMatHomoEqual(MatHomoGen matHomoP, MatHomoGen matHomoQ);
  int CalcMatHomoEqualThreshold(MatHomoGen matHomoP, MatHomoGen matHomoQ, double threshold);
};

#endif  // DUAL_QUAT_101_QUATERNION_OPERATIONS_H