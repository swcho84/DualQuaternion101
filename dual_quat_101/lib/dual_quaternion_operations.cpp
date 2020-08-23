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

// calculating the dual quaternion from the quaternion-based attitude and the 3D position, NED frame, 3-2-1 frame
DualQuaternion DualQuaternionOperation::CalcAttPos2DualQuaternion(Quaterniond qAtt, Vector3d pos)
{
  DualQuaternion result;
  Quaterniond qPos;  // virtual quaternion using the NED order position
  qPos.x() = pos(0);
  qPos.y() = pos(1);
  qPos.z() = pos(2);
  qPos.w() = 0.0;
  Quaterniond qAttNormalized;
  qAttNormalized = qAtt.normalized();
  result.qReal = qAttNormalized;
  result.qDual = quatOper_.GetScalarMultiplyQuaternion(0.5, quatOper_.GetMultiplyQuaternions(qPos, qAttNormalized));
  return result;
}

// calculating the quaternion-based attitude and the 3D position, NED frame, 3-2-1 frame from the dual quaternion
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

// calculating the dual quaternion from the axis-angle representation of the quaternion and the 3D position, NED frame,
// 3-2-1 frame (normalized)
DualQuaternion DualQuaternionOperation::CalcAxisAnglePos2DualQuaternion(Vector4d axisAngle, Vector3d pos)
{
  DualQuaternion result;
  Vector3d axis;
  Vector3d s0;
  axis(0) = axisAngle(0);
  axis(1) = axisAngle(1);
  axis(2) = axisAngle(2);
  s0 = quatOper_.GetVec3dCross(pos, axis);
  result = CalcDualQuaternionUsingRotPlucker(axisAngle(3), axis, s0);
  return result;
}

// calculating the dual quaternion using plucker coordinate
DualQuaternion DualQuaternionOperation::CalcDualQuaternionUsingRotPlucker(double theta, Vector3d axis, Vector3d s0)
{
  DualQuaternion result;

  if ((fabs((quatOper_.GetVec3dDot(axis, axis)) - 1.0) < PRECISION) ||
      (fabs(quatOper_.GetVec3dDot(axis, s0)) < PRECISION))
  {
    ROS_ERROR("please check the axis-angle status..in CalcDualQuaternionUsingRotPlucker");
  }

  double sthalf = sin((0.5) * (theta));
  double cthalf = sin((0.5) * (theta));
  result.qReal.w() = cthalf;
  result.qReal.x() = (sthalf) * (axis(0));
  result.qReal.y() = (sthalf) * (axis(1));
  result.qReal.z() = (sthalf) * (axis(2));
  result.qDual.w() = 0.0;
  result.qDual.x() = (sthalf) * (s0(0));
  result.qDual.y() = (sthalf) * (s0(1));
  result.qDual.z() = (sthalf) * (s0(2));
  return result;
}

// calculating the dual quaternion from line information
DualQuaternion DualQuaternionOperation::CalcLineInfo2DualQuaternion(Vector3d vecDir, Vector3d vecPosOnLine)
{
  DualQuaternion result;
  Vector3d s0;
  s0 = quatOper_.GetVec3dCross(vecPosOnLine, vecDir);
  result = CalcDualQuaternionUsingLinePlucker(vecDir, s0);
  return result;
}

// calculating the dual quaternion from line information
DualQuaternion DualQuaternionOperation::CalcDualQuaternionUsingLinePlucker(Vector3d vecDir, Vector3d s0)
{
  DualQuaternion result;

  if ((fabs((quatOper_.GetVec3dDot(vecDir, vecDir)) - 1.0) < PRECISION) ||
      (fabs(quatOper_.GetVec3dDot(vecDir, s0)) < PRECISION))
  {
    ROS_ERROR("please check the axis-angle status..in CalcDualQuaternionUsingLinePlucker");
  }

  result.qReal.w() = 0.0;
  result.qReal.x() = vecDir(0);
  result.qReal.y() = vecDir(1);
  result.qReal.z() = vecDir(2);
  result.qDual.w() = 0.0;
  result.qDual.x() = s0(0);
  result.qDual.y() = s0(1);
  result.qDual.z() = s0(2);
  return result;
}

// calculating the dual quaternion from plane information
DualQuaternion DualQuaternionOperation::CalcPlaneInfo2DualQuaternion(Vector3d vecNormalDir, double distBtwOrigin)
{
  DualQuaternion result;
  result.qReal.w() = 0.0;
  result.qReal.x() = vecNormalDir(0);
  result.qReal.y() = vecNormalDir(1);
  result.qReal.z() = vecNormalDir(2);
  result.qDual.w() = distBtwOrigin;
  result.qDual.x() = 0.0;
  result.qDual.y() = 0.0;
  result.qDual.z() = 0.0;
  return result;
}

// calculating the dual quaternion from the quaternion, pure rotation
DualQuaternion DualQuaternionOperation::CalcDualQuaternionFromPureRotation(Quaterniond q)
{
  DualQuaternion result;
  result.qReal = q.normalized();
  result.qDual.x() = 0.0;
  result.qDual.y() = 0.0;
  result.qDual.z() = 0.0;
  result.qDual.w() = 0.0;
  return result;
}

// calculating the dual quaternion from the position or translational vector
DualQuaternion DualQuaternionOperation::CalcDualQuaternionFromPurePosTrn(Vector3d u, int nCase)
{
  DualQuaternion result;
  switch (nCase)
  {
    default:
    case (POSITION):
    {
      result.qReal.x() = 0.0;
      result.qReal.y() = 0.0;
      result.qReal.z() = 0.0;
      result.qReal.w() = 1.0;
      result.qDual.x() = u(0);
      result.qDual.y() = u(1);
      result.qDual.z() = u(2);
      result.qDual.w() = 0.0;
      break;
    }
    case (TRNVEC):
    {
      result.qReal.x() = 0.0;
      result.qReal.y() = 0.0;
      result.qReal.z() = 0.0;
      result.qReal.w() = 1.0;
      result.qDual.x() = (0.5) * (u(0));
      result.qDual.y() = (0.5) * (u(1));
      result.qDual.z() = (0.5) * (u(2));
      result.qDual.w() = 0.0;
      break;
    }
  }
  return result;
}

// calculating the dual quaternion from the homogeneous matrix
DualQuaternion DualQuaternionOperation::CalcDualQuaternionFromMatHomo(MatHomoGen matHomoP)
{
  DualQuaternion result;
  RotTrnInfo rotTrnInfo;
  rotTrnInfo = quatOper_.GetRotTranInfoFromMatHomo(matHomoP);
  Quaterniond qAtt(rotTrnInfo.mRot);
  DualQuaternion dualQatt;
  dualQatt = CalcDualQuaternionFromPureRotation(qAtt);
  DualQuaternion dualQpos;
  dualQpos = CalcDualQuaternionFromPurePosTrn(rotTrnInfo.mTrn, TRNVEC);
  result = CalcMultiplyDualQuaternions(dualQpos, dualQatt);
  return result;
}

// calculating the homogeneous matrix from the dual quaternion
MatHomoGen DualQuaternionOperation::CalcMatHomoFromDualQuaternion(DualQuaternion dualQuat)
{
  MatHomoGen result;
  QuatPos quatPosRes;
  quatPosRes = CalcDualQuaternion2AttPos(dualQuat);
  RotTrnInfo rotTrnInfoRes;
  rotTrnInfoRes.mRot = quatOper_.GetQuaternionRotationMatrix(quatPosRes.qAtt);
  rotTrnInfoRes.mTrn = quatPosRes.pos;
  result = quatOper_.GetMatHomoFromRotTranInfo(rotTrnInfoRes);
  return result;
}

// calculating the conjugate dual quaternion
DualQuaternion DualQuaternionOperation::CalcConjugateDualQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetConjugateQuaternion(dualQuat.qReal);
  result.qDual = quatOper_.GetConjugateQuaternion(dualQuat.qDual);
  return result;
}

// calculating the multiplication of dual quaternions
DualQuaternion DualQuaternionOperation::CalcMultiplyDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetMultiplyQuaternions(dualQuat1.qReal, dualQuat2.qReal);
  result.qDual = quatOper_.GetAddQuaternions(quatOper_.GetMultiplyQuaternions(dualQuat1.qReal, dualQuat2.qDual),
                                             quatOper_.GetMultiplyQuaternions(dualQuat1.qDual, dualQuat2.qReal));
  return result;
}

// calculating the addition of dual quaternions
DualQuaternion DualQuaternionOperation::CalcAddDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetAddQuaternions(dualQuat1.qReal, dualQuat2.qReal);
  result.qDual = quatOper_.GetAddQuaternions(dualQuat1.qDual, dualQuat2.qDual);
  return result;
}

// calculating the subtraction of dual quaternions
DualQuaternion DualQuaternionOperation::CalcSubDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetSubQuaternions(dualQuat1.qReal, dualQuat2.qReal);
  result.qDual = quatOper_.GetSubQuaternions(dualQuat1.qDual, dualQuat2.qDual);
  return result;
}

// calculating the exponential dual quaternion
DualQuaternion DualQuaternionOperation::CalcExponentialDualQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetExpQuaternion(dualQuat.qReal);
  result.qDual = quatOper_.GetMultiplyQuaternions(result.qReal, dualQuat.qDual);
  return result;
}

// calculating the log dual quaternion
DualQuaternion DualQuaternionOperation::CalcLogDualQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  result.qReal = quatOper_.GetLogQuaternion(dualQuat.qReal);
  result.qDual = quatOper_.GetMultiplyQuaternions(quatOper_.GetConjugateQuaternion(dualQuat.qReal), dualQuat.qDual);
  double scale = (1.0) / (dualQuat.qReal.squaredNorm());
  result.qDual.coeffs() *= scale;
  return result;
}

// calculating the inverse dual quaternion
DualQuaternion DualQuaternionOperation::CalcInverseDualQuaternion(DualQuaternion dualQuat)
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

// calculating the norms of dual quaternion
Vector2d DualQuaternionOperation::CalcNormsDualQuaternion(DualQuaternion dualQuat)
{
  // (0): real part norm, (1): dual part norm
  Vector2d result;
  result(0) = dualQuat.qReal.norm();
  result(1) = (2.0) * (dualQuat.qReal.coeffs().dot(dualQuat.qDual.coeffs()));
  return result;
}

// calculating the normalized dual quaternion
DualQuaternion DualQuaternionOperation::CalcNormalizeDualQuaternion(DualQuaternion dualQuat)
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

// calculating the sign conversion of dual quaternion
DualQuaternion DualQuaternionOperation::CalcSgnDualQuaternion(DualQuaternion dualQuat)
{
  DualQuaternion result;
  result.qReal.x() = (-1.0) * (dualQuat.qReal.x());
  result.qReal.y() = (-1.0) * (dualQuat.qReal.y());
  result.qReal.z() = (-1.0) * (dualQuat.qReal.z());
  result.qReal.w() = (-1.0) * (dualQuat.qReal.w());
  result.qDual.x() = (-1.0) * (dualQuat.qDual.x());
  result.qDual.y() = (-1.0) * (dualQuat.qDual.y());
  result.qDual.z() = (-1.0) * (dualQuat.qDual.z());
  result.qDual.w() = (-1.0) * (dualQuat.qDual.w());
  return result;
}

// calculating Clifford conjugation transformation of type f1g (Alba Perez notation).
DualQuaternion DualQuaternionOperation::CalcConjF1gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion ABA;
  ABA = CalcMultiplyDualQuaternions(dualQuat1, dualQuat2);
  DualQuaternion result;
  result = CalcMultiplyDualQuaternions(ABA, dualQuat1);
  return result;
}

// calculating Clifford conjugation transformation of type f2g (Alba Perez notation).
DualQuaternion DualQuaternionOperation::CalcConjF2gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion ABA;
  ABA = CalcMultiplyDualQuaternions(dualQuat1, dualQuat2);
  DualQuaternion Astar;
  Astar = CalcConjugateDualQuaternion(dualQuat1);
  DualQuaternion result;
  result = CalcMultiplyDualQuaternions(ABA, Astar);
  return result;
}

// calculating Clifford conjugation transformation of type f3g (Alba Perez notation).
DualQuaternion DualQuaternionOperation::CalcConjF3gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion ABA;
  ABA = CalcMultiplyDualQuaternions(dualQuat1, dualQuat2);
  DualQuaternion Astar;
  Astar.qReal.w() = dualQuat1.qReal.w();
  Astar.qReal.x() = dualQuat1.qReal.x();
  Astar.qReal.y() = dualQuat1.qReal.y();
  Astar.qReal.z() = dualQuat1.qReal.z();
  Astar.qDual.w() = (-1.0) * (dualQuat1.qDual.w());
  Astar.qDual.x() = (-1.0) * (dualQuat1.qDual.x());
  Astar.qDual.y() = (-1.0) * (dualQuat1.qDual.y());
  Astar.qDual.z() = (-1.0) * (dualQuat1.qDual.z());
  DualQuaternion result;
  result = CalcMultiplyDualQuaternions(ABA, Astar);
  return result;
}

// calculating Clifford conjugation transformation of type f4g (Alba Perez notation).
DualQuaternion DualQuaternionOperation::CalcConjF4gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  DualQuaternion ABA;
  ABA = CalcMultiplyDualQuaternions(dualQuat1, dualQuat2);
  DualQuaternion Astar;
  Astar.qReal.w() = dualQuat1.qReal.w();
  Astar.qReal.x() = (-1.0) * (dualQuat1.qReal.x());
  Astar.qReal.y() = (-1.0) * (dualQuat1.qReal.y());
  Astar.qReal.z() = (-1.0) * (dualQuat1.qReal.z());
  Astar.qDual.w() = (-1.0) * (dualQuat1.qDual.w());
  Astar.qDual.x() = dualQuat1.qDual.x();
  Astar.qDual.y() = dualQuat1.qDual.y();
  Astar.qDual.z() = dualQuat1.qDual.z();
  DualQuaternion result;
  result = CalcMultiplyDualQuaternions(ABA, Astar);
  return result;
}

// calculating the equality between dual quaternions
int DualQuaternionOperation::CalcDualQuaternionEqual(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  int result1 = 0;
  if (fabs((dualQuat1.qReal.x()) - (dualQuat2.qReal.x())) > PRECISION)
    result1 += 1;
  if (fabs((dualQuat1.qReal.y()) - (dualQuat2.qReal.y())) > PRECISION)
    result1 += 1;
  if (fabs((dualQuat1.qReal.z()) - (dualQuat2.qReal.z())) > PRECISION)
    result1 += 1;
  if (fabs((dualQuat1.qReal.w()) - (dualQuat2.qReal.w())) > PRECISION)
    result1 += 1;
  if (fabs((dualQuat1.qDual.x()) - (dualQuat2.qDual.x())) > PRECISION)
    result1 += 1;
  if (fabs((dualQuat1.qDual.y()) - (dualQuat2.qDual.y())) > PRECISION)
    result1 += 1;
  if (fabs((dualQuat1.qDual.z()) - (dualQuat2.qDual.z())) > PRECISION)
    result1 += 1;
  if (fabs((dualQuat1.qDual.w()) - (dualQuat2.qDual.w())) > PRECISION)
    result1 += 1;

  int result2 = 0;
  if (fabs((dualQuat1.qReal.x()) + (dualQuat2.qReal.x())) > PRECISION)
    result2 += 1;
  if (fabs((dualQuat1.qReal.y()) + (dualQuat2.qReal.y())) > PRECISION)
    result2 += 1;
  if (fabs((dualQuat1.qReal.z()) + (dualQuat2.qReal.z())) > PRECISION)
    result2 += 1;
  if (fabs((dualQuat1.qReal.w()) + (dualQuat2.qReal.w())) > PRECISION)
    result2 += 1;
  if (fabs((dualQuat1.qDual.x()) + (dualQuat2.qDual.x())) > PRECISION)
    result2 += 1;
  if (fabs((dualQuat1.qDual.y()) + (dualQuat2.qDual.y())) > PRECISION)
    result2 += 1;
  if (fabs((dualQuat1.qDual.z()) + (dualQuat2.qDual.z())) > PRECISION)
    result2 += 1;
  if (fabs((dualQuat1.qDual.w()) + (dualQuat2.qDual.w())) > PRECISION)
    result2 += 1;

  return std::min(result1, result2);
}

// calculating the equality between dual quaternions with the threshold
int DualQuaternionOperation::CalcDualQuaternionEqualThreshold(DualQuaternion dualQuat1, DualQuaternion dualQuat2,
                                                              double threshold)
{
  int result1 = 0;
  if (fabs((dualQuat1.qReal.x()) - (dualQuat2.qReal.x())) > threshold)
    result1 += 1;
  if (fabs((dualQuat1.qReal.y()) - (dualQuat2.qReal.y())) > threshold)
    result1 += 1;
  if (fabs((dualQuat1.qReal.z()) - (dualQuat2.qReal.z())) > threshold)
    result1 += 1;
  if (fabs((dualQuat1.qReal.w()) - (dualQuat2.qReal.w())) > threshold)
    result1 += 1;
  if (fabs((dualQuat1.qDual.x()) - (dualQuat2.qDual.x())) > threshold)
    result1 += 1;
  if (fabs((dualQuat1.qDual.y()) - (dualQuat2.qDual.y())) > threshold)
    result1 += 1;
  if (fabs((dualQuat1.qDual.z()) - (dualQuat2.qDual.z())) > threshold)
    result1 += 1;
  if (fabs((dualQuat1.qDual.w()) - (dualQuat2.qDual.w())) > threshold)
    result1 += 1;

  int result2 = 0;
  if (fabs((dualQuat1.qReal.x()) + (dualQuat2.qReal.x())) > threshold)
    result2 += 1;
  if (fabs((dualQuat1.qReal.y()) + (dualQuat2.qReal.y())) > threshold)
    result2 += 1;
  if (fabs((dualQuat1.qReal.z()) + (dualQuat2.qReal.z())) > threshold)
    result2 += 1;
  if (fabs((dualQuat1.qReal.w()) + (dualQuat2.qReal.w())) > threshold)
    result2 += 1;
  if (fabs((dualQuat1.qDual.x()) + (dualQuat2.qDual.x())) > threshold)
    result2 += 1;
  if (fabs((dualQuat1.qDual.y()) + (dualQuat2.qDual.y())) > threshold)
    result2 += 1;
  if (fabs((dualQuat1.qDual.z()) + (dualQuat2.qDual.z())) > threshold)
    result2 += 1;
  if (fabs((dualQuat1.qDual.w()) + (dualQuat2.qDual.w())) > threshold)
    result2 += 1;

  return std::min(result1, result2);
}

// calculating the unit check of the dual quaternion
int DualQuaternionOperation::CalcDualQuaternionUnit(DualQuaternion dualQuat)
{
  int result = 0;
  Vector2d vecDualQuatNorm;
  vecDualQuatNorm = CalcNormsDualQuaternion(dualQuat);
  if ((fabs(vecDualQuatNorm(0) - 1.0) > PRECISION) || (fabs(vecDualQuatNorm(1) - 0.0) > PRECISION))
    result = 0;
  else
    result = 1;
  return result;
}

// calculating the point on the plane (dualQuat2-position is on the plane of dualQuat1, 1 if point Q is on plane P)
int DualQuaternionOperation::CalcDualQuaternionOnPlane(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  int result = 0;
  double dOnPlane = (dualQuat1.qReal.x()) * (dualQuat2.qDual.x()) + (dualQuat1.qReal.y()) * (dualQuat2.qDual.y()) +
                    (dualQuat1.qReal.z()) * (dualQuat2.qDual.z()) - (dualQuat1.qDual.w());
  if ((fabs(dOnPlane)) < PRECISION)
    result = 1;
  else
    result = 0;
  return result;
}

// -------------------------------------------------------------------------------------------------------------------------------------------------------
DualQuaternion DualQuaternionOperation::GetAttPos2DualQuaternion(Quaterniond qAtt, Vector3d pos)
{
  return CalcAttPos2DualQuaternion(qAtt, pos);
}

QuatPos DualQuaternionOperation::GetDualQuaternion2AttPos(DualQuaternion dualQuat)
{
  return CalcDualQuaternion2AttPos(dualQuat);
}

DualQuaternion DualQuaternionOperation::GetAxisAnglePos2DualQuaternion(Vector4d axisAngle, Vector3d pos)
{
  return CalcAxisAnglePos2DualQuaternion(axisAngle, pos);
}

DualQuaternion DualQuaternionOperation::GetLineInfo2DualQuaternion(Vector3d vecDir, Vector3d vecPosOnLine)
{
  return CalcLineInfo2DualQuaternion(vecDir, vecPosOnLine);
}

DualQuaternion DualQuaternionOperation::GetDualQuaternionFromPureRotation(Quaterniond q)
{
  return CalcDualQuaternionFromPureRotation(q);
}

DualQuaternion DualQuaternionOperation::GetDualQuaternionFromPurePosTrn(Vector3d u, int nCase)
{
  return CalcDualQuaternionFromPurePosTrn(u, nCase);
}

DualQuaternion DualQuaternionOperation::GetDualQuaternionFromMatHomo(MatHomoGen matHomoP)
{
  return CalcDualQuaternionFromMatHomo(matHomoP);
}

MatHomoGen DualQuaternionOperation::GetMatHomoFromDualQuaternion(DualQuaternion dualQuat)
{
  return CalcMatHomoFromDualQuaternion(dualQuat);
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
  return CalcInverseDualQuaternion(dualQuat);
}

Vector2d DualQuaternionOperation::GetNormsDualQuaternion(DualQuaternion dualQuat)
{
  return CalcNormsDualQuaternion(dualQuat);
}

DualQuaternion DualQuaternionOperation::GetNormalizeDualQuaternion(DualQuaternion dualQuat)
{
  return CalcNormalizeDualQuaternion(dualQuat);
}

DualQuaternion DualQuaternionOperation::GetSgnDualQuaternion(DualQuaternion dualQuat)
{
  return CalcSgnDualQuaternion(dualQuat);
}

DualQuaternion DualQuaternionOperation::GetConjF1gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  return CalcConjF1gDualQuaternions(dualQuat1, dualQuat2);
}

DualQuaternion DualQuaternionOperation::GetConjF2gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  return CalcConjF2gDualQuaternions(dualQuat1, dualQuat2);
}

DualQuaternion DualQuaternionOperation::GetConjF3gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  return CalcConjF3gDualQuaternions(dualQuat1, dualQuat2);
}

DualQuaternion DualQuaternionOperation::GetConjF4gDualQuaternions(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  return CalcConjF4gDualQuaternions(dualQuat1, dualQuat2);
}

DualQuaternion DualQuaternionOperation::GetPlaneInfo2DualQuaternion(Vector3d vecNormalDir, double distBtwOrigin)
{
  return CalcPlaneInfo2DualQuaternion(vecNormalDir, distBtwOrigin);
}

int DualQuaternionOperation::GetDualQuaternionEqual(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  return CalcDualQuaternionEqual(dualQuat1, dualQuat2);
}

int DualQuaternionOperation::GetDualQuaternionEqualThreshold(DualQuaternion dualQuat1, DualQuaternion dualQuat2,
                                                             double threshold)
{
  return CalcDualQuaternionEqualThreshold(dualQuat1, dualQuat2, threshold);
}

int DualQuaternionOperation::GetDualQuaternionUnit(DualQuaternion dualQuat)
{
  return CalcDualQuaternionUnit(dualQuat);
}

int DualQuaternionOperation::GetDualQuaternionOnPlane(DualQuaternion dualQuat1, DualQuaternion dualQuat2)
{
  return CalcDualQuaternionOnPlane(dualQuat1, dualQuat2);
}
// -------------------------------------------------------------------------------------------------------------------------------------------------------