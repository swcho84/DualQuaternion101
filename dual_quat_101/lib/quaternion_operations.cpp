#include "quaternion_operations.h"

using namespace std;
using namespace ros;
using namespace Eigen;

QuaternionOperation::QuaternionOperation()
{
}

QuaternionOperation::~QuaternionOperation()
{
}

void QuaternionOperation::MainLoop()
{
}

// calculating addition w.r.t the quaternions
Quaterniond QuaternionOperation::CalcAddQuaternions(Quaterniond q1, Quaterniond q2)
{
  Quaterniond result;
  result.vec() = q1.vec() + q2.vec();
  result.w() = q1.w() + q2.w();
  return result;
}

// calculating subtraction w.r.t the quaternions
Quaterniond QuaternionOperation::CalcSubQuaternions(Quaterniond q1, Quaterniond q2)
{
  Quaterniond result;
  result.vec() = q1.vec() - q2.vec();
  result.w() = q1.w() - q2.w();
  return result;
}

// calculating the conjugate quaternion
Quaterniond QuaternionOperation::CalcConjugateQuaternion(Quaterniond q)
{
  Quaterniond result;
  result = q.conjugate();
  return result;
}

// calculating scalar multiplication of the quaternion
Quaterniond QuaternionOperation::CalcScalarMultiplyQuaternion(double scalar, Quaterniond q)
{
  Quaterniond result;
  result.x() = scalar * q.x();
  result.y() = scalar * q.y();
  result.z() = scalar * q.z();
  result.w() = scalar * q.w();
  return result;
}

// calculating the quaternion multiplication (feat. Dr. Y.J.Na)
Quaterniond QuaternionOperation::CalcMultiplyQuaternions(Quaterniond q1, Quaterniond q2)
{
  Quaterniond result;
  result.vec() = q1.w() * q2.vec() + q2.w() * q1.vec() + q1.vec().cross(q2.vec());
  result.w() = q1.w() * q2.w() - q1.vec().transpose() * q2.vec();
  return result;
}

// calculating half quaternion
Quaterniond QuaternionOperation::CalcHalfQuaternion(Quaterniond q)
{
  Quaterniond result(0.5 * q.w(), 0.5 * q.x(), 0.5 * q.y(), 0.5 * q.z());
  return result;
}

// calculating exponential quaternion
Quaterniond QuaternionOperation::CalcExpQuaternion(Quaterniond q)
{
  Quaterniond result;
  double vecNorm = q.vec().norm();
  double expqW = exp(q.w());

  if (vecNorm == 0.0)
  {
    result.x() = 0.0;
    result.y() = 0.0;
    result.z() = 0.0;
    result.w() = expqW;
  }
  else
  {
    result.w() = (expqW) * (cos(vecNorm));
    result.vec() = (expqW) * (sinc(vecNorm)) * (q.vec());
  }

  return result;
}

// calculating log quaternion
Quaterniond QuaternionOperation::CalcLogQuaternion(Quaterniond q)
{
  Quaterniond result;
  double expqW = exp(q.w());
  double logqW = log(expqW);
  double a = acos(q.w() / expqW);

  if (a == 0.0)
  {
    result.x() = 0.0;
    result.y() = 0.0;
    result.z() = 0.0;
    result.w() = logqW;
  }
  else
  {
    result.w() = logqW;
    result.vec() = (q.vec()) / (expqW) / ((sin(a)) / (a));
  }

  return result;
}

// calculating the 2-norm of the quaternion
double QuaternionOperation::CalcNormQuaternion(Quaterniond q)
{
  return sqrt(q.x() * q.x() + q.y() * q.y() + q.z() * q.z() + q.w() * q.w());
}

// calculating the attitude matrix (i.e. rotation matrix from single quaternion)
Matrix3d QuaternionOperation::CalcQuaternionRotationMatrix(Quaterniond q)
{
  Matrix3d result;
  Matrix3d Iden;
  Iden = Matrix3d::Identity();
  double q0 = q.w();
  result = (q0 * q0 - q.vec().transpose() * q.vec()) * (Iden) + (2.0) * (q.vec() * q.vec().transpose()) -
           (2.0) * (q0) * (CalcQuaternionSymmSkewMatrix(q));
  return result;
}

// calculating the skew matrix for RPY, 1-2-3 transformation
Matrix3d QuaternionOperation::CalcQuaternionSkewMatrix(Quaterniond q)
{
  Matrix3d result;
  result.setZero();
  result(0, 1) = -q.z();
  result(0, 2) = q.y();
  result(1, 0) = q.z();
  result(1, 2) = -q.x();
  result(2, 0) = -q.y();
  result(2, 1) = q.x();
  return result;
}

// calculating the skew matrix for YPR, 3-2-1 transformation
Matrix3d QuaternionOperation::CalcQuaternionSymmSkewMatrix(Quaterniond q)
{
  Matrix3d result;
  result.setZero();
  result(0, 1) = q.z();
  result(0, 2) = -q.y();
  result(1, 0) = -q.z();
  result(1, 2) = q.x();
  result(2, 0) = q.y();
  result(2, 1) = -q.x();
  return result;
}

// assigning the quaternion directly using their values
Quaterniond QuaternionOperation::CalcSingleQuaternion(double qx, double qy, double qz, double qw)
{
  Quaterniond result;
  result.vec().x() = qx;
  result.vec().y() = qy;
  result.vec().z() = qz;
  result.w() = qw;
  return result;
}

// converting the quaternion to the Euler angle(3-2-1, ZYX, YPR) [rad]
Vector3d QuaternionOperation::CalcYPREulerAngFromQuaternion(Quaterniond q)
{
  tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
  tf2::Matrix3x3 matQuat(quat);
  Vector3d result;
  double dYaw, dPitch, dRoll = 0.0;
  matQuat.getEulerYPR(dYaw, dPitch, dRoll);
  result(0) = wrapD(dRoll);
  result(1) = wrapD(dPitch);
  result(2) = wrapD(dYaw);
  return result;
}

// converting the Euler angle(3-2-1, ZYX, YPR) [rad] to the quaternion
Quaterniond QuaternionOperation::CalcQuaternionFromYPREulerAng(Vector3d euler)
{
  tf2::Matrix3x3 matQuat;
  tf2::Quaternion quat;
  Quaterniond result;
  matQuat.setEulerYPR(euler(2), euler(1), euler(0));
  matQuat.getRotation(quat);
  result.vec().x() = (double)(quat.x());
  result.vec().y() = (double)(quat.y());
  result.vec().z() = (double)(quat.z());
  result.w() = (double)(quat.w());
  return result;
}

// calculating the axis-angle from the quaternion
Vector4d QuaternionOperation::CalcAxisAngFromQuaternion(Quaterniond q)
{
  Vector4d result;
  tf2::Quaternion quat(q.x(), q.y(), q.z(), q.w());
  double angle = 0.0;
  tf2::Vector3 axisInfo;
  angle = quat.getAngle();
  axisInfo = quat.getAxis();
  result(0) = axisInfo[0];
  result(1) = axisInfo[1];
  result(2) = axisInfo[2];
  result(3) = angle;
  return result;
}

// calculating the axis with the magnitude from the quaternion
Vector3d QuaternionOperation::CalcAxisMagFromQuaternion(Vector4d axisAng)
{
  Vector3d result;
  result(0) = axisAng(0) * axisAng(3);
  result(1) = axisAng(1) * axisAng(3);
  result(2) = axisAng(2) * axisAng(3);
  return result;
}

// calculating DCM, from NED to Body, using Euler angle (3->2->1)
// only 3-2-1 convention
// [          cy*cz,          cy*sz,            -sy]
// [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
// [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
Matrix3d QuaternionOperation::CalcDcmNtoB(Vector3d eulerAtt)
{
  Matrix3d result;
  double cx = cos(eulerAtt(0));
  double cy = cos(eulerAtt(1));
  double cz = cos(eulerAtt(2));
  double sx = sin(eulerAtt(0));
  double sy = sin(eulerAtt(1));
  double sz = sin(eulerAtt(2));
  result(0, 0) = cy * cz;
  result(0, 1) = cy * sz;
  result(0, 2) = -sy;
  result(1, 0) = sy * sx * cz - sz * cx;
  result(1, 1) = sy * sx * sz + cz * cx;
  result(1, 2) = cy * sx;
  result(2, 0) = sy * cx * cz + sz * sx;
  result(2, 1) = sy * cx * sz - cz * sx;
  result(2, 2) = cy * cx;
  return result;
}

// calculating DCM, Euler angle, 321 conversion (ref:from NED to Body, using Euler angle (3->2->1))
// only 3-2-1 convention
// [          cy*cz,          cy*sz,            -sy]
// [ sy*sx*cz-sz*cx, sy*sx*sz+cz*cx,          cy*sx]
// [ sy*cx*cz+sz*sx, sy*cx*sz-cz*sx,          cy*cx]
Matrix3d QuaternionOperation::CalcDcmEuler321(Vector3d eulerAtt)
{
  return CalcDcmNtoB(eulerAtt);
}

// converting from the position w.r.t ENU frame to the position w.r.t NED frame
Vector3d QuaternionOperation::ConvertPosFromEnuToNed(Vector3d posEnu)
{
  // tested(ok)
  Vector3d result;
  Vector3d attForEnuToNed;
  Matrix3d dcmForEnuToNed;
  attForEnuToNed(0) = (-180.0) * (D2R);
  attForEnuToNed(1) = (0.0) * (D2R);
  attForEnuToNed(2) = (90.0) * (D2R);
  dcmForEnuToNed = CalcDcmEuler321(attForEnuToNed);
  result = (dcmForEnuToNed) * (posEnu);
  return result;
}

// calculating the dot product between 3-dof vectors
double QuaternionOperation::CalcVec3dDot(Vector3d u, Vector3d v)
{
  return u.dot(v);
}

// calculating the cross product between 3-dof vectors
Vector3d QuaternionOperation::CalcVec3dCross(Vector3d u, Vector3d v)
{
  return u.cross(v);
}

// calculating the addition between 3-dof vectors
Vector3d QuaternionOperation::CalcVec3dAdd(Vector3d u, Vector3d v)
{
  return (u + v);
}

// calculating the subtraction between 3-dof vectors
Vector3d QuaternionOperation::CalcVec3dSub(Vector3d u, Vector3d v)
{
  return (u - v);
}

// calculating the sign conversion of 3-dof vector
Vector3d QuaternionOperation::CalcVec3dSgn(Vector3d u)
{
  Vector3d result;
  result(0) = (-1.0) * (u(0));
  result(1) = (-1.0) * (u(1));
  result(2) = (-1.0) * (u(2));
  return result;
}

// calculating the 2-norm of 3-dof vector
double QuaternionOperation::CalcVec3dNorm(Vector3d u)
{
  return sqrt(CalcVec3dDot(u, u));
}

// calculating the normalized result of 3-dof vector
Vector3d QuaternionOperation::CalcVec3dNormalize(Vector3d u)
{
  Vector3d result;
  double norm = CalcVec3dNorm(u);
  result(0) = (result(0)) / (norm + PRECISION);
  result(1) = (result(1)) / (norm + PRECISION);
  result(2) = (result(2)) / (norm + PRECISION);
  return result;
}

// calculating the distance between 3-dof vectors
double QuaternionOperation::CalcVec3dDist(Vector3d u, Vector3d v)
{
  Vector3d subVecRes;
  subVecRes = CalcVec3dSub(u, v);
  return CalcVec3dNorm(subVecRes);
}

// calculating the equality between 3-dof vectors
int QuaternionOperation::CalcVec3dEqual(Vector3d u, Vector3d v)
{
  int result = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    if (fabs((u(0)) - (v(0))) > PRECISION)
      result += 1;
  }
  return result;
}

// calculating the equality between 3-dof vectors with the threshold
int QuaternionOperation::CalcVec3dEqualThreshold(Vector3d u, Vector3d v, double threshold)
{
  int result = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    if (fabs((u(0)) - (v(0))) > threshold)
      result += 1;
  }
  return result;
}

// calculating the identity matrix, 3x3
Matrix3d QuaternionOperation::CalcMat3dIdentity()
{
  return Matrix3d::Identity();
}

// calculating the determinant of matrix, 3x3
double QuaternionOperation::CalcMat3dDeterminant(Matrix3d matP)
{
  return matP.determinant();
}

// calculating the addition of matrices, 3x3
Matrix3d QuaternionOperation::CalcMat3dAdd(Matrix3d matP, Matrix3d matQ)
{
  return matP + matQ;
}

// calculating the subtraction of matrices, 3x3
Matrix3d QuaternionOperation::CalcMat3dSub(Matrix3d matP, Matrix3d matQ)
{
  return matP - matQ;
}

// calculating the multiplication of matrices, 3x3
Matrix3d QuaternionOperation::CalcMat3dMultiply(Matrix3d matP, Matrix3d matQ)
{
  return matP * matQ;
}

// calculating the multiplication of matrix, 3x3 and vector3d
Vector3d QuaternionOperation::CalcMat3dVec3dMultiply(Matrix3d matP, Vector3d u)
{
  return matP * u;
}

// calculating the inverse of matrix, 3x3
Matrix3d QuaternionOperation::CalcMat3dInv(Matrix3d matP)
{
  return matP.inverse();
}

// solving the 3x3 equation, using inverse matrix
Vector3d QuaternionOperation::CalcMat3dVec3dSolve(Matrix3d matA, Vector3d vecB)
{
  return (CalcMat3dInv(matA)) * (vecB);
}

// calculating the equality between matrices, 3x3
int QuaternionOperation::CalcMat3dEqual(Matrix3d matP, Matrix3d matQ)
{
  int result = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    for (unsigned int j = 0; j < 3; j++)
    {
      if (fabs((matP(i, j)) - (matQ(i, j))) > PRECISION)
        result += 1;
    }
  }
  return result;
}

// calculating the equality between matrices, 3x3 with the threshold
int QuaternionOperation::CalcMat3dEqualThreshold(Matrix3d matP, Matrix3d matQ, double threshold)
{
  int result = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    for (unsigned int j = 0; j < 3; j++)
    {
      if (fabs((matP(i, j)) - (matQ(i, j))) > threshold)
        result += 1;
    }
  }
  return result;
}

// calculating the homogeneous matrix from the rotational and translational matrix
MatHomoGen QuaternionOperation::CalcMatHomoFromRotTranInfo(RotTrnInfo rotTrnInfo)
{
  MatHomoGen result;

  if (fabs(CalcMat3dDeterminant(rotTrnInfo.mRot) - 1.0) < PRECISION)
    ROS_ERROR("please check the rotation matrix status: determinant is less than 1.0");

  for (unsigned int i = 0; i < 3; i++)
  {
    for (unsigned int j = 0; j < 3; j++)
    {
      result(i, j) = rotTrnInfo.mRot(i, j);
    }
  }
  for (unsigned int k = 0; k < 3; k++)
    result(k, 3) = rotTrnInfo.mTrn(k, 0);

  return result;
}

// calculating the rotational and translational matrix from the homogeneous matrix
RotTrnInfo QuaternionOperation::CalcRotTranInfoFromMatHomo(MatHomoGen matHomoP)
{
  RotTrnInfo result;
  for (unsigned int i = 0; i < 3; i++)
  {
    for (unsigned int j = 0; j < 3; j++)
    {
      result.mRot(i, j) = matHomoP(i, j);
    }
  }
  for (unsigned int k = 0; k < 3; k++)
    result.mTrn(k, 0) = matHomoP(k, 3);
  return result;
}

// calculating the multiplication between homogeneous matrices
MatHomoGen QuaternionOperation::CalcMatHomoMultiply(MatHomoGen matHomoP, MatHomoGen matHomoQ)
{
  MatHomoGen result;
  for (unsigned int i = 0; i < 3; i++)
  {
    for (unsigned int j = 0; j < 4; j++)
    {
      result(i, j) = ((matHomoP(i, 0)) * (matHomoQ(0, j))) + ((matHomoP(i, 1)) * (matHomoQ(1, j))) +
                     ((matHomoP(i, 2)) * (matHomoQ(2, j)));
    }
  }
  for (unsigned int k = 0; k < 3; k++)
    result(k, 3) += matHomoP(k, 3);
  return result;
}

// calculating the multiplication between homogeneous matrix and vector, 4x1
VecHomoGen QuaternionOperation::CalcMatHomoVecHomoMultiply(MatHomoGen matHomoP, VecHomoGen vecHomoU)
{
  VecHomoGen result;
  for (unsigned int i = 0; i < 3; i++)
  {
    result(i, 0) = ((matHomoP(i, 0)) * (vecHomoU(0, 0))) + ((matHomoP(i, 1)) * (vecHomoU(1, 0))) +
                   ((matHomoP(i, 2)) * (vecHomoU(2, 0))) + ((matHomoP(i, 3)) * (vecHomoU(3, 0)));
  }
  result(3, 0) = vecHomoU(3, 0);
  return result;
}

// calculating the equality between matrices, 3x3
int QuaternionOperation::CalcMatHomoEqual(MatHomoGen matHomoP, MatHomoGen matHomoQ)
{
  int result = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    for (unsigned int j = 0; j < 4; j++)
    {
      if (fabs((matHomoP(i, j)) - (matHomoQ(i, j))) > PRECISION)
        result += 1;
    }
  }
  return result;
}

// calculating the equality between matrices, 3x3 with the threshold
int QuaternionOperation::CalcMatHomoEqualThreshold(MatHomoGen matHomoP, MatHomoGen matHomoQ, double threshold)
{
  int result = 0;
  for (unsigned int i = 0; i < 3; i++)
  {
    for (unsigned int j = 0; j < 4; j++)
    {
      if (fabs((matHomoP(i, j)) - (matHomoQ(i, j))) > threshold)
        result += 1;
    }
  }
  return result;
}

// for using functions in the other class
// -------------------------------------------------------------------------------------------------------------------------------------------------------
Quaterniond QuaternionOperation::GetAddQuaternions(Quaterniond q1, Quaterniond q2)
{
  return CalcAddQuaternions(q1, q2);
}

Quaterniond QuaternionOperation::GetSubQuaternions(Quaterniond q1, Quaterniond q2)
{
  return CalcSubQuaternions(q1, q2);
}

Quaterniond QuaternionOperation::GetConjugateQuaternion(Quaterniond q)
{
  return CalcConjugateQuaternion(q);
}

Quaterniond QuaternionOperation::GetScalarMultiplyQuaternion(double scalar, Quaterniond q)
{
  return CalcScalarMultiplyQuaternion(scalar, q);
}

Quaterniond QuaternionOperation::GetMultiplyQuaternions(Quaterniond q1, Quaterniond q2)
{
  return CalcMultiplyQuaternions(q1, q2);
}

Quaterniond QuaternionOperation::GetHalfQuaternion(Quaterniond q)
{
  return CalcHalfQuaternion(q);
}

Quaterniond QuaternionOperation::GetExpQuaternion(Quaterniond q)
{
  return CalcExpQuaternion(q);
}

Quaterniond QuaternionOperation::GetLogQuaternion(Quaterniond q)
{
  return CalcLogQuaternion(q);
}

double QuaternionOperation::GetNormQuaternion(Quaterniond q)
{
  return CalcNormQuaternion(q);
}

Matrix3d QuaternionOperation::GetQuaternionRotationMatrix(Quaterniond q)
{
  return CalcQuaternionRotationMatrix(q);
}

Matrix3d QuaternionOperation::GetQuaternionSkewMatrix(Quaterniond q)
{
  return CalcQuaternionSkewMatrix(q);
}

Matrix3d QuaternionOperation::GetQuaternionSymmSkewMatrix(Quaterniond q)
{
  return CalcQuaternionSymmSkewMatrix(q);
}

Quaterniond QuaternionOperation::GetSingleQuaternion(double qx, double qy, double qz, double qw)
{
  return CalcSingleQuaternion(qx, qy, qz, qw);
}

Vector3d QuaternionOperation::GetYPREulerAngFromQuaternion(Quaterniond q)
{
  return CalcYPREulerAngFromQuaternion(q);
}

Quaterniond QuaternionOperation::GetQuaternionFromYPREulerAng(Vector3d euler)
{
  return CalcQuaternionFromYPREulerAng(euler);
}

Vector4d QuaternionOperation::GetAxisAngFromQuaternion(Quaterniond q)
{
  return CalcAxisAngFromQuaternion(q);
}

Vector3d QuaternionOperation::GetAxisMagFromQuaternion(Vector4d axisAng)
{
  return CalcAxisMagFromQuaternion(axisAng);
}

double QuaternionOperation::GetVec3dDot(Vector3d u, Vector3d v)
{
  return CalcVec3dDot(u, v);
}

Vector3d QuaternionOperation::GetVec3dCross(Vector3d u, Vector3d v)
{
  return CalcVec3dCross(u, v);
}

Vector3d QuaternionOperation::GetVec3dAdd(Vector3d u, Vector3d v)
{
  return CalcVec3dAdd(u, v);
}

Vector3d QuaternionOperation::GetVec3dSub(Vector3d u, Vector3d v)
{
  return CalcVec3dSub(u, v);
}

Vector3d QuaternionOperation::GetVec3dSgn(Vector3d u)
{
  return CalcVec3dSgn(u);
}

double QuaternionOperation::GetVec3dNorm(Vector3d u)
{
  return CalcVec3dNorm(u);
}

Vector3d QuaternionOperation::GetVec3dNormalize(Vector3d u)
{
  return CalcVec3dNormalize(u);
}

double QuaternionOperation::GetVec3dDist(Vector3d u, Vector3d v)
{
  return CalcVec3dDist(u, v);
}

int QuaternionOperation::GetVec3dEqual(Vector3d u, Vector3d v)
{
  return CalcVec3dEqual(u, v);
}

int QuaternionOperation::GetVec3dEqualThreshold(Vector3d u, Vector3d v, double threshold)
{
  return CalcVec3dEqualThreshold(u, v, threshold);
}

Matrix3d QuaternionOperation::GetMat3dIdentity()
{
  return CalcMat3dIdentity();
}

double QuaternionOperation::GetMat3dDeterminant(Matrix3d matP)
{
  return CalcMat3dDeterminant(matP);
}

Matrix3d QuaternionOperation::GetMat3dAdd(Matrix3d matP, Matrix3d matQ)
{
  return CalcMat3dAdd(matP, matQ);
}

Matrix3d QuaternionOperation::GetMat3dSub(Matrix3d matP, Matrix3d matQ)
{
  return CalcMat3dSub(matP, matQ);
}

Matrix3d QuaternionOperation::GetMat3dMultiply(Matrix3d matP, Matrix3d matQ)
{
  return CalcMat3dMultiply(matP, matQ);
}

Vector3d QuaternionOperation::GetMat3dVec3dMultiply(Matrix3d matP, Vector3d u)
{
  return CalcMat3dVec3dMultiply(matP, u);
}

Matrix3d QuaternionOperation::GetMat3dInv(Matrix3d matP)
{
  return CalcMat3dInv(matP);
}

Vector3d QuaternionOperation::GetMat3dVec3dSolve(Matrix3d matA, Vector3d vecB)
{
  return CalcMat3dVec3dSolve(matA, vecB);
}

int QuaternionOperation::GetMat3dEqual(Matrix3d matP, Matrix3d matQ)
{
  return CalcMat3dEqual(matP, matQ);
}

int QuaternionOperation::GetMat3dEqualThreshold(Matrix3d matP, Matrix3d matQ, double threshold)
{
  return CalcMat3dEqualThreshold(matP, matQ, threshold);
}

void QuaternionOperation::GetMat3dPrint(Matrix3d matP)
{
  cout << "mat_1st=" << endl;
  cout << matP << endl;
}

void QuaternionOperation::GetMat3dsPrint(Matrix3d matP, Matrix3d matQ)
{
  cout << "mat_1st=" << endl;
  cout << matP << endl;
  cout << "\n" << endl;
  cout << "mat_2nd=" << endl;
  cout << matQ << endl;
}

MatHomoGen QuaternionOperation::GetMatHomoFromRotTranInfo(RotTrnInfo rotTrnInfo)
{
  return CalcMatHomoFromRotTranInfo(rotTrnInfo);
}

RotTrnInfo QuaternionOperation::GetRotTranInfoFromMatHomo(MatHomoGen matHomoP)
{
  return CalcRotTranInfoFromMatHomo(matHomoP);
}

MatHomoGen QuaternionOperation::GetMatHomoMultiply(MatHomoGen matHomoP, MatHomoGen matHomoQ)
{
  return CalcMatHomoMultiply(matHomoP, matHomoQ);
}

VecHomoGen QuaternionOperation::GetMatHomoVecHomoMultiply(MatHomoGen matHomoP, VecHomoGen vecHomoU)
{
  return CalcMatHomoVecHomoMultiply(matHomoP, vecHomoU);
}

int QuaternionOperation::GetMatHomoEqual(MatHomoGen matHomoP, MatHomoGen matHomoQ)
{
  return CalcMatHomoEqual(matHomoP, matHomoQ);
}

int QuaternionOperation::GetMatHomoEqualThreshold(MatHomoGen matHomoP, MatHomoGen matHomoQ, double threshold)
{
  return CalcMatHomoEqualThreshold(matHomoP, matHomoQ, threshold);
}

void QuaternionOperation::GetMatHomoPrint(MatHomoGen matHomoP)
{
  cout << "matHomo_1st=" << endl;
  cout << matHomoP << endl;
}

void QuaternionOperation::GetMatHomosPrint(MatHomoGen matHomoP, MatHomoGen matHomoQ)
{
  cout << "matHomo_1st=" << endl;
  cout << matHomoP << endl;
  cout << "\n" << endl;
  cout << "matHomo_2nd=" << endl;
  cout << matHomoQ << endl;
}

Vector3d QuaternionOperation::GetEulerNed(Vector3d eulerRos)
{
  Vector3d result;
  result(0) = eulerRos(0);
  result(1) = eulerRos(1);
  result(2) = (-1.0) * (eulerRos(2));
  return result;
}

// wrap-up function, angle between -PI and PI
float QuaternionOperation::wrapF(float angle)
{
  angle = (float)(fmod((double)(angle), 2.0 * PI));

  if (angle < -PI)
  {
    angle += 2.0f * PI;
  }
  else if (angle > PI)
  {
    angle -= 2.0f * PI;
  }
  else
  {
    angle = angle;
  }

  return angle;
}

// wrap-up function, angle between -PI and PI
double QuaternionOperation::wrapD(double angle)
{
  angle = fmod(angle, 2.0 * PI);

  if (angle < -PI)
  {
    angle += 2.0 * PI;
  }
  else if (angle > PI)
  {
    angle -= 2.0 * PI;
  }
  else
  {
    angle = angle;
  }

  return angle;
}
// -------------------------------------------------------------------------------------------------------------------------------------------------------