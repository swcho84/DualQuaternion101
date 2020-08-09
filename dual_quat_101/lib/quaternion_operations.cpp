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
  // assigning the quaternion directly
  Quaterniond q1;
  q1 = CalcSingleQuaternion(-0.002, -0.780, 0.260, 0.569);
  ROS_INFO("input, q1:(x,y,z,w)=(%.4lf,%.4lf,%.4lf,%.4lf)", q1.x(), q1.y(), q1.z(), q1.w());

  // converting the quaternion to the Euler angle(3-2-1, ZYX, YPR)
  Vector3d vecEulerQ1;
  vecEulerQ1 = QuaternionOperation::CalcYPREulerAngFromQuaternion(q1);
  ROS_INFO("input, q1:euler(3-2-1):(r,p,y)=(%.4lf,%.4lf,%.4lf)", vecEulerQ1(0) * R2D, vecEulerQ1(1) * R2D,
           vecEulerQ1(2) * R2D);

  ROS_INFO(" ");

  // converting the Euler angle(3-2-1, ZYX, YPR) to the quaternion
  Quaterniond q1CrsCheck;
  q1CrsCheck = CalcQuaternionFromYPREulerAng(vecEulerQ1);
  ROS_INFO("input, q1ROSCrsCheck:(x,y,z,w)=(%.4lf,%.4lf,%.4lf,%.4lf)", q1CrsCheck.x(), q1CrsCheck.y(), q1CrsCheck.z(),
           q1CrsCheck.w());

  ROS_INFO(" ");

  // assigning the quaternion directly
  Quaterniond q2;
  q2 = CalcSingleQuaternion(0.4619398, -0.1913417, 0.4619398, 0.7325378);
  ROS_INFO("input, q2:(x,y,z,w)=(%.4lf,%.4lf,%.4lf,%.4lf)", q2.x(), q2.y(), q2.z(), q2.w());

  // converting the quaternion to the Euler angle(3-2-1, ZYX, YPR)
  Vector3d vecEulerQ2;
  vecEulerQ2 = QuaternionOperation::CalcYPREulerAngFromQuaternion(q2);
  ROS_INFO("input, q1:euler(3-2-1):(r,p,y)=(%.4lf,%.4lf,%.4lf)", vecEulerQ2(0) * R2D, vecEulerQ2(1) * R2D,
           vecEulerQ2(2) * R2D);

  ROS_INFO(" ");

  // converting the Euler angle(3-2-1, ZYX, YPR) to the quaternion
  Quaterniond q2CrsCheck;
  q2CrsCheck = CalcQuaternionFromYPREulerAng(vecEulerQ2);
  ROS_INFO("input, q2ROSCrsCheck:(x,y,z,w)=(%.4lf,%.4lf,%.4lf,%.4lf)", q2CrsCheck.x(), q2CrsCheck.y(), q2CrsCheck.z(),
           q2CrsCheck.w());

  ROS_INFO(" ");

  // calculating addition w.r.t the quaternions
  Quaterniond qAddRes = CalcAddQuaternions(q1, q2);
  ROS_INFO("ouput, add = (x,y,z,w)=(%.4lf,%.4lf,%.4lf,%.4lf)", qAddRes.x(), qAddRes.y(), qAddRes.z(), qAddRes.w());

  ROS_INFO(" ");

  // calculating subtraction w.r.t the quaternions
  Quaterniond qSubRes = CalcSubQuaternions(q1, q2);
  ROS_INFO("ouput, sub = (x,y,z,w)=(%.4lf,%.4lf,%.4lf,%.4lf)", qSubRes.x(), qSubRes.y(), qSubRes.z(), qSubRes.w());

  ROS_INFO(" ");

  // calculating the conjugate quaternion
  Quaterniond q1ConjRes = CalcConjugateQuaternion(q1);
  ROS_INFO("ouput, conj,q1 = (x,y,z,w)=(%.4lf,%.4lf,%.4lf,%.4lf)", q1ConjRes.x(), q1ConjRes.y(), q1ConjRes.z(),
           q1ConjRes.w());

  ROS_INFO(" ");

  // calculating the conjugate quaternion
  Quaterniond q2ConjRes = CalcConjugateQuaternion(q2);
  ROS_INFO("ouput, conj,q2 = (x,y,z,w)=(%.4lf,%.4lf,%.4lf,%.4lf)", q2ConjRes.x(), q2ConjRes.y(), q2ConjRes.z(),
           q2ConjRes.w());

  ROS_INFO(" ");

  // calculating the quaternion multiplication
  Quaterniond qMulRes = CalcMultiplyQuaternions(q1, q2);
  ROS_INFO("ouput, mul = (x,y,z,w)=(%.4lf,%.4lf,%.4lf,%.4lf)", qMulRes.x(), qMulRes.y(), qMulRes.z(), qMulRes.w());

  ROS_INFO(" ");

  // 3x3 matrix
  ROS_INFO("output, q1, skew matrix by quaternion");
  cout << CalcQuaternionSkewMatrix(q1) << endl;

  ROS_INFO(" ");

  // 3x3 matrix
  ROS_INFO("output, q1, quaternion rotation matrix");
  cout << CalcQuaternionRotationMatrix(q1) << endl;

  ROS_INFO(" ");

  // calculating the axis-angle from the quaternion
  Vector4d axisInfoQ1;
  axisInfoQ1 = CalcAxisAngFromQuaternion(q1);
  ROS_INFO("ouput, axisInfo, q1 = (x,y,z,theta)=(%.4lf,%.4lf,%.4lf,%.4lf)", axisInfoQ1(0), axisInfoQ1(1), axisInfoQ1(2),
           axisInfoQ1(3) * R2D);

  ROS_INFO(" ");

  // calculating the axis with the magnitude from the quaternion
  Vector3d normAxisQ1;
  normAxisQ1 = CalcAxisMagFromQuaternion(CalcAxisAngFromQuaternion(q1));
  ROS_INFO("ouput, axisNorm, q1 = (x,y,z)=(%.4lf,%.4lf,%.4lf)", normAxisQ1(0) * R2D, normAxisQ1(1) * R2D,
           normAxisQ1(2) * R2D);

  ROS_INFO(" ");

  // 3x3 matrix
  ROS_INFO("output, q2, skew matrix by quaternion");
  cout << CalcQuaternionSkewMatrix(q2) << endl;

  ROS_INFO(" ");

  // 3x3 matrix
  ROS_INFO("output, q2, quaternion rotation matrix");
  cout << CalcQuaternionRotationMatrix(q2) << endl;

  ROS_INFO(" ");

  // calculating the axis-angle from the quaternion
  Vector4d axisInfoQ2;
  axisInfoQ2 = CalcAxisAngFromQuaternion(q2);
  ROS_INFO("ouput, axisInfo, q2 = (x,y,z,theta)=(%.4lf,%.4lf,%.4lf,%.4lf)", axisInfoQ2(0), axisInfoQ2(1), axisInfoQ2(2),
           axisInfoQ2(3) * R2D);

  ROS_INFO(" ");

  // calculating the axis with the magnitude from the quaternion
  Vector3d normAxisQ2;
  normAxisQ2 = CalcAxisMagFromQuaternion(CalcAxisAngFromQuaternion(q2));
  ROS_INFO("ouput, axisNorm, q2 = (x,y,z)=(%.4lf,%.4lf,%.4lf)", normAxisQ2(0) * R2D, normAxisQ2(1) * R2D,
           normAxisQ2(2) * R2D);
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
           (2.0) * (q0) * (CalcQuaternionSkewMatrix(q));
  return result;
}

// calculating the skew matrix for YPR, 3-2-1 transformation
Matrix3d QuaternionOperation::CalcQuaternionSkewMatrix(Quaterniond q)
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
  result(0) = dRoll;
  result(1) = dPitch;
  result(2) = dYaw;
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
  result(3) = angle;  // wrapD(angle);
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