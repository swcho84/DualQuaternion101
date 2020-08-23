#ifndef DUAL_QUAT_101_GLOBAL_HEADER_H
#define DUAL_QUAT_101_GLOBAL_HEADER_H

// for using vector type data
#include <iostream>
#include <string>
#include <stdio.h>
#include <ctime>
#include <vector>

// essential header for ROS operation
#include <ros/ros.h>

// for using eigen library
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

// for using tf w.r.t the quaternion
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define PRECISION 1e-10

#define POSITION 100
#define TRNVEC 200

using namespace std;
using namespace ros;
using namespace Eigen;

typedef Matrix<double, 3, 4> MatHomoGen;
typedef Matrix<double, 3, 3> MatRotInfo;
typedef Matrix<double, 3, 1> MatTrnInfo;
typedef Matrix<double, 4, 1> VecHomoGen;

typedef struct
{
  Quaterniond qReal;
  Quaterniond qDual;
} DualQuaternion;

typedef struct
{
  Quaterniond qAtt;
  Vector3d pos;
} QuatPos;

typedef struct
{
  MatRotInfo mRot;
  MatTrnInfo mTrn;
} RotTrnInfo;

#endif  // DUAL_QUAT_101_GLOBAL_HEADER_H