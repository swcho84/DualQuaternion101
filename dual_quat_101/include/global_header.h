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

using namespace std;
using namespace ros;
using namespace Eigen;

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

#endif  // DUAL_QUAT_101_GLOBAL_HEADER_H