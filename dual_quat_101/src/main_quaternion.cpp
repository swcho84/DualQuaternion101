#include "quaternion_operations.h"

using namespace std;
using namespace ros;
using namespace Eigen;

/*--------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *------------------------------------------------------------------*/

int main(int argc, char** argv)
{
  // Set up ROS.
  init(argc, argv, "quaternion_operations_node");
  NodeHandle nh("");

  QuaternionOperation quatOper;

  // Tell ROS how fast to run this node.
  Rate loopRate(30);

  // Main loop.
  // while (ok())
  // {
  //   quatOper.MainLoop();

  //   spinOnce();
  //   loopRate.sleep();
  // }

  quatOper.MainLoop();

  spin();

  quatOper.~QuaternionOperation();

  return 0;
}  // end main()