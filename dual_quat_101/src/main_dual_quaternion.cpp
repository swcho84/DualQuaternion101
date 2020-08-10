#include "dual_quaternion_operations.h"

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
  init(argc, argv, "dual_quaternion_operations_node");
  NodeHandle nh("");

  DualQuaternionOperation dualQuatOper;

  // Tell ROS how fast to run this node.
  Rate loopRate(30);

  dualQuatOper.MainLoop();
  spin();

  dualQuatOper.~DualQuaternionOperation();

  return 0;
}  // end main()