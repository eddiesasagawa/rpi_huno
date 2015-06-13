#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <Eigen/Dense>

/* ===DESCRIPTION===
This node is the forward kinematics node.

Inputs:
 sensor_msgs::JointState joint_current_angles

Outputs:
 ?? RightHand position/orientation
 ?? LeftHand position/orientation
 ?? RightFoot position/orientation
 ?? LeftFoot position/orientation

===TO-DOs===
-Output message format TBD.
-Calculate velocities and forces

*/

class HunoFK {
 public:
 ros::NodeHandle &node;
 ros::Subscriber sJoint_angles;

 HunoFK(ros::NodeHandle &n):
  node(n),
  sJoint_angles(node.subscribe("joint_odom",1,&HunoFK::runFK,this))
 {

 }

};
