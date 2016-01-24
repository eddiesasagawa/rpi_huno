#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"

#include "rpi_huno/HunoLimbPoses.h"

#include "common_functions.h"

/****************************************************************
**  inverse_kinematics_control
*****************************************************************
* author: E. Sasagawa
* 
* This node handles the limb-level control law for all 4 limbs
* and also performs inverse kinematics on the desired control
* action (EE position) to determine the command joint angles. 
*
* Subscribe to:
*  HunoLimbPoses message
*   geometry_msgs::Pose RightHand position/orientation
*   geometry_msgs::Pose LeftHand position/orientation
*   geometry_msgs::Pose RightFoot position/orientation
*   geometry_msgs::Pose LeftFoot position/orientation
*
*  LimbPose TargetPoints TODO
*   Target positions for each limb
*
* Publish:
*  sensor_msgs::JointState joint_angle_commands
*
* TODOs:
*  Identify other potential methods of getting target
*   pose/velocity. E.g. sending quadratic parameters
****************************************************************/

class IK_Control
{
private:
  ros::NodeHandle &node;       // ros node
  ros::Subscriber subPoseRsp;  // subscriber to HunoLimbPoses
  ros::Subscriber subPoseCmds; // subscriber to pose commands
  ros::Publisher pubJointCmds; // publisher for joint commands

  // limb control parameters (parameters for each limb)
  double errorRH;
  double errorLH;
  double errorRF;
  double errorLF;

  // Inverse kinematics variables

public:
  /* Constructor */
  IK_Control(ros::NodeHandle &n):
    node(n),
    subPose(node.subscribe("huno_poses", 1, &IK_Control::Run, this)),
    pubJointCmds(node.advertise<sensor_msgs::JointState>("/ik_joint_commands",1))
  {
  }
} // end class IK_Control

/** MAIN **/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "inverse_kinematics_control");
  ros::NodeHandle n;

  //IK_Control ik_control(n);

  ros::spin();

  return 1; // Shouldn't ever get here.
}

