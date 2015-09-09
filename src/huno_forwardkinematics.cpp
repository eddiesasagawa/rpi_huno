#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"

#include "HunoForwardKinematics.h"
#include "rpi_huno/HunoLimbPoses.h"

#include <sstream>
#include <Eigen/Dense>

/* ===DESCRIPTION===
This node is the forward kinematics node.

Subscribe To:
 sensor_msgs::JointState joint_current_angles

Publish:
 geometry_msgs::Pose RightHand position/orientation
 geometry_msgs::Pose LeftHand position/orientation
 geometry_msgs::Pose RightFoot position/orientation
 geometry_msgs::Pose LeftFoot position/orientation

===TO-DOs===
-Calculate velocities and forces

*/

#define DEG2RAD (3.14/180)

class HunoFK {
 public:
 ros::NodeHandle &node;
 ros::Subscriber sJoint_angles;
 ros::Publisher pHuno_poses;
 ros::Publisher pLog;

 HunoForwardKinematics huno_fwdkin;
 Eigen::Matrix4f head_T_LFoot;
 Eigen::Matrix4f head_T_RFoot;
 Eigen::Matrix4f head_T_LHand;
 Eigen::Matrix4f head_T_RHand;

 ros::Time init_time;

 //Constructor
 HunoFK(ros::NodeHandle &n):
  node(n),
  sJoint_angles(node.subscribe("joint_odom",1,&HunoFK::runFK,this)),
  pHuno_poses(node.advertise<rpi_huno::HunoLimbPoses>("/huno_poses",1)),
  pLog(node.advertise<std_msgs::String>("/huno_fk_log",1)),
  huno_fwdkin(),
  head_T_LFoot(Eigen::Matrix4f::Identity()),
  head_T_RFoot(Eigen::Matrix4f::Identity()),
  head_T_LHand(Eigen::Matrix4f::Identity()),
  head_T_RHand(Eigen::Matrix4f::Identity()),
  init_time(ros::Time::now())
 {
 }

 void publishFK() {
  Eigen::Vector3f rot_axis = Eigen::Vector3f::Zero();
  float rot_angle;
  rpi_huno::HunoLimbPoses huno_poses;

  huno_poses.header.stamp = ros::Time::now();

  //Left foot
  huno_poses.left_foot.position.x = head_T_LFoot(0,3);
  huno_poses.left_foot.position.y = head_T_LFoot(1,3);
  huno_poses.left_foot.position.z = head_T_LFoot(2,3);
  rot_angle = acos( ( ((head_T_LFoot.block<3,3>(0,0)).trace())-1 )/2 ); //Back out rotation unit vector and angle from rotation matrix
  if(sin(rot_angle) == 0) //Rotation matrix is identity and thus unit vector is arbitrary, or matrix was invalid such that it would cause divide by zero
  { rot_axis = Eigen::Vector3f::Zero(); }
  else
  {
   rot_axis(0) = ((head_T_LFoot(2,1)-head_T_LFoot(1,2))/(2*sin(rot_angle)));
   rot_axis(1) = ((head_T_LFoot(0,2)-head_T_LFoot(2,0))/(2*sin(rot_angle)));
   rot_axis(2) = ((head_T_LFoot(1,0)-head_T_LFoot(0,1))/(2*sin(rot_angle)));
  }
  huno_poses.left_foot.orientation.x = rot_axis(0)*sin(rot_angle/2);
  huno_poses.left_foot.orientation.y = rot_axis(1)*sin(rot_angle/2);
  huno_poses.left_foot.orientation.z = rot_axis(2)*sin(rot_angle/2);
  huno_poses.left_foot.orientation.w = cos(rot_angle/2);

  //Right foot
  huno_poses.right_foot.position.x = head_T_RFoot(0,3);
  huno_poses.right_foot.position.y = head_T_RFoot(1,3);
  huno_poses.right_foot.position.z = head_T_RFoot(2,3);
  rot_angle = acos( ( ((head_T_RFoot.block<3,3>(0,0)).trace())-1 )/2 ); //Back out rotation unit vector and angle from rotation matrix
  if(sin(rot_angle) == 0) //Rotation matrix is identity and thus unit vector is arbitrary
  { rot_axis = Eigen::Vector3f::Zero(); }
  else
  {
   rot_axis(0) = ((head_T_RFoot(2,1)-head_T_RFoot(1,2))/(2*sin(rot_angle)));
   rot_axis(1) = ((head_T_RFoot(0,2)-head_T_RFoot(2,0))/(2*sin(rot_angle)));
   rot_axis(2) = ((head_T_RFoot(1,0)-head_T_RFoot(0,1))/(2*sin(rot_angle)));
  }
  huno_poses.right_foot.orientation.x = rot_axis(0)*sin(rot_angle/2);
  huno_poses.right_foot.orientation.y = rot_axis(1)*sin(rot_angle/2);
  huno_poses.right_foot.orientation.z = rot_axis(2)*sin(rot_angle/2);
  huno_poses.right_foot.orientation.w = cos(rot_angle/2);

  //Left hand
  huno_poses.left_hand.position.x = head_T_LHand(0,3);
  huno_poses.left_hand.position.y = head_T_LHand(1,3);
  huno_poses.left_hand.position.z = head_T_LHand(2,3);
  rot_angle = acos( ( ((head_T_LHand.block<3,3>(0,0)).trace())-1 )/2 ); //Back out rotation unit vector and angle from rotation matrix
  if(sin(rot_angle) == 0) //Rotation matrix is identity and thus unit vector is arbitrary
  { rot_axis = Eigen::Vector3f::Zero(); }
  else
  {
   rot_axis(0) = ((head_T_LHand(2,1)-head_T_LHand(1,2))/(2*sin(rot_angle)));
   rot_axis(1) = ((head_T_LHand(0,2)-head_T_LHand(2,0))/(2*sin(rot_angle)));
   rot_axis(2) = ((head_T_LHand(1,0)-head_T_LHand(0,1))/(2*sin(rot_angle)));
  }
  huno_poses.left_hand.orientation.x = rot_axis(0)*sin(rot_angle/2);
  huno_poses.left_hand.orientation.y = rot_axis(1)*sin(rot_angle/2);
  huno_poses.left_hand.orientation.z = rot_axis(2)*sin(rot_angle/2);
  huno_poses.left_hand.orientation.w = cos(rot_angle/2);

  //Right hand
  huno_poses.right_hand.position.x = head_T_RHand(0,3);
  huno_poses.right_hand.position.y = head_T_RHand(1,3);
  huno_poses.right_hand.position.z = head_T_RHand(2,3);
  rot_angle = acos( ( ((head_T_RHand.block<3,3>(0,0)).trace())-1 )/2 ); //Back out rotation unit vector and angle from rotation matrix
  if(sin(rot_angle) == 0) //Rotation matrix is identity and thus unit vector is arbitrary
  { rot_axis = Eigen::Vector3f::Zero(); }
  else
  {
   rot_axis(0) = ((head_T_RHand(2,1)-head_T_RHand(1,2))/(2*sin(rot_angle)));
   rot_axis(1) = ((head_T_RHand(0,2)-head_T_RHand(2,0))/(2*sin(rot_angle)));
   rot_axis(2) = ((head_T_RHand(1,0)-head_T_RHand(0,1))/(2*sin(rot_angle)));
  }
  huno_poses.right_hand.orientation.x = rot_axis(0)*sin(rot_angle/2);
  huno_poses.right_hand.orientation.y = rot_axis(1)*sin(rot_angle/2);
  huno_poses.right_hand.orientation.z = rot_axis(2)*sin(rot_angle/2);
  huno_poses.right_hand.orientation.w = cos(rot_angle/2);

  //Publish
  pHuno_poses.publish(huno_poses);
 }

 void runFK(const sensor_msgs::JointState& joint_states) {
  head_T_LFoot = huno_fwdkin.LimbFK(0,4, joint_states.position[0]);

  head_T_RFoot = huno_fwdkin.RightFootFK(5,9, &joint_states.position[5]);

  head_T_LHand = huno_fwdkin.LeftHandFK(10,12, &joint_states.position[10]);

  head_T_RHand = huno_fwdkin.RightHandFK(13,15, &joint_states.position[13]);

  publishFK();

  std_msgs::String log_msg;
  std::stringstream log_line;
  log_line << (ros::Time::now()-init_time).toSec() << ',';
  log_line << joint_states.position[13] << ',';
  log_line << joint_states.position[14] << ',';
  log_line << joint_states.position[15] << ',';
  log_line << head_T_RHand(0,3) << ',';
  log_line << head_T_RHand(1,3) << ',';
  log_line << head_T_RHand(2,3) << ',';

  log_msg.data = log_line.str();
  pLog.publish(log_msg);
 }

}; //End class

int main(int argc, char **argv) {
 ros::init(argc, argv, "huno_forwardkinematics");
 ros::NodeHandle n;

 HunoFK huno_fk(n);

 ros::spin();

 return 1; //Shouldn't ever get here.
}
