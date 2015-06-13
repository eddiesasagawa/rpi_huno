#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "common_functions.h"

#include <algorithm>
#include <vector>

/*
===== DESCRIPTION =======
This node will receive the joint angles published by the servo_controller
and send commands to the servo_controller.

Eventually, this node will also contain forward and inverse kinematics
(or maybe just one).

*/

#define NUM_JOINTS 16

class HunoKinematics {
 public:
 ros::NodeHandle &node;
 ros::Subscriber meas_joint_angles;
 ros::Publisher joint_commands;

 sensor_msgs::JointState joint_data;
 int slew_direction[NUM_JOINTS];
 std::vector<double> max_joint_pos_limits;
 std::vector<double> min_joint_pos_limits;
 double joint_slew;

 //======FUNCTIONS==============
 //Constructor
 HunoKinematics(ros::NodeHandle &n) : node(n),
  meas_joint_angles(node.subscribe("joint_odom",1,&HunoKinematics::commandJoints, this)),
  joint_commands(node.advertise<sensor_msgs::JointState>("/joint_commands",1))
 {
  //Set slew directions
  std::fill(slew_direction, slew_direction+NUM_JOINTS, 1);
  //Get slew limit
  if(!node.getParam("/kinematics/slew", joint_slew))
  { throw ros::Exception("No joint slew limit"); }

  //Get max and min positions limits
  if(!node.getParam("/jointControl/max_limits", max_joint_pos_limits))
  { throw ros::Exception("No joint max limits"); }
  else if(max_joint_pos_limits.size()!=NUM_JOINTS)
  { throw ros::Exception("Joint max limits vector incorrect"); }

  if(!node.getParam("/jointControl/min_limits", min_joint_pos_limits))
  { throw ros::Exception("No joint min limits"); }
  else if(min_joint_pos_limits.size()!=NUM_JOINTS)
  { throw ros::Exception("Joint min limits vector incorrect"); }

  //Size the joint_data message structure
  joint_data.name.resize(NUM_JOINTS);
  joint_data.position.resize(NUM_JOINTS);
  joint_data.effort.resize(NUM_JOINTS);
 } //constructed HunoKinematics

 //CALLBACK
 void commandJoints(const sensor_msgs::JointState& current_joint_angles)
 {
  double tmp_pos, tmp_load;
  joint_data.header.stamp = ros::Time::now();
  for(int joint=0; joint<NUM_JOINTS; joint++)
  {
   joint_data.name[joint] = current_joint_angles.name[joint];

   if( isLeftArm(joint) || isRightArm(joint) )
   {//Simply pass joint angle plus or minus slew rate for now
    tmp_pos = current_joint_angles.position[joint];
    tmp_load = current_joint_angles.effort[joint];

    if(tmp_pos > max_joint_pos_limits[joint])
    { slew_direction[joint] = -1; }
    else if(tmp_pos < min_joint_pos_limits[joint])
    { slew_direction[joint] = 1; }

    joint_data.position[joint] = tmp_pos + slew_direction[joint]*joint_slew;
    joint_data.effort[joint] = 1; //((tmp_load / 64)+0.5);
   }
   else //is a leg joint, so don't move those for now
   {
    joint_data.position[joint] = current_joint_angles.position[joint];
    joint_data.effort[joint] = 2;
   }
  }
  //Publish data
  joint_commands.publish(joint_data);
 } //End callback
}; //End HunoKinematics class

//=====MAIN======
int main(int argc, char **argv)
{
 ros::init(argc, argv, "huno_kinematics");
 ros::NodeHandle n;

 HunoKinematics huno_kinematics(n);

 ros::spin();

 return 1;
}
