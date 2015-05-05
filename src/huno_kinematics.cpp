#include "ros/ros.h"
#include "rpi_huno/ServoOdom.h"

#define NUM_JOINTS 16

class hunoKinematics
{
 public:
 ros::NodeHandle &node;
 ros::Subscriber meas_joint_angles;
 ros::Publisher joint_commands;

 rpi_huno::ServoOdom joint_command_data;

 //Constructor
 hunoKinematics(ros::NodeHandle &n): node(n),
  meas_joint_angles(node.subscribe()),
  joint_commands(node.advertise(""))
 {

 }



};
