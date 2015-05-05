#include "ros/ros.h"
#include "rpi_huno/ServoOdom.h"

#include <algorithm>

#define NUM_JOINTS 16

class HunoKinematics {
 public:
 ros::NodeHandle &node;
 ros::Subscriber meas_joint_angles;
 ros::Publisher joint_commands;

 rpi_huno::ServoOdom joint_data;
 int slew_direction[NUM_JOINTS];
 
 double joint_slew;

 //======FUNCTIONS==============
 //Constructor
 HunoKinematics(ros::NodeHandle &n) : node(n),
  meas_joint_angles(node.subscribe("servo_odom",1,&HunoKinematics::commandJoints, this)),
  joint_commands(node.advertise<rpi_huno::ServoOdom>("/joint_commands",1))
 {
  //Set slew directions
  std::fill(slew_direction, slew_direction+NUM_JOINTS, 1);
  //Get slew limit
  if(!node.getParam("/kinematics/slew", joint_slew))
  { throw ros::Exception("No joint slew limit"); }
 } //constructed HunoKinematics

 //CALLBACK
 void commandJoints(const rpi_huno::ServoOdom& current_joint_angles)
 {
  double tmp_pos, tmp_load;
  for(int joint=0; joint<NUM_JOINTS; joint++)
  {
   //Simply pass joint angle plus or minus slew rate for now
   tmp_pos = current_joint_angles.pos[joint];
   tmp_load = current_joint_angles.torqload[joint];
   
   if(tmp_pos > 210)
   { slew_direction[joint] = -1; }
   else if(tmp_pos < 40)
   { slew_direction[joint] = 1; }
   
   joint_data.pos[joint] = tmp_pos + slew_direction[joint]*joint_slew;
   joint_data.torqload[joint] = ((tmp_load / 64)+0.5);

  }
 }

};
