#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <sstream>

/*
====DESCRIPTION
This node simulates joint_control node so that the real hardware 
is not necessary to run the system.

For now, this node will immediately publish back out the target
joint angles.

Eventually, a simple control law to simulate joint tracking
performance may be added. -- maybe --
*/

#define NUM_JOINTS 16
#define SAM3_MAX_COUNT 254
#define SAM3_DEG_TO_CTS (255.0/269) //Convert degrees to counts

class JointController {
private:
 ros::NodeHandle &node;
 //Subscribe to desired joint angles
 ros::Subscriber joint_commands;
 //Publish current joint angles
 ros::Publisher joint_angles;

 //Servo Status Variables
 sensor_msgs::JointState joint_status;
 int joint_target_pos_cts[NUM_JOINTS];
 int joint_target_torq[NUM_JOINTS];

 //======FUNCTIONS=========
 //--CALLBACK--
 // Update the target positions of each servo motor
 // @param joint_cmds : the subscribed message that triggered this callback
 void updateJointTargets(const sensor_msgs::JointState& joint_cmds)
 {
  int tmp_pos_cmd_cts, tmp_torq_cts;
  for(int motor = 0; motor < NUM_JOINTS; motor++)
  {
   //Ensure commands are valid first and convert
   tmp_pos_cmd_cts = int((joint_cmds.position[motor])*SAM3_DEG_TO_CTS);
   if(tmp_pos_cmd_cts > SAM3_MAX_COUNT)
   { tmp_pos_cmd_cts = SAM3_MAX_COUNT; }
   else if(tmp_pos_cmd_cts < 0)
   { tmp_pos_cmd_cts = 0; }

   tmp_torq_cts = int(joint_cmds.effort[motor]);
   if(tmp_torq_cts < 0)
   { tmp_torq_cts = 0; }
   else if(tmp_torq_cts > 4)
   { tmp_torq_cts = 4; }

   joint_target_pos_cts[motor] = tmp_pos_cmd_cts;
   joint_target_torq[motor] = tmp_torq_cts;
  }
 }

public:
 //--RUN FUNCTION--
 // Main function to run each cycle, sending commands to servo motors and receiving current positions
 void run(void)
 {
  joint_status.header.stamp = ros::Time::now();

  ros::Time start_wait = ros::Time::now();
  ros::Duration waiting_time = ros::Time::now() - start_wait;

  for(int motor=0; motor<NUM_JOINTS; motor++)
  {
   joint_status.position[motor] = joint_target_pos_cts[motor] / SAM3_DEG_TO_CTS;
   joint_status.effort[motor] = joint_target_torq[motor];
  }

  //Publish status
  joint_angles.publish(joint_status);
 }

 //Constructor
 // @param n : node handle
 JointController(ros::NodeHandle &n) : node(n),
  joint_commands(node.subscribe("joint_commands", 1, &JointController::updateJointTargets, this)),
  joint_angles(node.advertise<sensor_msgs::JointState>("/joint_odom",1))
 {
  //Initialize joint data
  joint_status.header.stamp = ros::Time::now();
  joint_status.name.resize(NUM_JOINTS);
  joint_status.position.resize(NUM_JOINTS);
  joint_status.effort.resize(NUM_JOINTS);

  for(int i=0; i<NUM_JOINTS; i++)
  {
   std::stringstream joint_name;
   joint_name << "j" << i;
   joint_status.name[i] = joint_name.str();

   joint_status.position[i] = 100;
   joint_status.effort[i] = 1;

   joint_target_pos_cts[i] = 100;
   joint_target_torq[i] = 1;
  }
 } //constructed JointController

 //Destructor
 ~JointController()
 {
 } //destructed JointController
}; //end class

//=========MAIN=====
int main(int argc, char **argv)
{
 ros::init(argc, argv, "joint_control_simulator");
 ros::NodeHandle n;

 //Get parameters from ROS Param Server
 double loop_freq;
 if(!n.getParam("/jointControl/loop_freq", loop_freq))
 { throw ros::Exception("No joint_control loop frequency"); }
 ros::Rate r(loop_freq);

 JointController joint_control(n);

 while(ros::ok())
 {
  ros::spinOnce();
  joint_control.run();

  r.sleep();
 }

 // Shouldn't ever get here.
 return 1;
}

