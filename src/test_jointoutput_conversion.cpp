#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "rpi_huno/ServoOdom.h"

#include <sstream>

/* =DESCRIPTION=
This node is a temporary node to convert rpi huno servoOdom messages to
sensor msgs JointState messages for robot state publisher.
*/

#define NUM_JOINTS 3
#define INIT_JOINT 13

class JointOdomConversion {
 public:
 ros::NodeHandle &node;
 ros::Subscriber sub_servo_odom;
 ros::Publisher pub_joint_states;

 sensor::JointState joint_state_odom;

 //Constructor
 JointOdomConversion(ros::NodeHandle &n): node(n),
  sub_servo_odom(node.subscribe("joint_odom",1,&JointOdomConversion::converter, this)),
  pub_joint_states(node.advertise<sensor_msgs::JointState>("joint_states", 1))
 {

 }

 //Callback
 void converter(const rpi_huno::ServoOdom& servo_odom_msg){
  joint_state_odom.header.stamp = ros::Time::now();
  joint_state_odom.name.resize(NUM_JOINTS);
  joint_state_odom.position.resize(NUM_JOINTS);

  for(int i=0; i<NUM_JOINTS; i++) {
   std::stringstream joint_name;
   joint_name.str("");
   joint_name << "j" << (i+INIT_JOINT);
   joint_state_odom.name[i] = joint_name.str();
   joint_state_odom.position[i] = servo_odom_msg.pos[i+INIT_JOINT];
  }

  pub_joint_states.publish(joint_state_odom);
 }
};

int main(int argc, char** argv) {
 ros::init(argc, argv, "joint_odom_conversion");
 ros::NodeHandle n;

 JointOdomConversion jointconversion(n);

 ros::spin();

 return 1;
}
