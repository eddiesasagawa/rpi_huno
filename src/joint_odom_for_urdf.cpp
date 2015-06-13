#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

/* ===DESCRIPTION====
Node to take joint angle outputs and re-publish for robot state
publisher to convert urdf.

Messages are already in sensor_msgs::JointState, but trim down based
on how joints are in urdf model.

Eventually this node will be unnecessary as all joints will be tracked.
*/

#define NUM_URD_JOINTS 3
#define URD_INIT_JOINT 13

class JointOdomConversion {
 public:
 ros::NodeHandle &node;
 ros::Subscriber sub_joint_odom;
 ros::Publisher pub_joint_odom_p;

 sensor_msgs::JointState joint_states;

//Constructor
 JointOdomConversion(ros::NodeHandle &n):
  node(n),
  sub_joint_odom(node.subscribe("joint_odom",1,&JointOdomConversion::converter, this)),
  pub_joint_odom_p(node.advertise<sensor_msgs::JointState>("/joint_states", 1))
 {
  joint_states.header.stamp = ros::Time::now();
  joint_states.name.resize(NUM_URD_JOINTS);
  joint_states.position.resize(NUM_URD_JOINTS);
 }

 void converter(const sensor_msgs::JointState& joint_odom) {
  joint_states.header.stamp = ros::Time::now();

  for(int i=0; i<NUM_URD_JOINTS; i++) {
   joint_states.name[i] = joint_odom.name[i+URD_INIT_JOINT];
   joint_states.position[i] = joint_odom.position[i+URD_INIT_JOINT];
  }
  pub_joint_odom_p.publish(joint_states);
 }

};

int main(int argc, char** argv) {
 ros::init(argc, argv, "joint_odom_conversion");
 ros::NodeHandle n;

 JointOdomConversion joint_odom_conversion(n);

 ros::spin();

 return 1;
}
