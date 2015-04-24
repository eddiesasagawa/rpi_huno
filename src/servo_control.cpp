#include "ros/ros.h"
#include "msg/ServoOdom.h"

#include <wiringSerial.h>

/*
====DESCRIPTION
This node serves as the interface between software and servo motors
on the RQ Huno

Specifically, this node will achieve the following:
 1. Query the servo motors for current position and current load
 2. Publish the received position and load data
 3. Subscribe to joint position and load commands to obtain desired
     joint positions
 4. Send joint commands to individual servo motors

*/

class servo_controller {
 public:
 ros::NodeHandle &node;
 //Subscribe to desired joint angles
 ros::Subscriber joint_commands;
 //Publish current joint angles
 ros::Publisher joint_angles;

 //Serial port descriptor
 int servo_port;

 servo_controller(ros::NodeHandle &n) : node(n),
  joint_commands(node.subscribe("joint_commands", 1, &servo_controller::sendServoCommands, this)),
  joint_angles(node.advertise<rpi_huno::ServoOdom>("/servo_odom",1)),
  {
   //Open serial port to servo motors (single port for all 16 motors)
   //Because servo motors are daisy-chain UART
   servo_port = serialOpen("/dev/ttyAMA0", 115200);
   if(servo_port < 0)
   { throw ros::Exception("Servo Port failed to be opened"); }
  }

 //--CALLBACK / Send Servo Commands--
 void sendServoCommands(const rpi_huno::ServoOdom& servo_cmds)
 {
  
 }
