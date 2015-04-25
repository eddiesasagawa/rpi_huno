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

 //======FUNCTIONS=========
 
 //Constructor
 servo_controller(ros::NodeHandle &n) : node(n),
  joint_commands(node.subscribe("joint_commands", 1, &servo_controller::servoInterface, this)),
  joint_angles(node.advertise<rpi_huno::ServoOdom>("/servo_odom",1)),
 {
  //Open serial port to servo motors (single port for all 16 motors)
  //Because servo motors are daisy-chain UART
  servo_port = serialOpen("/dev/ttyAMA0", 115200);
  if(servo_port < 0)
  { throw ros::Exception("Servo Port failed to be opened"); }
 } //constructed servo_controller

 //Destructor
 ~servo_controller()
 {
  serialClose(servo_port);
 } //destructed servo_controller

 //Send commands to servos
 int command_servo(int motor_ID, int des_pos, int des_torq)
 {
  //des_pos and des_torq must already be converted to
  //servo required integers:
  //position : 0~255
  //torque : 0~5 (high to low)
  int tmp_byte1, tmp_byte2;
  
 }

 //Request servo status
 void request_servo(int motor_ID, int *current_pos, int *current_load)
 {
  //returns current position and current load of servo
  //position : 0~255
  //load : 0~255
  int tmp_byte1, tmp_byte2;
  
 }

 //--CALLBACK / Send Servo Commands--
 void servoInterface(const rpi_huno::ServoOdom& servo_cmds)
 {
  
 }
