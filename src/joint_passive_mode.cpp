#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <wiringSerial.h>
#include <sstream>
/*
====DESCRIPTION
This node serves as the interface between software and joint servo motors
on the RQ Huno.

This is a node that simply commands the joints to remain in passive mode while
putting out joint angle values for debugging.

====Changes
ver0.0
-Identified as such June 6 2015.
ver0.1
-Changed joint pos and load messages from custom message to sensor_msgs/JointState

====ADDITIONAL INFO
==SAM-3 Servo Motors
Control Angle :      0~254 counts
Operation Angle :    0~269 degrees
Resolution :         8 bits / 1.055 deg
Unit Control Angle : 269 deg / 255 counts = 1.055 deg/count
Control Error :      +/- 0.8 deg

-wCK Protocol through full-duplex UART
Baud Rate :          115200 bps
1 Command (4byte)
Transmission Time :  0.347 usec
Command Response
Time (read/write) :  1180 usec(?)

*/

#define NUM_JOINTS 16
#define SAM3_MAX_COUNT 254
#define SAM3_DEG_TO_CTS (255.0/269) //Convert degrees to counts

class JointPassiveMode {
 public:
 ros::NodeHandle &node;
 //Publish current joint angles
 ros::Publisher joint_angles;

 //Serial port descriptor
 int servo_port;

 //Servo Status Variables
 sensor_msgs::JointState joint_status;

 //Other
 double jointSlewLimit;

 //======FUNCTIONS=========
 //Constructor
 JointPassiveMode(ros::NodeHandle &n) : node(n),
  joint_angles(node.advertise<sensor_msgs::JointState>("/joint_odom",1))
 {
  //Open serial port to servo motors (single port for all 16 motors)
  //Because servo motors are daisy-chain UART
  servo_port = serialOpen("/dev/ttyAMA0", 115200);
  if(servo_port < 0)
  { throw ros::Exception("Servo Port failed to be opened"); }
  if(serialDataAvail(servo_port))
  { //Buffer already had something
   serialFlush(servo_port);
  }

  //Initialize joint data
  int measured_pos_cts, measured_load_cts;
  int availData=0;
  joint_status.header.stamp = ros::Time::now();
  joint_status.name.resize(NUM_JOINTS);
  joint_status.position.resize(NUM_JOINTS);
  joint_status.effort.resize(NUM_JOINTS);
  for(int i=0; i<NUM_JOINTS; i++)
  { request_status(i); } //request current joint angles
  ros::Time start_wait = ros::Time::now();
  ros::Duration waiting_time = ros::Time::now() - start_wait;
  while(availData < 2*NUM_JOINTS       //Need to wait until all joints report back
        && waiting_time.toSec() < 0.5) //Timeout
  {
   availData = serialDataAvail(servo_port);
   waiting_time = ros::Time::now() - start_wait;
  }

  if(availData == 2*NUM_JOINTS)
  {
   for(int i=0; i<NUM_JOINTS; i++)
   {
    std::stringstream joint_name;
    joint_name << "j" << i;
    joint_status.name[i] = joint_name.str();

    measured_pos_cts = 0;
    measured_load_cts = 0;
    availData = read_joint_buffer(measured_pos_cts, measured_load_cts);
    if(availData < 0)
    { throw ros::Exception("Init joint buffer error"); }
    if(measured_pos_cts == 0)
    { throw ros::Exception("Failed to update joint pos"); }

    joint_status.position[i] = measured_pos_cts / SAM3_DEG_TO_CTS;
    joint_status.effort[i] = double(measured_load_cts);
   }
  }
  else
  { throw ros::Exception("Ctor failed to initialize joint angles"); }
 } //constructed JointController

 //Destructor
 ~JointPassiveMode()
 {

  serialClose(servo_port);
  ROS_INFO("Closed servo port");
 } //destructed JointController

 //Send commands to joint servos
 void send_passive_cmd(int motor_ID)
 {
  //des_pos and des_torq must already be converted to
  //servo required integers:
  //position : 0~254
  //torque : 0~4 (high to low)
  int tmp_byte1 = 0;
  int tmp_byte2 = 0;
  int checksum = 0;

  tmp_byte1 = (6 << 5) | motor_ID;
  tmp_byte2 = (1 << 4);
  checksum = (tmp_byte1 ^ tmp_byte2) & 0x7f;

  //Send to serial port according to wCK protocol
  serialPutchar(servo_port, 0xff); //Header
  serialPutchar(servo_port, tmp_byte1); //Data1 (motor id)
  serialPutchar(servo_port, tmp_byte2); //Data2
  serialPutchar(servo_port, checksum); //Checksum

  //Servo will return a position and load from the motor
 }

 //Request joint servo status
 void request_status(int motor_ID)
 {
  //returns current position and current load of servo
  //load : 0~254
  //position : 0~254
  int tmp_byte1 = 0;
  int tmp_byte2 = 0;
  int checksum = 0;
  int data_avail = 0;

  tmp_byte1 = (5 << 5) | motor_ID;
  checksum = (tmp_byte1 ^ tmp_byte2) & 0x7f;

  //Send to serial port according to wCK protocol
  serialPutchar(servo_port, 0xff);
  serialPutchar(servo_port, tmp_byte1);
  serialPutchar(servo_port, tmp_byte2);
  serialPutchar(servo_port, checksum);
 }

 int read_joint_buffer(int &current_pos_cts, int &current_load_cts)
 {
  int tmp_dataAvail = serialDataAvail(servo_port);
  if(tmp_dataAvail)
  {
   current_load_cts = serialGetchar(servo_port);
   current_pos_cts = serialGetchar(servo_port);
   tmp_dataAvail = serialDataAvail(servo_port);
   return tmp_dataAvail;
  }
  else
  { return -1; }
 }

 //--RUN FUNCTION--
 void run(void)
 {
  int tmp_pos_cts, tmp_load_cts;
  int availData=0;
  joint_status.header.stamp = ros::Time::now();
  for(int motor=0; motor<NUM_JOINTS; motor++)
  { send_passive_cmd(motor); }

  ros::Time start_wait = ros::Time::now();
  ros::Duration waiting_time = ros::Time::now() - start_wait;
  while(availData < (2*NUM_JOINTS)
       && waiting_time.toSec() < 0.5)
  {
   availData = serialDataAvail(servo_port);
   waiting_time = ros::Time::now() - start_wait;
  }

  if(availData == (2*NUM_JOINTS))
  {
   for(int motor=0; motor<NUM_JOINTS; motor++)
   {
    tmp_pos_cts = 0;
    tmp_load_cts = 0;
    availData = read_joint_buffer(tmp_pos_cts, tmp_load_cts);

    joint_status.position[motor] = tmp_pos_cts / SAM3_DEG_TO_CTS;
    joint_status.effort[motor] = double(tmp_load_cts);
   }
  }
  else
  { throw ros::Exception("Failed to read all joints"); }

  //Publish status
  joint_angles.publish(joint_status);
 }
}; //end class

//=========MAIN=====
int main(int argc, char **argv)
{
 ros::init(argc, argv, "joint_passive_mode");
 ros::NodeHandle n;

 //Get parameters from ROS Param Server
 double loop_freq;
 if(!n.getParam("/jointPassive/loop_freq", loop_freq))
 { throw ros::Exception("No joint_passive loop frequency"); }
 ros::Rate r(loop_freq);

 JointPassiveMode joint_passive_mode(n);

 while(ros::ok())
 {
  ros::spinOnce();
  joint_passive_mode.run();

  r.sleep();
 }

 // Shouldn't ever get here.
 return 1;
}
