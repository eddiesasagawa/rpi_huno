#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

#include <wiringSerial.h>
#include <sstream>
/*
====DESCRIPTION
This node serves as the interface between software and joint servo motors
on the RQ Huno

Specifically, this node will achieve the following:
 1. Query the servo motors for current position and current load
 2. Publish the received position and load data
 3. Subscribe to joint position and load commands to obtain desired
     joint positions
 4. Send joint commands to individual servo motors

====Changes
ver0.0
-Identified as such June 6 2015.
ver0.1
-Changed joint pos and load messages from custom message to sensor_msgs/JointState


====TO-DOs
- Add timeout exceptions to waiting for joints to respond
- Convert ROS_INFO calls to a message log publisher
- Display in log what the joint_target_pos array contains for
   potential debugging.

====ISSUES
-For all 16 joints to send back their commands, it takes on average 5.5 msecs.
  -Occassionally there will be spikes in length of time required, observed up to 20 msec!
  -For now, priority will be placed on waiting for all joints to report back.

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

#define DEBUG_MODE 1

#define NUM_JOINTS 16
#define SAM3_MAX_COUNT 254
#define SAM3_DEG_TO_CTS (255.0/269) //Convert degrees to counts

class JointController {
 public:
 ros::NodeHandle &node;
 //Subscribe to desired joint angles
 ros::Subscriber joint_commands;
 //Publish current joint angles
 ros::Publisher joint_angles;

 //Serial port descriptor
 int servo_port;

 //Servo Status Variables
 sensor_msgs::JointState joint_status;
 int joint_target_pos_cts[NUM_JOINTS];
 int joint_target_torq[NUM_JOINTS];

 //Other
 double jointSlewLimit;

 //======FUNCTIONS=========
 //Constructor
 JointController(ros::NodeHandle &n) : node(n),
  joint_commands(node.subscribe("joint_commands", 1, &JointController::updateJointTargets, this)),
  joint_angles(node.advertise<sensor_msgs::JointState>("/joint_odom",1))
 {
  //Open serial port to servo motors (single port for all 16 motors)
  //Because servo motors are daisy-chain UART
  servo_port = serialOpen("/dev/ttyAMA0", 115200);
  if(servo_port < 0)
  { throw ros::Exception("Servo Port failed to be opened"); }
  if(serialDataAvail(servo_port))
  { //Buffer already had something
//   ROS_INFO("Number of bytes found at start = %d", serialDataAvail(servo_port));
   serialFlush(servo_port);
//   ROS_INFO("Flushed servo port");
  }
  //Get parameters from ROS Param Server
  if(!node.getParam("/jointControl/joint_slew_limit", jointSlewLimit))
  { throw ros::Exception("No joint slew limit"); }

  //Initialize joint data
  int measured_pos_cts, measured_load_cts;
  int availData=0;
  joint_status.header.stamp = ros::Time::now();
  joint_status.name.resize(NUM_JOINTS);
  joint_status.position.resize(NUM_JOINTS);
  joint_status.effort.resize(NUM_JOINTS);
  for(int i=0; i<NUM_JOINTS; i++)
  { request_status(i); } //request current joint angles
  //ros::Duration(0.0012).sleep(); //wait 1.2 millisec for responses from servos
  ros::Time start_wait = ros::Time::now();
  ros::Duration waiting_time = ros::Time::now() - start_wait;
  while(availData < 2*NUM_JOINTS       //Need to wait until all joints report back
        || waiting_time.toSec() > 0.5) //Timeout
  {
   availData = serialDataAvail(servo_port);
   waiting_time = ros::Time::now() - start_wait;
  }

  if(availData == 2*NUM_JOINTS)
  {
//   ROS_INFO("Grabbing angles... %d left..", availData);
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

//    ROS_INFO("%d left..", availData);

    joint_status.position[i] = measured_pos_cts / SAM3_DEG_TO_CTS;
    joint_status.effort[i] = double(measured_load_cts);

    joint_target_pos_cts[i] = measured_pos_cts;
    joint_target_torq[i] = 2;
   }
  }
  else
  { throw ros::Exception("Ctor failed to initialize joint angles"); }
 } //constructed JointController

 //Destructor
 ~JointController()
 {

  serialClose(servo_port);
  ROS_INFO("Closed servo port");
 } //destructed JointController

 //Send commands to joint servos
 void send_command(int motor_ID, int des_pos_cts, int des_torq_cts)
 {
  //des_pos and des_torq must already be converted to
  //servo required integers:
  //position : 0~254
  //torque : 0~4 (high to low)
  int tmp_byte1 = 0;
  int tmp_byte2 = 0;
  int checksum = 0;

  tmp_byte1 = (des_torq_cts << 5) | motor_ID;
  tmp_byte2 = des_pos_cts;
  checksum = (tmp_byte1 ^ tmp_byte2) & 0x7f;

  //Send to serial port according to wCK protocol
  serialPutchar(servo_port, 0xff); //Header
  serialPutchar(servo_port, tmp_byte1); //Data1 (torque and motor id)
  serialPutchar(servo_port, tmp_byte2); //Target Position
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

 //--CALLBACK--
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

 //--RUN FUNCTION--
 void run(void)
 {
  int tmp_pos_cts, tmp_load_cts;
  int availData=0;
  joint_status.header.stamp = ros::Time::now();
  for(int motor=0; motor<NUM_JOINTS; motor++)
  { send_command(motor, joint_target_pos_cts[motor], joint_target_torq[motor]); }

  //ros::Duration(0.0012).sleep(); //wait for 1.2 millisec for responses from motors
  ros::Time start_wait = ros::Time::now();
  ros::Duration waiting_time = ros::Time::now() - start_wait;
  while(availData < (2*NUM_JOINTS)
       || waiting_time.toSec() > 0.5)
  {
   availData = serialDataAvail(servo_port);
   waiting_time = ros::Time::now() - start_wait;
  }

  if(availData == (2*NUM_JOINTS))
  {
//   ROS_INFO("Run loop. Grabbing joint angles.. %d left..", availData);
   for(int motor=0; motor<NUM_JOINTS; motor++)
   {
    tmp_pos_cts = 0;
    tmp_load_cts = 0;
    availData = read_joint_buffer(tmp_pos_cts, tmp_load_cts);
//    ROS_INFO("Run loop. %d left..", availData);

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
 ros::init(argc, argv, "joint_control");
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
