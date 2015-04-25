#include "ros/ros.h"
#include "msg/ServoOdom.h"

#include <wiringSerial.h>

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

====TO-DOs
- Write a run() function that reads and writes to joints at a set rate
   independent of the callback, and callback only updates the commands.
   Telemetry will be published here.
- Convert ROS_INFO calls to a message log publisher
- Display in log what the joint_target_pos array contains for
   potential debugging.

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
#define SAM3_DEG_TO_CTS (255/269) //Convert degrees to counts

class servo_controller {
 public:
 ros::NodeHandle &node;
 //Subscribe to desired joint angles
 ros::Subscriber joint_commands;
 //Publish current joint angles
 ros::Publisher joint_angles;

 //Serial port descriptor
 int servo_port;

 //Servo Status Variables
 int joint_meas_pos[NUM_JOINTS];
 int joint_meas_load[NUM_JOINTS];
 int joint_target_pos[NUM_JOINTS];
 int joint_target_torq[NUM_JOINTS];

 //Other
 double jointSlewLimit;

 //======FUNCTIONS=========
 //Constructor
 servo_controller(ros::NodeHandle &n) : node(n),
  joint_commands(node.subscribe("joint_commands", 1, &servo_controller::updateJointTargets, this)),
  joint_angles(node.advertise<rpi_huno::ServoOdom>("/servo_odom",1))
 {
  //Open serial port to servo motors (single port for all 16 motors)
  //Because servo motors are daisy-chain UART
  servo_port = serialOpen("/dev/ttyAMA0", 115200);
  if(servo_port < 0)
  { throw ros::Exception("Servo Port failed to be opened"); }
  if(serialDataAvail(servo_port))
  { //Buffer already had something
   ROS_INFO("Number of bytes found at start = %d", serialDataAvail(servo_port));
   serialFlush(servo_port);
   ROS_INFO("Flushed servo port");
  }
  //Get parameters from ROS Param Server
  if(!node.getParam("/joint_params/joint_slew_limit", jointSlewLimit))
  { throw ros::Exception("No joint slew limit"); }

  //Initialize joint data
  updateJointStatus();
  for(int i=0; i<NUM_JOINTS; i++)
  {
   joint_target_pos[i] = joint_meas_pos[i];
   joint_target_torq[i] = 2;
  }
 } //constructed servo_controller

 //Destructor
 ~servo_controller()
 {
  serialClose(servo_port);
  ROS_INFO("Closed servo port");
 } //destructed servo_controller

 //Send commands to joint servos
 int send_command(int motor_ID, int des_pos, int des_torq)
 {
  //des_pos and des_torq must already be converted to
  //servo required integers:
  //position : 0~254
  //torque : 0~4 (high to low)
  int tmp_byte1 = 0;
  int tmp_byte2 = 0;
  int checksum = 0;

  tmp_byte1 = (des_torq << 5) | motor_ID;
  tmp_byte2 = des_pos;
  checksum = (tmp_byte1 ^ tmp_byte2) & 0x7f;

  //Send to serial port according to wCK protocol
  serialPutchar(servo_port, 0xff) //Header
  serialPutchar(servo_port, tmp_byte1); //Data1 (torque and motor id)
  serialPutchar(servo_port, tmp_byte2); //Target Position
  serialPutchar(servo_port, checksum); //Checksum

  //Servo will return a position and load from the motor
  if(serialDataAvail(servo_port) == 2) //Check first, getchar has 10sec block!
  {
   ROS_INFO("Command sent, response found");

   tmp_byte1 = serialGetchar(servo_port); //Load
   tmp_byte2 = serialGetchar(servo_port); //Position
  }

  return 1; //For now
/*  //Check position return to target position
  if(tmp_byte2 < des_pos+jointSlewLimit || tmp_byte2 > des_pos-jointSlewLimit)
  { return 1; }
  else //Assume for now something failed.
  { return 0; }*/
 }

 //Request joint servo status
 int request_status(int motor_ID, int &current_pos, int &current_load)
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

  //Flush buffer to make sure we get the target data only
  serialFlush(servo_port);
  //Send to serial port according to wCK protocol
  serialPutchar(servo_port, 0xff);
  serialPutchar(servo_port, tmp_byte1);
  serialPutchar(servo_port, tmp_byte2);
  serialPutchar(servo_port, checksum);

  data_avail = serialDataAvail(servo_port);
  if(data_avail == 2)
  {
   current_load = serialGetchar(servo_port);
   current_pos = serialGetchar(servo_port);
   if( current_load<0 || current_pos<0 || current_load>SAM3_MAX_COUNT || current_pos>SAM3_MAX_COUNT )
   {
    ROS_INFO("Response obtained are out of range! Pos = %d, load = %d", current_pos, current_load);
    return 1;
   }
   return 2;
  }
  else if(data_avail > 2)
  {
   ROS_INFO("Buffer contains %d, more than expected in status read", data_avail);
   return 1;
  }
  else
  {
   ROS_INFO("Buffer contains %d, less than expected in status read", data_avail);
   return 0;
  }
 }

 //Update joint measured position and load
 void updateJointStatus(void)
 {
  int measured_position;
  int measured_load;
  int request_rtn;

  for(int motor=0; motor<NUM_JOINTS; motor++)
  {
   //clear each loop
   measured_position = 0;
   measured_load = 0;
   request_rtn = 0;

   request_rtn = request_status(motor, measured_position, measured_load);
   if(request_rtn == 2)
   {
    joint_meas_pos[motor] = measured_position;
    joint_meas_load[motor] = measured_load;
   }
  }
 }

 //--CALLBACK--
 void updateJointTargets(const rpi_huno::ServoOdom& joint_cmds)
 {
  int tmp_pos_cmd, tmp_torq;
  for(int motor = 0; motor < NUM_JOINTS; motor++)
  {
   //Ensure commands are valid first and convert
   tmp_pos_cmd = int((joint_cmds.pos[motor])*SAM3_DEG_TO_CTS);
   if(tmp_pos_cmd > SAM3_MAX_COUNT)
   { tmp_pos_cmd = SAM3_MAX_COUNT; }
   else if(tmp_pos_cmd < 0)
   { tmp_pos_cmd = 0; }

   tmp_torq = int(joint_cmds.torqload[motor]);
   if(tmp_torq < 0)
   { tmp_torq = 0; }
   else if(tmp_torq > 4)
   { tmp_torq = 4; }

   joint_target_pos[motor] = tmp_pos_cmd;
   joint_target_torq[motor] = tmp_torq;
  }
 }
