#include "ros/ros.h"
#include "rpi_huno/ServoOdom.h"

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
#define LOOP_FREQ 10

#define NUM_JOINTS 16
#define SAM3_MAX_COUNT 254
#define SAM3_DEG_TO_CTS (255/269) //Convert degrees to counts

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
 rpi_huno::ServoOdom joint_status;
 int joint_target_pos[NUM_JOINTS];
 int joint_target_torq[NUM_JOINTS];

 //Other
 double jointSlewLimit;

 //======FUNCTIONS=========
 //Constructor
 JointController(ros::NodeHandle &n) : node(n),
  joint_commands(node.subscribe("joint_commands", 1, &JointController::updateJointTargets, this)),
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
  int measured_position, measured_load;
  int availData=0;
  joint_status.time_now = ros::Time::now();
  for(int i=0; i<NUM_JOINTS; i++)
  { request_status(i); } //request current joint angles
  ros::Duration(0.0012).sleep(); //wait 1.2 millisec for responses from servos

  availData = serialDataAvail(servo_port);
  if(availData)
  {
   ROS_INFO("Grabbing angles... %d left..", availData);
   for(int i=0; i<NUM_JOINTS; i++)
   {
    measured_position = 0;
    measured_load = 0;
    availData = read_joint_buffer(measured_position, measured_load);
    if(availData == 0)
    { throw ros::Exception("Init joint buffer error"); }
    ROS_INFO("%d left..", availData);

    joint_status.pos[i] = measured_position;
    joint_status.torqload[i] = measured_load;

    joint_target_pos[i] = measured_position;
    joint_target_torq[i] = 2;
   }
  }
 } //constructed JointController

 //Destructor
 ~JointController()
 {
  serialClose(servo_port);
  ROS_INFO("Closed servo port");
 } //destructed JointController

 //Send commands to joint servos
 void send_command(int motor_ID, int des_pos, int des_torq)
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

 int read_joint_buffer(int &current_pos, int &current_load)
 {
  int tmp_dataAvail = serialDataAvail(servo_port);
  if(tmp_dataAvail)
  {
   current_load = serialGetchar(servo_port);
   current_pos = serialGetchar(servo_port);
   tmp_dataAvail = serialDataAvail(servo_port);
   return tmp_dataAvail;
  }
  else
  { return 0; }
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

 //--RUN FUNCTION--
 void run(void)
 {
  int tmp_pos, tmp_load;
  int availData=0;
  joint_status.time_now = ros::Time::now();
  //TODO: Re-order sequence as necessary
  for(int motor=0; motor<NUM_JOINTS; motor++)
  { send_command(motor, joint_target_pos[motor], joint_target_torq[motor]); }

  ros::Duration(0.0012).sleep(); //wait for 1.2 millisec for responses from motors

  availData = serialDataAvail(servo_port);
  if(availData)
  {
   ROS_INFO("Run loop. Grabbing joint angles.. %d left..", availData);
   for(int motor=0; motor<NUM_JOINTS; motor++)
   {
    tmp_pos = 0;
    tmp_load = 0;
    availData = read_joint_buffer(tmp_pos, tmp_load);
    ROS_INFO("Run loop. %d left..", availData);

    joint_status.pos[i] = tmp_pos;
    joint_status.torqload[i] = tmp_load;
   }
  }

  //Publish status
  joint_angles.publish(joint_status);
 }
}; //end class

//=========MAIN=====
int main(int argc, char **argv)
{
 ros::init(argc, argv, "servo_control");
 ros::NodeHandle n;
 ros::Rate r(LOOP_FREQ);

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
