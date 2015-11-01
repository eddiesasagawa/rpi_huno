# rpi_huno
ROS implementation on the Raspberry Pi for RQ Huno control

login user: pi
static ip: 10.42.0.45

===TODO===
-> Determine min and max arm radii r1 and r2, centered around q14, q11
-> Code inverse kinematics and control alg for arms.
-> Implement IK for legs and test output to check if legs can lift RPi successfully.
-> Workspace control algorithm to stand and sit.
-> Adjust power switch for easier access.
-> Setup multimaster_fkie to publish joint odom to laptop
-> Setup rviz for URDF visualization on laptop
   -> robot_control_center project on laptop
-> Add IR measurements from eyes
-> Add IMU
-> Add RPi camera
