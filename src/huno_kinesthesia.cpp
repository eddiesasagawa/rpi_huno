#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"

#include "HunoLimbKinematics.h"
#include "rpi_huno/HunoLimbPoses.h"

#include "huno_globals.h"

#include <sstream>

/***************************************
 *** huno_kinesthesia
 **************************************
 * ROS Node
 * 
 * This node manages the kinesthetic sense
 * of rq_huno.
 * This includes limb kinematics and control.
 *
 ***************************************/
