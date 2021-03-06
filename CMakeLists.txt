cmake_minimum_required(VERSION 2.8.3)
project(rpi_huno)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  sensor_msgs
  geometry_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   HunoLimbPoses.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
#   std_msgs  # Or other packages containing msgs
#   sensor_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rpi_huno
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs message_runtime
  DEPENDS Eigen system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN_INCLUDE_DIRS})

## Declare a cpp library
# add_library(rpi_huno
#   src/${PROJECT_NAME}/rpi_huno.cpp
# )

## Declare a cpp executable
# add_executable(rpi_huno_node src/rpi_huno_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(rpi_huno_node rpi_huno_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(rpi_huno_node
#   ${catkin_LIBRARIES}
# )

## Joint control nodes
# check for wiringPi header file to see if running on actual hardware
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES(wiringSerial.h USING_PI)
if( USING_PI )
  add_executable(joint_control src/joint_control.cpp)
  target_link_libraries(joint_control ${catkin_LIBRARIES} wiringPi)
  add_dependencies(joint_control rpi_huno_generate_messages_cpp)

  add_executable(joint_passive_mode src/joint_passive_mode.cpp)
  target_link_libraries(joint_passive_mode ${catkin_LIBRARIES} wiringPi)
  add_dependencies(joint_passive_mode rpi_huno_generate_messages_cpp)
endif( USING_PI )

# Joint Control Simulator node
# this can be used on real or simulated environment
add_executable(joint_control_simulator src/joint_control_simulator.cpp)
target_link_libraries(joint_control_simulator ${catkin_LIBRARIES})
add_dependencies(joint_control_simulator rpi_huno_generate_messages_cpp)

# add global function library
add_library(rpi_huno src/common_functions.cpp)

add_executable(huno_kinematics src/huno_kinematics.cpp src/common_functions.cpp)
target_link_libraries(huno_kinematics ${catkin_LIBRARIES} rpi_huno)
add_dependencies(huno_kinematics rpi_huno_generate_messages_cpp)

add_executable(joint_odom_conversion src/joint_odom_for_urdf.cpp)
target_link_libraries(joint_odom_conversion ${catkin_LIBRARIES})
add_dependencies(joint_odom_conversion rpi_huno_generate_messages_cpp)

add_executable(huno_forwardkinematics src/huno_forwardkinematics.cpp src/HunoForwardKinematics.cpp src/common_functions.cpp)
target_link_libraries(huno_forwardkinematics ${catkin_LIBRARIES} rpi_huno)
add_dependencies(huno_forwardkinematics rpi_huno_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rpi_huno rpi_huno_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rpi_huno.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
