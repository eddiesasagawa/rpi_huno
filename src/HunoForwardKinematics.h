#ifndef HUNO_FK_CLASS
#define HUNO_FK_CLASS

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <Eigen/Dense>

class HunoForwardKinematics {
 ////NOTE: Base frame is the Head frame!
 public:
 //Variables:
 Eigen::Matrix4f gLFoot_0;
 Eigen::Matrix4f gRFoot_0;
 Eigen::Matrix4f gLHand_0;
 Eigen::Matrix4f gRHand_0;

 Eigen::Matrix<float,3,16> omega_all; // 3 x 16 matrix containing 16 column-vectors, each a unit vector of the corresponding joint twist axis
 Eigen::Matrix<float,16,3> q_all; // 16 x 3 matrix containing 16 rows of points, each a 3D point on the joint twist axis
 Eigen::Matrix<float,16,1> ref_angles; // Vector of reference angles for joints

 const int num_joints;

 //Functions:
 //Constructor
 HunoForwardKinematics():
  num_joints(16)
 {
  //Set g_0 reference configuration matrices
  gLFoot_0 << 1, 0, 0, -0.05375,
              0, 1, 0, 0.0115,
              0, 0, 1, -0.10615,
              0, 0, 0, 1;
  gRFoot_0 << 1, 0, 0, -0.05375,
              0, 1, 0, -0.0115,
              0, 0, 1, -0.10615,
              0, 0, 0, 1;
  gLHand_0 << 1, 0, 0, 0.0462,
              0, 1, 0, 0.0672,
              0, 0, 1, -0.07575,
              0, 0, 0, 1;
  gRHand_0 << 1, 0, 0, 0.0462,
              0, 1, 0, -0.0672,
              0, 0, 1, -0.07575,
              0, 0, 0, 1;

  //Set twist axes and points and reference angles
  omega_all << -1,  0,  0,  0, -1, -1,  0,  0,  0, -1,  0, -1,  0,  0, -1,  0,
                0, -1, -1, -1,  0,  0,  1,  1,  1,  0, -1,  0, -1,  1,  0,  1,
                0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0;

  q_all << -0.06245,  0.0115, -0.0699,
           -0.02515,  0.0115, -0.0699,
            0.02385,  0.0115, -0.0699,
           -0.01545,  0.0115, -0.09515,
           -0.05375,  0.0115, -0.09515,
           -0.06245, -0.0115, -0.0699,
           -0.02515, -0.0115, -0.0699,
            0.02385, -0.0115, -0.0699,
           -0.01545, -0.0115, -0.09515,
           -0.05375, -0.0115, -0.09515,
           -0.0263,   0.0292, -0.0266,
           -0.0263,   0.0672, -0.0266,
           -0.0263,   0.0672, -0.07575,
           -0.0263,  -0.0292, -0.0266,
           -0.0263,  -0.0672, -0.0266,
           -0.0263,  -0.0672, -0.07575;
  //q_all.transposeInPlace(); //Transpose so that points are columns now for copying to vectors.

  ref_angles << 134.5,
                134.5,
                224.5,
                134.5,
                134.5, //check theta_4
                134.5, //check theta_5
                134.5,
                 44.5,
                134.5,
                134.5, //check theta_9
                 44.5, //check theta_10
                 44.5,
                 44.5,
                224.5, //check theta_13
                224.5,
                224.5;
  ref_angles = ref_angles * (3.14/180); //Convert to radians

 } //End constructor

 // \brief Calculate exponential of angle theta about twist
 // \param joint_id : ID of joint that is calculated for
 // \param theta : angle of rotation about twist (RADIANS)
 // \out   exp_xihat_theta : 4x4 matrix containing exponential matrix of twist
 Eigen::Matrix4f exp_xihat_theta(int joint_id, float theta) {
  Eigen::Vector3f omega = omega_all.col(joint_id);
  Eigen::Vector3f q = (q_all.row(joint_id)).transpose();
  float theta_corrected = theta - ref_angles(joint_id);

  Eigen::Matrix3f omegahat;
  omegahat << 0, -omega(2), omega(1),
              omega(2), 0, -omega(0),
              -omega(1), omega(0), 0;

  Eigen::Matrix3f exp_omegahat_theta;
  exp_omegahat_theta = Eigen::Matrix3f::Identity() + omegahat*sin(theta_corrected) + omegahat*omegahat*(1-cos(theta_corrected));

  Eigen::Matrix4f out_exp_xihat_theta;
  Eigen::Vector3f temp_pos_vector;
  temp_pos_vector = (Eigen::Matrix3f::Identity()-exp_omegahat_theta)*(-1*q); // Check that omegahat does not need to be used
  out_exp_xihat_theta.block<3,3>(0,0) = exp_omegahat_theta;
  out_exp_xihat_theta.block<3,1>(0,3) = temp_pos_vector;
  out_exp_xihat_theta(3,0) =  0;
  out_exp_xihat_theta(3,1) =  0;
  out_exp_xihat_theta(3,2) =  0;
  out_exp_xihat_theta(3,3) =  1;

  if(joint_id > 12) {
   ROS_INFO("Joint %d : theta = %f, theta_corr = %f", joint_id, theta, theta_corrected);
   ROS_INFO("q = %f, %f, %f", q(0), q(1), q(2));
   ROS_INFO("omega = %f, %f, %f", omega(0), omega(1), omega(2));
   ROS_INFO("out_eXiHat = ");
   ROS_INFO(" %f, %f, %f, %f, ", out_exp_xihat_theta(0,0), out_exp_xihat_theta(0,1), out_exp_xihat_theta(0,2), out_exp_xihat_theta(0,3));
   ROS_INFO(" %f, %f, %f, %f, ", out_exp_xihat_theta(1,0), out_exp_xihat_theta(1,1), out_exp_xihat_theta(1,2), out_exp_xihat_theta(1,3));
   ROS_INFO(" %f, %f, %f, %f, ", out_exp_xihat_theta(2,0), out_exp_xihat_theta(2,1), out_exp_xihat_theta(2,2), out_exp_xihat_theta(2,3));
   ROS_INFO(" %f, %f, %f, %f; ", out_exp_xihat_theta(3,0), out_exp_xihat_theta(3,1), out_exp_xihat_theta(3,2), out_exp_xihat_theta(3,3));
  }

  return out_exp_xihat_theta;
 }

 //--Limb Forward Kinematics--
 // \brief Four functions below that calculate FK for each limb
 // \params thetaxx : angles for each joint in the limb (RADIANS)
 // \out    FKmatrix: 4x4 homogeneous transformation matrix from Head frame to end of limb
 Eigen::Matrix4f LeftFootFK(float theta00, float theta01, float theta02, float theta03, float theta04) {
  Eigen::Matrix4f out_LFootFK;
  out_LFootFK =   (exp_xihat_theta(0,theta00)) * (exp_xihat_theta(1,theta01))
                * (exp_xihat_theta(2,theta02)) * (exp_xihat_theta(3,theta03))
                * (exp_xihat_theta(4,theta04)) * gLFoot_0;

  return out_LFootFK;
 }

 Eigen::Matrix4f RightFootFK(float theta05, float theta06, float theta07, float theta08, float theta09) {
  Eigen::Matrix4f out_RFootFK;
  out_RFootFK =   (exp_xihat_theta(5,theta05)) * (exp_xihat_theta(6,theta06))
                * (exp_xihat_theta(7,theta07)) * (exp_xihat_theta(8,theta08))
                * (exp_xihat_theta(9,theta09)) * gRFoot_0;

  return out_RFootFK;
 }

 Eigen::Matrix4f LeftHandFK(float theta10, float theta11, float theta12) {
  Eigen::Matrix4f out_LHandFK;
  out_LHandFK =   (exp_xihat_theta(10,theta10)) * (exp_xihat_theta(11,theta11))
                * (exp_xihat_theta(12,theta12)) * gLHand_0;

  return out_LHandFK;
 }

 Eigen::Matrix4f RightHandFK(float theta13, float theta14, float theta15) {
  Eigen::Matrix4f out_RHandFK;
  out_RHandFK =   (exp_xihat_theta(13,theta13)) * (exp_xihat_theta(14,theta14))
                * (exp_xihat_theta(15,theta15)) * gRHand_0;

  return out_RHandFK;
 }

};

#endif
