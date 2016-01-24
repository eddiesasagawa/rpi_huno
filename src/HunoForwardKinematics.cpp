#include "HunoForwardKinematics.h"

/// Constructor
HunoForwardKinematics::HunoForwardKinematics() :
 num_joints(16)
{
  /// Set g_0 reference configuration matrices
  g_l_foot_0 << 1, 0, 0, -0.05375,
              0, 1, 0, 0.0115,
              0, 0, 1, -0.10615,
              0, 0, 0, 1;
  g_r_foot_0 << 1, 0, 0, -0.05375,
              0, 1, 0, -0.0115,
              0, 0, 1, -0.10615,
              0, 0, 0, 1;
  g_l_hand_0 << 1, 0, 0, 0.0462,
              0, 1, 0, 0.0672,
              0, 0, 1, -0.07575,
              0, 0, 0, 1;
  g_r_hand_0 << 1, 0, 0, 0.0462,
              0, 1, 0, -0.0672,
              0, 0, 1, -0.07575,
              0, 0, 0, 1;

  /// Set twist axes and points and reference angles
  omega_all << -1,  0,  0,  0,  1, -1,  0,  0,  0, -1,  0,  1,  0,  0,  1,  0,
                0,  1,  1, -1,  0,  0, -1, -1,  1,  0, -1,  0,  1,  1,  0, -1,
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

  ref_angles << 127.6,
                128.7,
                225.7,
                126.6,
                126.6,
                128.7,
                128.7,
                 38.0,
                130.8,
                127.6,
                 70.7, //theta_10
                 38.0,
                 38.0,
                187.7, //theta_13
                218.3,
                221.5;
  ref_angles = ref_angles * (3.14/180); //Convert to radians
} //End Constructor

Eigen::Matrix4d HunoForwardKinematics::exp_xihat_theta(int joint_id, double theta)
{
 Eigen::Vector3d omega = omega_all.col(joint_id);
  Eigen::Vector3d q = (q_all.row(joint_id)).transpose();
  double theta_corrected = theta - ref_angles(joint_id);

  /// exponential of skew symmetric matrix
  Eigen::Matrix3d omegahat;
  omegahat << 0, -omega(2), omega(1),
              omega(2), 0, -omega(0),
              -omega(1), omega(0), 0;

  Eigen::Matrix3d exp_omegahat_theta;
  exp_omegahat_theta = Eigen::Matrix3d::Identity() + omegahat*sin(theta_corrected) + omegahat*omegahat*(1-cos(theta_corrected));

  Eigen::Matrix4d out_exp_xihat_theta;
  Eigen::Vector3d temp_pos_vector;
  temp_pos_vector = (Eigen::Matrix3d::Identity()-exp_omegahat_theta)*(q); // omega x (-omega x q) = q
  out_exp_xihat_theta.block<3,3>(0,0) = exp_omegahat_theta;
  out_exp_xihat_theta.block<3,1>(0,3) = temp_pos_vector;
  out_exp_xihat_theta(3,0) =  0;
  out_exp_xihat_theta(3,1) =  0;
  out_exp_xihat_theta(3,2) =  0;
  out_exp_xihat_theta(3,3) =  1;

  return out_exp_xihat_theta;
} //End exp_xihat_theta

Eigen::Matrix4d HunoForwardKinematics::LimbFK(int low_id, int high_id, const double *thetas)
{
 if(low_id < 0 || low_id > 13 || high_id < 4 || high_id > 15)
 {
  throw ros::Exception("Invalid joint id range passed to LimbFK");
 }

 int mult_ctr = 0;

 Eigen::Matrix4d out_limb_fk = Eigen::Matrix4d::Identity();
 for(int joint_i = low_id; joint_i <= high_id; joint_i++)
 {
  out_limb_fk *= exp_xihat_theta(joint_i, ( (*(thetas+(joint_i-low_id))) * DEG2RAD ) );
  mult_ctr++;
 }

 if( isLeftLeg(high_id) )
 {
  if(mult_ctr != 5)
   throw ros::Exception("Multiplication LF failed");

  out_limb_fk *= g_l_foot_0;
 }
 else if( isRightLeg(high_id) )
 {
  if(mult_ctr != 5)
   throw ros::Exception("Multiplication RF failed");

  out_limb_fk *= g_r_foot_0;
 }
 else if( isLeftArm(high_id) )
 {
  if(mult_ctr != 3)
   throw ros::Exception("Multiplication LH failed");

  out_limb_fk *= g_l_hand_0;
 }
 else //is Right Arm
 {
  if(mult_ctr != 3)
   throw ros::Exception("Multiplication RH failed");

  out_limb_fk *= g_r_hand_0;
 }

 return out_limb_fk;
} // End LimbFK
