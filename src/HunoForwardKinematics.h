#ifndef HUNO_FK_H
#define HUNO_FK_H

#include <Eigen/Dense>

/* Forward Kinematics class for the RQ Huno.
// Calculates the end positions of each limb based on the incoming joint angle messages,
// and publishes them.
// Utilizes Product of Exponentials formula for forward kinematics calculations.
// NOTE: Base frame is the Head frame
*/
class HunoForwardKinematics
{
public:
 /// Constructor
 HunoForwardKinematics();

 /// Limb forward kinematics functions
 /// @param low_id : lowest Joint ID number in limb
 /// @param high_id : highest joint ID number in limb
 /// @param thetas : array of angles for each joint in the limb (RADIANS)
 /// @out   out_fk_matrix : 4x4 homogeneous transformation matrix from head frame to
 ///                        end effector frame of limb in current configuration.
 Eigen::Matrix4f LimbFK(int low_id, int high_id, float *thetas);

private:
 /// Reference configuration matrices for each limb
 Eigen::Matrix4f g_l_foot_0;
 Eigen::Matrix4f g_r_foot_0;
 Eigen::Matrix4f g_l_hand_0;
 Eigen::Matrix4f g_r_hand_0;

 /// The following variables are hard-coded for now, since they are not anticipated to change.
 /// TODO: However in future, may revisit when incorporating calibration against errors.

 /// Joint twist axes
 Eigen::Matrix<float,3,16> omega_all;

 /// 3D points on each twist axis
 Eigen::Matrix<float,16,3> q_all;

 /// Offset angles for each joint
 Eigen::Matrix<float,16,1> ref_angles;

 /// Number of joints
 const int num_joints;

 /// Calculate screw matrix resulting from rotation of angle theta
 /// @param joint_id : Joint ID number
 /// @param theta : Radians of rotation about joint twist axis
 /// @out   out_exp_xihat_theta : 4x4 containing exponential matrix of twist
 Eigen::Matrix4f exp_xihat_theta(int joint_id, float theta);

}

#endif
