#ifndef HUNO_KINEMATICS_H
#define HUNO_KINEMATICS_H

#include "geometry_msgs/Pose.h"
#include "huno_globals.h"

#include <Eigen/Dense>

/********************************************************************
* Kinematics class for the RQ Huno.
* 
* author: E. Sasagawa
* 
* Calculates the end positions of each limb based on the current
* joint angles, utilizing Product of Exponentials.
* Also includes inverse kinematics methods. Assumes that the
* inverse kinematics are being calculated at the same pose the
* forward kinematics are calculated, since the jacobians will
* be calculated based on currently stored values.
*
* NOTE: Base frame is the Head frame
*
********************************************************************/
class C_HunoKinematics
{
public:
  /* Constructor */
  C_HunoKinematics(void);

  /*
   * Limb forward kinematics functions
   *
   * @param lowId : lowest Joint ID number in limb
   * @param highId : highest joint ID number in limb
   * @param thetasDeg : pointer to array of angles for each joint
   *                 in the limb (degrees)
   * @out   out_fkMatrix : 4x4 homogeneous transformation matrix 
   *                        from head frame to end effector frame
   *                        of limb in current configuration.
   */
  geometry_msgs::Pose LimbFK(int lowId, int highId, double *thetasD);

  /* 
   * Jacobian inverse function using SVD (though not quite necessary here 
   * since all Jacobians are square) 
   */
  bool LimbIK(hunolimb_t limb, double *inVelMps, double *outThetaDotDgs);

private:
  /* Reference configuration matrices for each limb */
  Eigen::Matrix4d g_LFoot0;
  Eigen::Matrix4d g_RFoot0;
  Eigen::Matrix4d g_LHand0;
  Eigen::Matrix4d g_RHand0;

  /* Jacobian matrices for each limb, truncated to size */
  Eigen::Matrix5d J_LFoot;
  Eigen::Matrix5d J_RFoot;
  Eigen::Matrix3d J_LHand;
  Eigen::Matrix3d J_RHand;

  /// The following variables are hard-coded for now, since they
  /// are not anticipated to change.
  /// TODO: However in future, may revisit when incorporating 
  /// calibration against errors.

  /* Joint twist axes (hard-coded) */
  Eigen::Matrix<double, 3, HUNO_NUM_JOINTS> omegaAll;

  /* 3D points on each twist axis (hard-coded) */
  Eigen::Matrix<double, 3, HUNO_NUM_JOINTS> qAll;

  /* Twist vectors for each joint (calculated and stored at initialization) */
  Eigen::Matrix<double, 6, HUNO_NUM_JOINTS> zetaAll;

  /* Offset angles for each joint (hard-coded) */
  Eigen::Matrix<double, HUNO_NUM_JOINTS, 1> thetaRefRAll;

  /*
   * Calculate screw matrix resulting from rotation of angle theta
   * 
   * @param jointId : Joint ID number
   * @param thetaRad : Radians of rotation about joint twist axis
   * @out   out_exp_XihatTheta : 4x4 containing exponential 
   *                              matrix of twist
   */
  Eigen::Matrix4d ExpXihatTheta(int jointId, double thetaR);

  /* 
   * Generate the adjoint matrix given 4x4 ExpXihatTheta
   * 
   * @param expXihatTheta
   * @return 6x6 adjoint matrix
   */
  Eigen::Matrix6d AdjointMatrix(Eigen::Matrix4d expXihatTheta);

  /*
   * checks the input set of angular velocities (RADIANS per sec)
   * against max limits and directions.
   * If a max limit is exceeded, the all joint velocities are
   * scaled down accordingly.
   *
   * @param outThetaDotDgs : pointer to array of double
   * @return None 
   */
  void BoundThetaDot( double *outThetaDotRps );
};

#endif
