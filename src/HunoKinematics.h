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
 C_HunoKinematics();

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
 geometry_msgs::Pose LimbFK(int lowId, int highId, float *thetasDeg);

 /* 
  * Jacobian inverse function using SVD (though not quite necessary here 
  * since all Jacobians are square) 
  */
 bool LimbIK(hunolimb_t limb, float *inVelMps, float *outThetaDotDgs);

private:
 /* Reference configuration matrices for each limb */
 Eigen::Matrix4f g_LFoot0;
 Eigen::Matrix4f g_RFoot0;
 Eigen::Matrix4f g_LHand0;
 Eigen::Matrix4f g_RHand0;

 /* Jacobian matrices for each limb, truncated to size */
 Eigen::Matrix5f J_LLeg;
 Eigen::Matrix5f J_RLeg;
 Eigen::Matrix3f J_LArm;
 Eigen::Matrix3f J_RArm;

 /// The following variables are hard-coded for now, since they
 /// are not anticipated to change.
 /// TODO: However in future, may revisit when incorporating 
 /// calibration against errors.

 /* Joint twist axes (hard-coded) */
 Eigen::Matrix<float, HUNO_NUM_JOINTS, 3> omegaAll;

 /* 3D points on each twist axis (hard-coded) */
 Eigen::Matrix<float, HUNO_NUM_JOINTS, 3> qAll;

 /* Twist vectors for each joint (calculated and stored at initialization) */
 Eigen::Matrix<float, HUNO_NUM_JOINTS, 6> zetaAll;

 /* Offset angles for each joint (hard-coded) */
 Eigen::Matrix<float, HUNO_NUM_JOINTS, 1> refAnglesDeg;

 /*
  * Calculate screw matrix resulting from rotation of angle theta
  * 
  * @param jointId : Joint ID number
  * @param thetaRad : Radians of rotation about joint twist axis
  * @out   out_exp_XihatTheta : 4x4 containing exponential 
  *                              matrix of twist
  */
 Eigen::Matrix4f Exp_XihatTheta(int jointId, float thetaRad);

 /*
  * TODO: Save the Jacobian matrices for each limb. Define an enum
 for each limb and switch based on that in LimbFK. Also generate each
 column of jacobian during for loop, truncating as necessary.
  */
  
 /* 
  * Jacobian inverse function using SVD (though not quite necessary here 
  * since all Jacobians are square) 
  */
 
}

#endif
