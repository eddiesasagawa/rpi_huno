#include "HunoLimb.h"

C_HunoLimb::C_HunoLimb(hunolimb_t limb, int numJoints, const double *p_thetasRef_M, const double *p_omegas_M, const double *p_qs_M, const double *p_g0Mat_M, const double *p_jointVelLimits_M) : 
    m_limbType( limb ),
    m_numJoints( numJoints ),
    m_thetasRef_M( Eigen::Map<Eigen::VectorXd>(p_thetasRef_M, numJoints) ),
    m_omegas_M( Eigen::Map<Eigen::MatrixXd>(p_omegas_M, 3, numJoints) ),
    m_qs_M( Eigen::Map<Eigen::MatrixXd>(p_qs_M, 3, numJoints) ),
    m_zetas_M( Eigen::MatrixXd::Zero(3, numJoints) ),
    m_g0Mat_M( Eigen::Map<Eigen::Matrix4d>(p_g0Mat_M) ),
    m_isJacobianLocked( true ),
    m_jacobian_M( Eigen::MatrixXd::Zero(numJoints, numJoints) ),
    m_jointVelLimits_M( Eigen::Map<Eigen::VectorXd>(p_jointVelLimits_M, numJoints) )
{
  /* Calculate twist coordiantes for each joint */
  // twist coord = [ (-omega X q), omega ]'
  // construct column by column
  for (int idx = 0; idx < m_numJoints; idx++)
  {
    m_zetas_M.block<3,1>(0,idx) = ( (-m_omegas_M.col(idx)).cross(m_qs_M.col(idx)) ); // nu
    m_zetas_M.block<3,1>(3,idx) = m_omegas_M.col(idx); // omega
  }
} // end constructor  

geometry_msgs::Pose C_HunoLimb::ForwardKinematics(const double *thetas_C)
{ //TODO fix output to msg
  geometry_msgs::Pose outLimbPose;
  Eigen::Vector3d rot_axis = Eigen::Vector3d::Zero();
  double rot_angle;

  Eigen::Matrix4d limbTF = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d expXihatTheta = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 5> tempJacobian;
  tempJacobian.setZero(); // initialize temporary variable to zeros
  int limbCtr = 0;

  /* Lock jacobian matrix while being formed */
  m_isJacobianLocked = true;

  /* Calculate joint transformation matrices sequentially and populate jacobian */
  for (int jointIdx = 0; jointIdx < m_numJoints; jointIdx++)
  {
    /* use previous expXihatTheta to generate adjoint matrix
       and fill next column of jacobian */
    tempJacobian.col(limbCtr) = AdjointMatrix(expXihatTheta) * m_zetas_M.col(jointIdx);

    /* calculate for next joint */
    expXihatTheta = ExpXihatTheta(jointIdx, (*(thetas_C+jointIdx))*DEG2RAD);
    limbTF *= expXihatTheta;

    /* increment counter */
    limbCtr++; 
  }
  
  if (limbCtr == m_numJoints)
  {
    limbTF *= m_g0Mat_M; // apply configuration matrix 
    m_jacobian_M = tempJacobian.block(0,0, m_numJoints, m_numJoints); // save jacobian
    
    m_isJacobianLocked = false; // unlock jacobian matrix
  }
  else
  { // reset matrices since something didn't add up.
    limbTF.setZero();
    m_jacobian_M.setZero();
  }

  /* save limb transformation matrix into pose message */
  outLimbPose.position.x = limbTF(0,3);
  outLimbPose.position.y = limbTF(1,3);
  outLimbPose.position.z = limbTF(2,3);

  //back out rotation unit vector and angle from rotation matrix
  rot_angle = acos( ( ((limbTF.block<3,3>(0,0)).trace())-1 ) /2 );
  if (sin(rot_angle) != 0)
  {
    rot_axis(0) = (limbTF(2,1)-limbTF(1,2)) / (2*sin(rot_angle));
    rot_axis(1) = (limbTF(0,2)-limbTF(2,0)) / (2*sin(rot_angle));
    rot_axis(2) = (limbTF(1,0)-limbTF(0,1)) / (2*sin(rot_angle));
  }
  // else rot_axis is zeroes

  outLimbPose.orientation.x = rot_axis(0)*sin(rot_angle/2);
  outLimbPose.orientation.y = rot_axis(1)*sin(rot_angle/2);
  outLimbPose.orientation.z = rot_axis(2)*sin(rot_angle/2);
  outLimbPose.orientation.w = cos(rot_angle/2);

  return outLimbPose; 
} // end ForwardKinematics()

bool C_HunoLimb::InverseKinematics(const double *p_inEEVel_C, double *p_outThetasDot_C)
{
  bool ikResult = true;

  if (m_isJacobianLocked) // jacobian matrix not fully formed
  {
    ikResult = false;
  }
  else
  {
    /* Define temporary output vector */
    Eigen::VectorXd tempThetasDot_M(m_numJoints);
    tempThetasDot_M.setZero();

    /* Map array of double to Eigen Vector for manipulation, converting to MU */
    Eigen::VectorXd inEEVel_M = Eigen::Map<VectorXd>(p_inEEVel_C, m_numJoints) / 1000;  

    /* Perform SVD */
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m_jacobian_M, ComputeThinU | ComputeThinV);

    tempThetasDot_M = svd.solve( inEEVel_M );

    /* Check answer and trim as necessary */
    BoundThetaDot( tempThetasDot_M );

    /* Assign to output array */
    Eigen::Map<VectorXd>(p_outThetasDot_C, m_numJoints) = tempThetasDot_M * RAD2DEG;
  }

  return ikResult;
} // end InverseKinematics() 

Eigen::Matrix4d C_HunoLimb::ExpXihatTheta(int jointIdx, double &theta_M)
{
  Eigen::Vector3d omega = m_omegas_M.col(jointIdx);
  Eigen::Vector3d q = m_qs_M.col(jointIdx);
  double thetaCorrected_M = theta_M - m_thetasRef_M(jointIdx);

  Eigen::Matrix3d omegahat;
  Eigen::Matrix3d expOmegahatTheta;
  Eigen::Matrix4d outExpXihatTheta;
  Eigen::Vector3d tempPosVec;

  /* skew-symmetric matrix */
  omegahat <<        0, -omega(2),  omega(1),
              omega(2),         0, -omega(0),
             -omega(1),  omega(0),         0;

  /* exponential of skew symmetric matrix */
  expOmegahatTheta = Eigen::Matrix3d::Identity() + omegahat*sin(thetaCorrected_M) + omegahat*omegahat*(1-cos(thetaCorrected_M));

  /* fill joint transformation matrix */
  // omega X (-omega X q) = q
  tempPosVec = (Eigen::Matrix3d::Identity()-expOmegahatTheta)*q;
  outExpXihatTheta.block<3,3>(0,0) = expOmegahatTheta;
  outExpXihatTheta.block<3,1>(0,3) = tempPosVec;
  outExpXihatTheta(3,0) = 0;
  outExpXihatTheta(3,1) = 0;
  outExpXihatTheta(3,2) = 0;
  outExpXihatTheta(3,3) = 1;

  return outExpXihatTheta;
} // end ExpXihatTheta()

Eigen::Matrix6d C_HunoLimb::AdjointMatrix(const Eigen::Ref<const Eigen::Matrix4d> &expXihatTheta)
{
  Eigen::Matrix6d outAdMat = Eigen::Matrix6d::Zero();

  Eigen::Matrix3d pHat <<                 0, -expXihatTheta(3,2),  expXihatTheta(3,1),
                          expXihatTheta(3,2),                  0, -expXihatTheta(3,0),
                         -expXihatTheta(3,1), expXihatTheta(3,0),                   0;
  
  outAdMat.block<3,3>(0,0) = expXihatTheta.block<3,3>(0,0);
  outAdMat.block<3,3>(3,3) = expXihatTheta.block<3,3>(0,0);
  outAdMat.block<3,3>(0,3) = pHat*(expXihatTheta.block<3,3>(0,0));

  return outAdMat;
} // end AdjointMatrix()

void C_HunoLimb::BoundJointVels( Eigen::Ref<Eigen::VectorXd> &thetaDots_M )
{
  /* check if any angular velocities exceed limits
     and scale if necessary */
  

} // end BoundJointVels


