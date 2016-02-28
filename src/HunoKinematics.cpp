#include "HunoKinematics.h"

C_HunoKinematics::C_HunoKinematics(void):
 /* Initialize Jacobians to zeros so they give no result if used */
 J_LFoot(Eigen::Matrix5d::Zero()),
 J_RFoot(Eigen::Matrix5d::Zero()),
 J_LHand(Eigen::Matrix3d::Zero()),
 J_RHand(Eigen::Matrix3d::Zero())
{
  Eigen::Matrix<double, HUNO_NUM_JOINTS, 3> tempMat;
  Eigen::Vector3d tempVec;

  /* Set g_0 reference configuration matrices */
  g_l_foot_0 << 1, 0, 0, -0.05375,
                0, 1, 0,  0.0115,
                0, 0, 1, -0.10615,
                0, 0, 0,  1;
  g_r_foot_0 << 1, 0, 0, -0.05375,
                0, 1, 0, -0.0115,
                0, 0, 1, -0.10615,
                0, 0, 0,  1;
  g_l_hand_0 << 1, 0, 0,  0.0462,
                0, 1, 0,  0.0672,
                0, 0, 1, -0.07575,
                0, 0, 0,  1;
  g_r_hand_0 << 1, 0, 0,  0.0462,
                0, 1, 0, -0.0672,
                0, 0, 1, -0.07575,
                0, 0, 0,  1;

  /* Set twist axes */ 
  tempMat << -1,  0,  0,
              0,  1,  0,
              0,  1,  0,
              0, -1,  0,
              1,  0,  0,
             -1,  0,  0,
              0, -1,  0,
              0, -1,  0,
              0,  1,  0,
             -1,  0,  0,
              0, -1,  0,
              1,  0,  0,
              0,  1,  0,
              0,  1,  0,
              1,  0,  0,
              0, -1,  0;
 
  // transpose to get column vectors for each joint
  omegaAll = tempMat.transpose();

  /* Set points of twist */
  tempMat <<  -0.06245,  0.0115, -0.0699,
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
 
  // transpose to get column vectors for each joint
  qAll = tempMat.transpose();

  /* Set reference angles */
  thetaRefRAll << 127.6,
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

  // convert to radians
  thetaRefRAll = thetaRefRAll * (3.14/180); 

  /* Calculate twist coordinates for each joint */
  // twist coord = [ (-omega X q), omega ]'
  // construct column by column
  for (int idx = 0; idx < HUNO_NUM_JOINTS; idx++)
  {
    tempvec = (-omegaAll.col(idx)).cross(qAll.col(idx));
    zetaAll.block<3,1>(0,idx) = tempVec;           // nu
    zetaAll.block<3,1>(3,idx) = omegaAll.col(idx); // omega
  }
} // end constructor

geometry::Pose C_HunoKinematics::LimbFK(int lowId, int highId, double *thetasD) //TODO fix output to msg
{
  Eigen::Matrix4d expXihatTheta = Eigen::Matrix4d::Identity();
  Eigen::Matrix<double, 6, 5> tempJacobian;
  tempJacobian.setZero(); // initialize temporary variable to zeros
  hunolimb_t limb;
  int multiplyCtr = 0;

  /* Identify limb */
  limb = IdentifyLimb(lowId);
  if (UnknownLimb == limb)
  { throw ros::Exception("LimbFK unknown lowId"); }
  else if (IdentifyLimb(highId) != limb)
  { throw ros::Exception("LimbFK jointId high/low mismatch"); }
 
  /* multiply each joint exponential matrix */
  Eigen::Matrix4d outLimbFk = Eigen::Matrix4d::Identity();
  for (int jointIdx = lowId; jointIdx <= highIdx; jointIdx++)
  {
    /* use previous expXihatTheta to generate adjoint matrix
       and fill next column of jacobian */
    tempJacobian.col(multiplyCtr) = AdjointMatrix(expXihatTheta) * zetaAll.col(jointIdx);   
    
    /* calculate for next joint */
    expXihatTheta =  ExpXihatTheta(jointIdx, ((*(thetasD+(jointIdx-lowId)))*DEG2RAD) );
    outLimbFk *= expXihatTheta;

    /* increment counter */
    multiplyCtr++;  
  } 

  /* apply limb specific operations */
  switch (limb)
  {
  case LeftLeg:
    if (multiplyCtr != 5)
    { throw ros::Exception("LimbFK multiplication LF failed"); }
    
    outLimbFk *= g_LFoot0;                  // apply configuration matrix
    J_LFoot = tempJacobian.block<5,5>(0,0); // save jacobian
    break;

  case RightLeg:
    if (multiplyCtr != 5)
    { throw ros::Exception("LimbFK multiplication RF failed"); }

    outLimbFk *= g_RFoot0;                  // apply configuration matrix
    J_RFoot = tempJacobian.block<5,5>(0,0); // save jacobian
    break;

  case LeftArm:
    if (multiplyCtr != 3)
    { throw ros::Exception("LimbFK multiplication LH failed"); }

    outLimbFk *= g_LHand0;                  // apply configuration matrix
    J_LHand = tempJacobian.block<3,3>(0,0); // save truncated jacobian
    break;

  case RightArm:
    if (multiplyCtr != 3)
    { throw ros::Exception("LimbFK multiplication RH failed"); }

    outLimbFk *= g_RHand0;                  // apply configuration matrix
    J_RHand = tempJacobian.block<3,3>(0,0); // save truncated jacobian
    break;

  default:
    // shouldn't be here since already checked limb
    break;
  }

  return outLimbFk; 
} // end LimbFK()


bool C_HunoKinematics::LimbIK(hunolimb_t limb, double *inVelMps, double *outThetaDotDgs)
{
  /* Compute inverse using SVD */
  bool ikResult = true;

  switch(limb)
  {
  case LeftLeg:
    {
      /* re-assign array parameters to Eigen Vectors */
      Eigen::Vector5d inEEVel = Eigen::Map<Vector5d>(inVelMps);
      Eigen::Vector5d outJointThetaDot = Eigen::Map<Vector5d>(outThetaDotDgs);

      /* perform SVD */
      Eigen::JacobiSVD<Matrix5d> svd(J_LFoot, ComputeThinU | ComputeThinV);
      outJointThetaDot = svd.solve( inEEVel ); // still in radians

      /* check answer */
      BoundThetaDot( outThetaDotDgs );          // still in radians, do sanity checks and speed checks

      /* convert to degrees per second */
      
      break;
    }
  case RightLeg:
    break;
  case LeftArm:
    break;
  case RightArm:
    break;
  default:
    // unknown limb
    ikResult = false;
    break;
  }

  return ikResult;
} // end LimbIK()

Eigen::Matrix4d C_HunoKinematics::ExpXihatTheta(int jointId, double thetaR)
{
  Eigen::Vector3d omega = omegaAll.col(jointId);
  Eigen::Vector3d q = qAll.col(jointId);
  double thetaCorrectedR = thetaR - thetaRefRAll(jointId);

  /* skew symmetric matrix */
  Eigen::Matrix3d omegahat;
  omegahat <<         0, -omega(2),  omega(1),
               omega(2),         0, -omega(0),
              -omega(1),  omega(0),         0;

  /* exponential of skew symmetrix matrix */
  Eigen::Matrix3d expOmegahatTheta;
  expOmegahatTheta = Eigen::Matrix3d::Identity() + omegahat*sin(thetaCorrectedR) + omegahat*omegahat*(1-cos(thetaCorrectedR));

  /* fill joint transformation matrix */
  Eigen::Matrix4d outExpXihatTheta;
  Eigen::Vector3d tempPosVec;
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

Eigen::Matrix6d AdjointMatrix(Eigen::Matrix4d expXihatTheta)
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
