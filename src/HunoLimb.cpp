#include "HunoLimb.h"

C_HunoLimb::C_HunoLimb(hunolimb_t limb, int numJoints, const double *p_thetasRef_M, const double *p_omegas_M, const double *p_qs_M, const double *p_g0Mat_M) : 
    m_limbType( limb ),
    m_numJoints( numJoints ),
    m_thetasRef_M( Eigen::Map<Eigen::MatrixXd>(p_thetasRef_M, 3, numJoints) ),
    m_omegas_M( Eigen::Map<Eigen::MatrixXd>(p_omegas_M, 3, numJoints) ),
    m_qs_M( Eigen::Map<Eigen::MatrixXd>(p_qs_M, 3, numJoints) ),
    m_zetas_M( Eigen::MatrixXd::Zero(3, numJoints) ),
    m_g0Mat_M( Eigen::Map<Eigen::Matrix4d>(p_g0Mat_M) ),
    m_isJacobianLocked( true ),
    m_jacobian_M( Eigen::MatrixXd::Zero(numJoints, numJoints) )
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
