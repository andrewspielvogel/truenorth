/**
 * @file
 * @date March 2018
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of so3_att.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include <truenorth/helper_funcs.h>
#include <truenorth/so3_att.h>




/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

SO3Att::SO3Att(Eigen::VectorXd k,Eigen::Matrix3d R_align, float lat) 
{
  
  // estimator gains
  kg_ = k(0);
  kn_ = k(1);

  lat_ = lat;

  // initialize usefull vectors
  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_E(0,0,7.292150/100000.0);
  Eigen::Vector3d a_e = g_e + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0/9.81;

  R_ni = R_align;
  
  w_E_n_ = get_R_en(lat_).transpose()*w_E;
  a_n_  = get_R_en(lat_).transpose()*a_e;

  P_ = R_ni.transpose()*a_n_.normalized()*a_n_.normalized().transpose()*R_ni;


}

SO3Att::~SO3Att(void)
{
}

void SO3Att::step(Eigen::Vector3d ang,Eigen::Vector3d g, Eigen::Vector3d north, float dt)
{

  
  if (dt == 0 )
  {
    return;
  }

  
  /**************************************************************
   * Attitude Estimator
   **************************************************************/

  
  P_ = R_ni.transpose()*a_n_.normalized()*a_n_.normalized().transpose()*R_ni;

  // Define local level (g_error_) and heading (h_error_) error terms
  g_error_ = kg_*skew((g).normalized())*R_ni.transpose()*a_n_.normalized();
  h_error_ = P_*kn_*skew(north.normalized())*R_ni.transpose().block<3,1>(0,0);
  
  R_ni     =  R_ni*((skew(g_error_ + h_error_ + ang - R_ni.transpose()*w_E_n_)*dt).exp());
  
}
