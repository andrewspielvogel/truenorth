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
#include <helper_funcs/helper_funcs.h>
#include <truenorth/so3_att.h>
#include <iostream>




/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

SO3Att::SO3Att(config_params parameters) 
{
  
  params = parameters;

  // initialize usefull vectors
  Eigen::Vector3d g_e(cos(params.lat),0,sin(params.lat));
  Eigen::Vector3d w_E(0,0,7.292150/100000.0);
  Eigen::Vector3d a_e = g_e + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0/9.81;

  R_ni = params.R0*params.R_align;

  w_E_n_ = get_R_en(params.lat).transpose()*w_E;
  a_n_  = get_R_en(params.lat).transpose()*a_e;

  P_ = R_ni.transpose()*a_n_.normalized()*a_n_.normalized().transpose()*R_ni;
  h_error_ << 0,0,1;

}

SO3Att::~SO3Att(void)
{
}

void SO3Att::step(Eigen::Vector3d w_i,Eigen::Vector3d g_i, Eigen::Vector3d north_i, float dt)
{

  
  if (dt == 0 )
  {
    return;
  }

  
  /**************************************************************
   * Attitude Estimator
   **************************************************************/
  Eigen::Vector3d north_n(0,1,0);

  P_ = R_ni.transpose()*a_n_.normalized()*a_n_.normalized().transpose()*R_ni;

  // Define local level (g_error_) and heading (h_error_) error terms
  g_error_ = skew(g_i.normalized())*R_ni.transpose()*a_n_.normalized();
  h_error_ = P_*(skew(north_i.normalized())*R_ni.transpose()*north_n);

  
  R_ni     =  R_ni*((skew(params.K_g*g_error_ + params.K_north*h_error_ + w_i - R_ni.transpose()*w_E_n_)*dt).exp());
  
}
