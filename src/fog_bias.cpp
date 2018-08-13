/**
 * @file
 * @date March 2018
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of fog_bias.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <helper_funcs/helper_funcs.h>
#include <truenorth/fog_bias.h>
#include <iostream>


/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

FOGBias::FOGBias(config_params parameters)
{
  
  params = parameters;
  

  Eigen::Vector3d g_e(cos(params.lat),0,sin(params.lat));
  Eigen::Vector3d w_E(0,0,7.292150/100000.0);
  w_E_n = get_R_en(params.lat).transpose()*w_E;
  Eigen::Vector3d a_e = g_e + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0/9.81;

  gamma_ = fabs(w_E_n(2))/a_e.norm();

  a_b = params.acc_bias;
  w_b = params.ang_bias;

  w_E_north = params.R0.transpose().block<3,2>(0,0)*w_E_n.block<2,1>(0,0);

  acc_hat = params.R0.transpose()*get_R_en(params.lat)*a_e;
  start_ = 0;

  t_ = 0;

}

FOGBias::~FOGBias(void)
{
}

void FOGBias::step(Eigen::Vector3d ang,Eigen::Vector3d acc,float dt)
{
  
  if (dt == 0 )
  {
    return;
  }

  // wait until you have a full mag reading to give an initial guess of the heading
  if (start_<1)
  {

    acc_hat = acc - a_b;
    start_ += 1;

    return;

  }


  t_ += dt;

  /**************************************************************
   * Sensor Bias and North Vector Estimator
   **************************************************************/

  Eigen::Vector3d da = acc_hat - acc;
  
  Eigen::Vector3d dacc_hat   = -skew(ang - w_b - w_E_north)*acc_hat + skew(ang)*a_b - params.K_acc*da;

  //Eigen::Vector3d dw_E_north = -skew(ang - gamma_*acc.normalized())*w_E_north - params.K_E_n*skew(acc)*da;

  Eigen::Vector3d dw_E_north = -skew(ang - gamma_*acc.normalized())*w_E_north - params.K_E_n*skew(acc)*da - (w_E_north.norm()-w_E_n(0))*w_E_north.normalized();


  Eigen::Vector3d dw_b       = -params.K_ang_bias*skew(acc)*da;
  Eigen::Vector3d da_b       = params.K_acc_bias*skew(ang)*da;


  acc_hat   = acc_hat   + dt*dacc_hat;
  w_E_north = w_E_north + dt*dw_E_north;


  if (t_ > 15.0*60.0){
    w_b       = w_b       + dt*dw_b;
    a_b       = a_b       + dt*da_b;
  }
  
}
