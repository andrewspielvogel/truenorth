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
#include <unsupported/Eigen/MatrixFunctions>
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
  a_e = a_e*9.81;
  std::cout<<a_e.norm()<<"\n";

  gamma_ = fabs(w_E_n(2))/a_e.norm();

  a_b = params.acc_bias;
  w_b = params.ang_bias;

  w_E_n(2) = 0.0;

<<<<<<< HEAD
  w_E_north =  params.R_align.transpose()*params.R0.transpose()*get_R_en(params.lat)*skew(w_E)*a_e;

  gamma_ = w_E_north.norm();

  acc_hat = params.R0.transpose()*get_R_en(params.lat)*a_e;
=======
  // 2018-10-04 LLW added optional parameters to set ICs for acc_hat and w_E_north
  // set these to sentinel values in case the user does not provide these parameters
  // on the command line
  Eigen::Vector3d sentinel(-100.0,-100.0,-100.0);
  
  if (params.w_E_north == sentinel)
    {
      w_E_north = params.R_align.transpose()*params.R0.transpose()*w_E_n;
      parameters.w_E_north = w_E_north;
      printf("Using default computed values to initialize w_E_north.\n");      
    }
  else
    {
      w_E_north = params.w_E_north;
      printf("Using command line values to initialize w_E_north.\n");
      
    }

  if (params.acc_hat == sentinel)
    {
      acc_hat = params.R0.transpose()*get_R_en(params.lat)*a_e;
      parameters.acc_hat = acc_hat;
      printf("Using default computed values to initialize acc_hat.\n");            
    }
  else
    {
      acc_hat = params.acc_hat;
      printf("Using command line values to initialize acc_hat.\n");      
    }
  
      
>>>>>>> be9c481c8a59c4425c867b48f468a1ecc00fc7ac
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
  
  Eigen::Vector3d dacc_hat   = -skew(ang - w_b)*acc_hat + w_E_north+ skew(ang)*a_b - params.K_acc*da;

<<<<<<< HEAD
  float min = 1.0;  
=======
  
  // 2018-10-04 LLW  commented out bias adaptation supression for first 15 min of operatino
  // float min = 15.0;  
>>>>>>> be9c481c8a59c4425c867b48f468a1ecc00fc7ac

  
  Eigen::Vector3d dw_E_north = -params.K_E_n*da - skew(ang)*w_E_north;


  Eigen::Vector3d dw_b       = -params.K_ang_bias*skew(acc)*da;
  Eigen::Vector3d da_b       = params.K_acc_bias*skew(ang)*da;


  acc_hat   = acc_hat   + dt*dacc_hat;
  //w_E_north = w_E_north + dt*dw_E_north;
  w_E_north = (-skew(ang)*dt).exp()*w_E_north - dt*params.K_E_n*da;


<<<<<<< HEAD
  if (t_ > min*60.0){
    w_b       = w_b       + dt*dw_b;
    a_b       = a_b       + dt*da_b;
  }

  // if (t_ > 20.0*60.0){

  //   params.K_acc << 1.69,0.0,0.0,0.0,1.69,0.0,0.0,0.0,1.69;
  //   params.K_E_n << 0.0011,0.0,0.0,0.0,0.0011,0.0,0.0,0.0,0.0011;
  //   params.K_ang_bias << 0.00018,0.0,0.0,0.0,0.00019,0.0,0.0,0.0,0.00018;
  //   params.K_acc_bias << 2.48,0.0,0.0,0.0,2.48,0.0,0.0,0.0,2.48;

  // }
=======
  //w_E_north = w_E_north.normalized()*w_E_n(0);
  
  //  if (t_ > min*60.0){
    w_b       = w_b       + dt*dw_b;
    a_b       = a_b       + dt*da_b;
  //  }
>>>>>>> be9c481c8a59c4425c867b48f468a1ecc00fc7ac
    
  
}
