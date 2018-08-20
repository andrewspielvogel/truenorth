/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of att_est.h.
 *
 */

#include <Eigen/Core>
#include <truenorth/att_est.h>
#include <iostream>


/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

AttEst::AttEst(config_params parameters) : bias(parameters), att(parameters)
{

  params = parameters;
  
  printf("*************************\nATTITUDE ESTIMATOR USING GAINS:\n*************************\n");
  printf("        r0: [%f,%f,%f] (rpy)\n",rot2rph(params.R0)(0),rot2rph(params.R0)(1),rot2rph(params.R0)(2));
  printf("   r_align: [%f,%f,%f] (rpy)\n",rot2rph(params.R_align)(0),rot2rph(params.R_align)(1),rot2rph(params.R_align)(2));
  printf("     k_acc: [%f,%f,%f] (diag)\n",params.K_acc(0,0),params.K_acc(1,1),params.K_acc(2,2));
  printf("k_ang_bias: [%f,%f,%f] (diag)\n",params.K_ang_bias(0,0),params.K_ang_bias(1,1),params.K_ang_bias(2,2));
  printf("k_acc_bias: [%f,%f,%f] (diag)\n",params.K_acc_bias(0,0),params.K_acc_bias(1,1),params.K_acc_bias(2,2));
  printf("     k_E_n: [%f,%f,%f] (diag)\n",params.K_E_n(0,0),params.K_E_n(1,1),params.K_E_n(2,2));
  printf("       k_g: [%f,%f,%f] (diag)\n",params.K_g(0,0),params.K_g(1,1),params.K_g(2,2));
  printf("   k_north: [%f,%f,%f] (diag)\n",params.K_north(0,0),params.K_north(1,1),params.K_north(2,2));
  printf("       lat: %f (degrees)\n",params.lat);
  
}

AttEst::~AttEst(void)
{
}

void AttEst::step(Eigen::Vector3d ang, Eigen::Vector3d acc, float dt)
{


  
  /**************************************************************
   * Step Sensor Bias and North Vector Estimator
   **************************************************************/

  
  bias.step(ang, acc, dt);
  
  /**************************************************************
   * Step Attitude Estimator
   **************************************************************/

  //att.step(ang-bias.w_b, bias.acc_hat-bias.a_b, bias.w_E_north, dt);
  att.step(ang-bias.w_b, acc-bias.a_b, bias.w_E_north, dt);
  
}
