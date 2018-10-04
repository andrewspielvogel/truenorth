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
  
  // 2018-10-04 LLW added additional stuff
  printf("  last_mod: %s\n",params.last_mod.c_str());
  printf("        hz: %d (s^-1)\n",params.hz);
  printf("       lat: %+18.12f (deg)  %+18.12f (rad)\n",params.lat * (180.0/M_PI), params.lat);  
  printf("    o_file: %s\n",params.o_file.c_str());
  printf("    i_file: %s\n",params.i_file.c_str());

  printf("        r0: [%+18.12f,%+18.12f,%+18.12f] (rpy)\n",rot2rph(params.R0)(0),rot2rph(params.R0)(1),rot2rph(params.R0)(2));
  printf("   r_align: [%+18.12f,%+18.12f,%+18.12f] (rpy)\n",rot2rph(params.R_align)(0),rot2rph(params.R_align)(1),rot2rph(params.R_align)(2));
  printf("     k_acc: [%+18.12f,%+18.12f,%+18.12f] (diag)\n",params.K_acc(0,0),params.K_acc(1,1),params.K_acc(2,2));
  printf("k_ang_bias: [%+18.12f,%+18.12f,%+18.12f] (diag)\n",params.K_ang_bias(0,0),params.K_ang_bias(1,1),params.K_ang_bias(2,2));
  printf("k_acc_bias: [%+18.12f,%+18.12f,%+18.12f] (diag)\n",params.K_acc_bias(0,0),params.K_acc_bias(1,1),params.K_acc_bias(2,2));
  printf("     k_E_n: [%+18.12f,%+18.12f,%+18.12f] (diag)\n",params.K_E_n(0,0),params.K_E_n(1,1),params.K_E_n(2,2));
  printf("       k_g: [%+18.12f,%+18.12f,%+18.12f] (diag)\n",params.K_g(0,0),params.K_g(1,1),params.K_g(2,2));
  printf("   k_north: [%+18.12f,%+18.12f,%+18.12f] (diag)\n",params.K_north(0,0),params.K_north(1,1),params.K_north(2,2));

  // 2018-10-04 LLW added additional stuff  
  printf("  ang_bias: [%+18.12f,%+18.12f,%+18.12f] (vec)\n",params.ang_bias(0),params.ang_bias(1),params.ang_bias(2));
  printf("  acc_bias: [%+18.12f,%+18.12f,%+18.12f] (vec)\n",params.acc_bias(0),params.acc_bias(1),params.acc_bias(2));
  printf("  mag_bias: [%+18.12f,%+18.12f,%+18.12f] (vec)\n",params.mag_bias(0),params.mag_bias(1),params.mag_bias(2));
  printf(" w_E_north: [%+18.12f,%+18.12f,%+18.12f] (vec)\n",params.w_E_north(0),params.w_E_north(1),params.w_E_north(2));
  printf("   acc_hat: [%+18.12f,%+18.12f,%+18.12f] (vec)\n",params.acc_hat(0),params.acc_hat(1),params.acc_hat(2));
  printf(" Estimator State Variable Initial Values  --------------------------------------------\n");
  printf("  ang_bias: [%+18.12f,%+18.12f,%+18.12f] variable (vec)\n",bias.w_b(0),bias.w_b(1),bias.w_b(2));
  printf("  acc_bias: [%+18.12f,%+18.12f,%+18.12f] variable (vec)\n",bias.a_b(0),bias.a_b(1),bias.a_b(2));
  //  printf("  mag_bias: [%+18.12f,%+18.12f,%+18.12f] variable (vec)\n",params.mag_bias(0),params.mag_bias(1),params.mag_bias(2));  
  printf(" w_E_north: [%+18.12f,%+18.12f,%+18.12f] variable (vec)\n",bias.w_E_north(0),bias.w_E_north(1),bias.w_E_north(2));
  printf("   acc_hat: [%+18.12f,%+18.12f,%+18.12f] variable (vec)\n",bias.acc_hat(0),bias.acc_hat(1),bias.acc_hat(2));  

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
