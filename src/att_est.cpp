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
<<<<<<< HEAD
  printf("       lat: %+18.12f (deg)  %+18.12f (rad)\n",params.lat * (180.0/M_PI), params.lat);

  float kg = 0.1;
  params.K_g << kg,0,0,0,kg,0,0,0,kg;
  params.K_north << 0,0,0,0,0,0,0,0,0;

  acc_g = new SO3Att(params);
  
=======

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

>>>>>>> be9c481c8a59c4425c867b48f468a1ecc00fc7ac
}

AttEst::~AttEst(void)
{
}

void AttEst::step(Eigen::Vector3d ang, Eigen::Vector3d acc, float dt)
{

  Eigen::Vector3d g_e(cos(params.lat),0,sin(params.lat));
  Eigen::Vector3d w_E(0,0,7.292150/100000.0);
  Eigen::Vector3d a_e = g_e*9.81 + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0;

  Eigen::Vector3d a_n  = get_R_en(params.lat).transpose()*a_e;

  
  /**************************************************************
   * Step Sensor Bias and North Vector Estimator
   **************************************************************/

  acc_g->step(ang, acc, ang, dt);

  //acc = acc_g->R_ni.transpose()*a_n;

  bias.step(ang, acc, dt);
  
  /**************************************************************
   * Step Attitude Estimator
   **************************************************************/

  att.step(ang-bias.w_b, acc-bias.a_b, bias.w_E_north, dt);
  //att.step(ang-bias.w_b, acc, bias.w_E_north, dt);
  
}
