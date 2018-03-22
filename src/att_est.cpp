/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of att_est.h.
 *
 */

#include <Eigen/Core>
#include <truenorth/att_est.h>



/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

AttEst::AttEst(Eigen::VectorXd k,Eigen::Matrix3d R_align, float lat) : bias(k.tail(4),R_align,lat), att(k.head(2),R_align,lat)
{
  
  printf("USING GAINS:\n");
  printf("kg: %f\n",k(0));
  printf("kn: %f\n",k(1));
  printf("ka: %f\n",k(2));
  printf("kE: %.10f\n",k(3));
  printf("kb: %.10f\n",k(4));
  printf("kab: %f\n",k(5));
  
}

AttEst::~AttEst(void)
{
}

void AttEst::step(Eigen::Vector3d ang, Eigen::Vector3d acc, float dt)
{


  
  /**************************************************************
   * Sensor Bias and North Vector Estimator
   **************************************************************/

  bias.step(ang, acc, dt);
  
  /**************************************************************
   * Attitude Estimator
   **************************************************************/

  
  att.step(ang-bias.w_b, bias.acc_hat-bias.a_b, bias.w_E_north, dt);
  
}
