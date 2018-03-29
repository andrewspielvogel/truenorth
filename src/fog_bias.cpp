/**
 * @file
 * @date March 2018
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of fog_bias.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <truenorth/helper_funcs.h>
#include <truenorth/fog_bias.h>



/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

FOGBias::FOGBias(Eigen::VectorXd k, Eigen::Matrix3d R0, float lat)
{
  
  // estimator gains
  ka_ = k(0);
  kE_ = k(1);
  kb_ = k(2);
  kab_= k(3);
  

  Eigen::Vector3d g_e(cos(lat),0,sin(lat));
  Eigen::Vector3d w_E(0,0,7.292150/100000.0);
  w_E_n = get_R_en(lat).transpose()*w_E;
  Eigen::Vector3d a_e = g_e + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0/9.81;

  gamma_ = fabs(w_E_n(2))/a_e.norm();

  a_b <<0,0,0;
  w_b <<0,0,0;

  w_E_north = R0.transpose().block<3,2>(0,0)*w_E_n.block<2,1>(0,0);

  acc_hat = R0.transpose()*get_R_en(lat)*a_e;
  start_ = 0;


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


  
  /**************************************************************
   * Sensor Bias and North Vector Estimator
   **************************************************************/



  Eigen::Matrix3d Kab;
  Kab<<kab_,0,0,0,kab_,0,0,0,10*kab_;

  Eigen::Matrix3d Kwb;
  Kwb<<kb_,0,0,0,kb_,0,0,0,5*kb_;
  
  Eigen::Vector3d dacc_hat   = -skew(ang - w_b - w_E_north)*acc_hat + skew(ang)*a_b - ka_*(acc_hat - acc);
  //Eigen::Vector3d dw_E_north = -skew(ang - gamma_*acc)*w_E_north - kE_*skew(acc)*acc_hat;
  Eigen::Vector3d dw_E_north = -skew(ang - gamma_*acc)*w_E_north - (Eigen::MatrixXd::Identity(3,3) - w_E_north.normalized()*w_E_north.normalized().transpose())*kE_*skew(acc)*acc_hat - w_E_north.normalized()*(w_E_north.norm() - w_E_n(0));
  //Eigen::Vector3d dw_E_north = -skew(ang - gamma_*acc)*w_E_north - kE_*skew(acc)*acc_hat - w_E_north.normalized()*(w_E_north.norm() - w_E_n(0));
  Eigen::Vector3d dw_b       = -Kwb*skew(acc)*acc_hat;  
  Eigen::Vector3d da_b       = Kab*skew(ang)*(acc_hat-acc);

  acc_hat   = acc_hat   + dt*dacc_hat;
  w_E_north = w_E_north + dt*dw_E_north;  
  w_b       = w_b       + dt*dw_b;
  a_b       = a_b       + dt*da_b;

  
}
