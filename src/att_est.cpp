/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of att_est.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <truenorth/helper_funcs.h>
#include <truenorth/att_est.h>



/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

AttEst::AttEst(Eigen::VectorXd k,Eigen::Matrix3d R_align, float lat, int hz) 
{
  
  // estimator gains
  kg_ = k(0);
  kw_ = k(1);
  ka_ = k(2);
  kE_ = k(3);
  kb_ = k(4);
  kab_= k(5);
  ka2_= 1;//k(6)
  
  hz_ = hz;
  lat_ = lat;

  // initialize usefull vectors
  double earthrate = 7.292150/100000.0;
  Eigen::Matrix3d R_en = get_R_en(lat_);

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_E(0,0,earthrate);
  Eigen::Vector3d a_e = g_e + skew(w_E)*skew(w_E)*g_e*6371.0*1000.0/9.81;
  Eigen::Vector3d e_e = w_E.cross(a_e);

  a_n = R_en.transpose()*a_e;
  e_n_ = R_en.transpose()*e_e.normalized();
  P_   = a_n.normalized()*a_n.normalized().transpose();
  R_ni = R_align;

  w_E_n = R_en.transpose()*w_E;
  gamma_ = w_E_n(2)/a_n.norm();
  w_E_n_mag_ = w_E_n(0);

  
  //prev_acc_ = R_ni.transpose()*a_n;

  h_error_<< 0,0,0;

  wearth_n_ = R_en.transpose()*w_E;
  east_est_n = R_en.transpose()*e_e;

  a_b << -0.005,-0.010,0;
  w_E_n(2) = 0;
  w_E_north = R_ni.transpose()*w_E_n;
  w_b <<  0,0,0;//0.0605/10000.0,0.1234/10000.0,-0.195/10000.0;
  acc_hat = R_ni.transpose()*a_n;
  acc_hat_ab = R_ni.transpose()*a_n;
  start_ = 0;

}

AttEst::~AttEst(void)
{
}

void AttEst::step(Eigen::Vector3d ang,Eigen::Vector3d acc, float dt)
{

  if (dt == 0 )
  {
    return;
  }

  if (!start_)
  {

    acc_hat = acc;
    start_ = 1;
    return;

  }


  /*
   *
   * a_b bias
   *
   */

  /*
  Eigen::Vector3d da2 = acc_hat_ab - acc;
  Eigen::Vector3d dacc2 = -skew(ang)*(acc-a_b) - ka2_*da2;
  Eigen::Vector3d dab = skew(ang)*da2;
  acc_hat_ab = acc_hat_ab + dt*dacc2;
  a_b = a_b + dt*kab_*dab;

  acc = acc - a_b;
  */
  
  Eigen::Matrix3d I;

  I << 1,0,0,0,1,0,0,0,1;

    
  P_ = R_ni.transpose()*a_n.normalized()*a_n.normalized().transpose()*R_ni;
  
  Eigen::Vector3d e(0,1,0);
  
  // Define local level (g_error_) and heading (h_error_) error terms
  g_error_ = kg_*skew(acc_hat-a_b)*R_ni.transpose()*a_n;
  h_error_ = P_*kw_*skew(w_E_north.normalized())*R_ni.transpose()*w_E_n;

  
  // update R_ni

  R_ni =  R_ni*((skew(g_error_ + h_error_ + ang - R_ni.transpose()*wearth_n_)*dt).exp());

  
  Eigen::Vector3d dacc_hat   = -skew(ang - w_b - w_E_north)*(acc_hat-a_b) - ka_*(acc_hat - acc);
  Eigen::Vector3d dw_E_north = -skew(ang - gamma_*acc)*w_E_north - kE_*skew(acc)*acc_hat;
  Eigen::Vector3d dw_b       = -kb_*skew(acc)*acc_hat;
  Eigen::Vector3d da_b       = kab_*skew(ang)*(acc_hat-acc);


  P_ = (acc_hat - a_b).normalized()*(acc_hat - a_b).normalized().transpose();

  acc_hat   = acc_hat   + dt*dacc_hat;
  w_E_north = w_E_north + dt*dw_E_north;
  w_b       = w_b       + dt*dw_b;
  a_b       = a_b       + dt*da_b;
  acc_hat   = acc_hat.normalized()*a_n.norm();
  w_E_north = w_E_n_mag_*(((I-P_)*w_E_north).normalized());
  
  



}
