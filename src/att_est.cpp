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
  kf_ = k(2);

  hz_ = hz;
  lat_ = lat;

  
  double earthrate = 15.04*M_PI/180.0/3600.0;
  Eigen::Matrix3d R_en = get_R_en(lat_);

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_e(0,0,earthrate);
  Eigen::Vector3d a_e = g_e + skew(w_e)*skew(w_e)*g_e*6371.0*1000.0/9.81;
  Eigen::Vector3d e_e = w_e.cross(a_e);

  a_n = R_en.transpose()*a_e;
  e_n_ = R_en.transpose()*e_e.normalized();
  P_   = a_n.normalized()*a_n.normalized().transpose();
  R_ni = R_align;


  prev_acc_ = R_ni.transpose()*a_n;

  h_error_<< 0,0,0;

  wearth_n_ = R_en.transpose()*w_e;
  east_est_n = R_en.transpose()*e_e;

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

  east_est_n = kf_*east_est_n + (1.0-kf_)*(R_ni*(ang.cross(R_ni.transpose()*a_n) + (R_ni.transpose()*a_n-prev_acc_)/dt));
  
  // Define local level (g_error_) and heading (h_error_) error terms
  g_error_ = R_ni.transpose()*(kg_*(R_ni*acc).cross(a_n));
  h_error_ = (east_est_n.cross(e_n_));


  h_error_ = P_*h_error_;
  
  // save current gravity vector for doing calculating heading error
  prev_acc_ = R_ni.transpose()*a_n;

  // update R_ni
  R_ni = R_ni*((skew(g_error_ + R_ni.transpose()*kw_*h_error_ + ang - R_ni.transpose()*wearth_n_)*dt).exp());



}
