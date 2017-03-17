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

/**
 * @brief Constructor.
 * @param k Estimation gains/cutoff frequency (k(0): kg, k(1): kw, k(2): cuttoff_freq.
 * @param R_align Initial NED 2 Instrument Alignment estimation.
 * @param lat Latitude.
 * @param hz Sampling hz.
 */
AttEst::AttEst(Eigen::VectorXd k,Eigen::Matrix3d R_align, float lat, float hz)
{
  kg_ = k(0);
  kw_ = k(1);

  lat_ = lat;

  B_ = 1.0/hz/(1.0/hz + 1.0/(2.0*M_PI*k(2)));
  //B_ << 1.0/hz/(1.0/hz + 1.0/(2.0*M_PI*k(2))),0,0,0,1.0/hz/(1.0/hz + 1.0/(2.0*M_PI*k(2))),0,0,0,0;

  //A_ = 1.0 - B_;
  A_ = 1.0 - 1.0/hz/(1.0/hz + 1.0/(2.0*M_PI*k(2)));
  
  double earthrate = 15.0*M_PI/180.0/3600.0;
  Eigen::Matrix3d R_en = get_R_en(lat_);
  Rb_ = R_align;

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_e(0,0,earthrate);
  Eigen::Vector3d a_e = g_e + skew(w_e)*skew(w_e)*g_e*6371.0*1000.0/9.81;
  Eigen::Vector3d e_e = w_e.cross(a_e);

  a_n_ = R_en.transpose()*a_e;
  e_n_ = R_en.transpose()*e_e.normalized();
  P_   = a_n_.normalized()*a_n_.normalized().transpose(); 

  Rd_ = Eigen::Matrix3d::Identity(3,3);

  R_ni = Rb_*Rd_;

  east_est_n_ << 0.0,earthrate*cos(lat_),0.0;
  //east_est_n_ = R_align*east_est_n_;

  prev_acc_ << 0,0,0;

  wearth_n = R_en.transpose()*w_e;

}

/**
 * Destructor.
 */
AttEst::~AttEst(void)
{
}

/**
 * @brief Cycle estimation once.
 *
 * @param ang Angular velocity measurement.
 * @param acc Linear acceleration measurement.
 * @param t Time of sample.
 * @param dt Time between samples.
 */
void AttEst::step(Eigen::Vector3d ang,Eigen::Vector3d acc, float t, float dt)
{

  if (dt == 0 )
  {
    return;
  }
  
  east_est_n_ = A_*east_est_n_ + B_*R_ni*(ang.cross(acc) + (acc-prev_acc_)/dt);

  prev_acc_ = acc;
  
  g_error_ = R_ni.transpose()*(kg_*(R_ni*acc).cross(a_n_));
  h_error_ = R_ni.transpose()*(kw_*P_*east_est_n_.normalized().cross(e_n_));

  R_ni = R_ni*((skew(g_error_ + h_error_ + ang - R_ni.transpose()*wearth_n)*dt).exp());
 


}
