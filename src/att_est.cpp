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

  B_ = 1.0;//1.0/hz/(1.0/hz + 1.0/(2.0*M_PI*k(2)));

  A_ = 1.0 - B_;

  Eigen::Matrix3d R_en = get_R_en(lat_);
  Rb_ = R_align;

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_e(0,0,15.0*M_PI/180.0/3600.0);
  Eigen::Vector3d a_e = g_e + skew(w_e)*skew(w_e)*g_e*6371*1000/9.81;
  Eigen::Vector3d e_e = w_e.cross(a_e);

  a_n_ = R_en.transpose()*a_e;
  e_n_ = R_en.transpose()*e_e;
  P_   = a_e.normalized()*a_e.normalized().transpose(); 

  Rd_ = Eigen::Matrix3d::Identity(3,3);

  east_est_n_ << 0,(15*M_PI/180/3600)*cos(lat_),0;

  prev_acc_ << 0,0,0;

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

  // integrate Rd
  Eigen::Matrix3d w_hat = skew(ang)*dt;
  Rd_ = Rd_*w_hat.exp();
 
 
  // calculate east error
  Eigen::Vector3d dacc = (acc-prev_acc_)/dt;

  east_est_n_ = A_*east_est_n_ + B_*Rb_*Rd_*(ang.cross(acc) + dacc);

  Eigen::Vector3d a_tilde = kg_*Rb_.transpose()*(Rb_*Rd_*acc).cross(a_n_);
  Eigen::Vector3d e_tilde = kw_*Rb_.transpose()*P_*east_est_n_.cross(e_n_);


  
  prev_acc_ = acc;

  // update R
  Rb_ = Rb_*((skew(a_tilde + e_tilde)*dt).exp());



  // update R_ni
  R_ni = Rb_*Rd_;


}
