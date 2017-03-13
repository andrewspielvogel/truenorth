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

  A_ = 1.0 - B_;

  Eigen::Matrix3d R_en = get_R_en(lat_);
  Rb_ = R_en*R_align;

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

  Eigen::Matrix3d R_sn = get_R_sn(lat_, t);
  Eigen::Matrix3d R_en = get_R_en(lat_);
  Eigen::Vector3d up_n(0,0,-1);
  Eigen::Vector3d east_n(0,1,0);

  // integrate Rd
  Eigen::Matrix3d w_hat = skew(ang)*dt;
  Rd_ = Rd_*w_hat.exp();
 

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_e(0,0,15.0*M_PI/180.0/3600.0);
  Eigen::Vector3d a_e = g_e + skew(w_e)*skew(w_e)*g_e*6371*1000/9.81;
  //a_e.normalize();

  Eigen::Vector3d a_n = R_en.transpose()*a_e.normalized();

  Eigen::Vector3d acc_true_s = get_R_se(t)*a_e;
  Eigen::Vector3d acc_est_s  = Rb_*Rd_*acc;


  // calculate local level error
  Eigen::Vector3d g_error = kg_*Rb_.transpose()*skew(acc_est_s)*acc_true_s;


  // calculate east error
  Eigen::Vector3d dacc = (acc-prev_acc_)/dt;

  Eigen::Vector3d east_est_n = R_sn.transpose()*Rb_*Rd_*(skew(ang)*acc + dacc);
  east_est_n_ = A_*east_est_n_ + B_*east_est_n;

  Eigen::Vector3d east_error_n = skew(east_est_n_.normalized())*east_n;
  
  Eigen::Vector3d east_error = kw_*Rb_.transpose()*R_sn*east_error_n.dot(a_n)*a_n;
  //error_ = east_error;

  Eigen::Matrix3d error_hat = skew(g_error + east_error)*dt;
  Eigen::Matrix3d dR = error_hat.exp();

  prev_acc_ = acc;

  // update R
  Rb_ = Rb_*dR;



  // update R_ni
  R_ni = R_sn.transpose()*Rb_*Rd_;


}
