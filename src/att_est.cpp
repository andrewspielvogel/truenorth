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
 * @param gain Estimation gain.
 * @param R0 Initial rotation estimate.
 */
AttEst::AttEst(Eigen::VectorXd k,Eigen::Matrix3d R_align, float lat)
{
  kg_ = k(0);
  kw_ = k(1);
  east_cut_ = k(2);

  lat_ = lat;

  A_ = 0.999993716834432;
  B_ = 8.885737960976864/1000000;

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
 * Assume input output relation is \f$y = Ru\f$ where \f$R\in SO(3)\f$ is constant
 * @param u Input measurement.
 * @param y Output measurement.
 * @param dt Time between samples.
 */
void AttEst::step(Eigen::Vector3d ang,Eigen::Vector3d acc, float t, float dt)
{

  Eigen::Matrix3d R_sn = get_R_sn(lat_, t);
  Eigen::Matrix3d R_en = get_R_en(lat_);
  Eigen::Vector3d up_n(0,0,-1);
  Eigen::Vector3d east_n(0,1,0);

  // integrate Rd
  Rd_ = Rd_*mat_exp(skew((ang)*dt));
 

  Eigen::Vector3d g_e(cos(lat_),0,sin(lat_));
  Eigen::Vector3d w_e(0,0,15*M_PI/180/3600);
  Eigen::Vector3d a_e = g_e + skew(w_e)*skew(w_e)*g_e;
  a_e.normalize();
  Eigen::Vector3d a_n = R_en.transpose()*a_e;

  Eigen::Vector3d acc_true_s = get_R_se(t)*a_e;
  Eigen::Vector3d acc_est_s  = Rb_*Rd_*acc;


  // calculate local level error
  Eigen::Vector3d g_error = kg_*Rb_.transpose()*skew(acc_est_s)*acc_true_s;

  Eigen::Vector3d dacc = (acc-prev_acc_)/dt;

  Eigen::Vector3d east_est_n = R_sn.transpose()*Rb_*Rd_*(skew(ang)*acc + dacc);
  east_est_n_ = A_*east_est_n_ + B_*east_est_n;

  Eigen::Vector3d east_error_n = skew(east_est_n_.normalized())*east_n;
  
  Eigen::Vector3d east_error = kw_*Rb_.transpose()*R_sn*east_error_n.dot(a_n)*a_n;

  Eigen::Matrix3d dR = mat_exp(skew(g_error + east_error)*dt);

  prev_acc_ = acc;

  // update R
  Rb_ = Rb_*dR;



  // update R_ni
  R_ni = R_sn.transpose()*Rb_*Rd_;


}
