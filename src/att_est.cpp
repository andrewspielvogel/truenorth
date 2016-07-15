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
AttEst::AttEst(Eigen::VectorXd k,Eigen::Matrix3d R_align)
{
  ka_ = k(0);
  ke_ = k(1);
  kg_ = k(2);
  kw_ = k(3);

  lat_ =  39.32*M_PI/180;

  Eigen::Matrix3d R_en = get_R_en(lat_);

  Rb_ = R_en*R_align;

  Rd_ = Eigen::Matrix3d::Identity(3,3);

  acc_est_ = -R_align.block<3,1>(0,2);
  east_est_z_ = R_align.block<3,1>(0,1);

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

  Eigen::Vector3d up_n(0,0,-1);
  Eigen::Vector3d east_n(0,1,0);
  Eigen::Vector3d acc_est_prev = acc_est_;

  // filter acc
  Eigen::Vector3d da = acc - acc_est_;
  acc_est_ = acc_est_ + ka_*da*dt;

  // differentiant acc_est
  Eigen::Vector3d dacc = (acc_est_ - acc_est_prev)/dt;

  // calculate g error
  Eigen::Vector3d acc_true_s = R_sn*up_n;
  Eigen::Vector3d acc_est_s  = Rb_*Rd_*acc_est_prev;

  Eigen::Vector3d g_error = kg_*Rb_.transpose()*skew(acc_est_s)*acc_true_s;

  // estimate east
  Eigen::Vector3d east_true_s = R_sn*east_n;
  Eigen::Vector3d east_est = skew(ang)*acc_est_prev + dacc;
  Eigen::Vector3d east_est_z = Rd_*east_est;

  // filter east estimation signal
  east_est_z_ = east_est_z_ +ke_*dt*(east_est_z-east_est_z_);

  // calculate heading error
  Eigen::Vector3d east_est_s = Rb_*east_est_z_;

  Eigen::Vector3d east_error_s = skew(east_est_s)*east_true_s;
  Eigen::Vector3d east_error_s_along_g = east_error_s.dot(acc_true_s)*acc_true_s;

  Eigen::Vector3d east_error = kw_*Rb_.transpose()*east_error_s_along_g;

  // update laws
  Eigen::Matrix3d dR_g = mat_exp(skew(g_error)*dt);
  Eigen::Matrix3d dR_e = mat_exp(skew(east_error)*dt);


  // update R
  Rb_ = Rb_*dR_g*dR_e;

  // integrate Rd
  Eigen::Matrix3d dRd = skew((ang)*dt);
  Rd_ = Rd_*mat_exp(dRd);

  R_ni = R_sn.transpose()*Rb_*Rd_;


}
