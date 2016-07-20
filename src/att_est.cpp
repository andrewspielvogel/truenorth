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
  ka_ = k(0);
  ke_ = k(1);
  kg_ = k(2);
  kw_ = k(3);

  lat_ = lat;

  dacc_ <<0,0,0;

  Eigen::Matrix3d R_en = get_R_en(lat_);
  Rb_ = R_en*R_align;

  Rd_ = Eigen::Matrix3d::Identity(3,3);

  acc_est_ = -R_align.block<3,1>(0,2);
  east_est_z_ = R_align.block<3,1>(0,1);
  ang_est_ <<0,0,0;


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


  ka_ = exp(-dt*ka_*2*M_PI);
  ke_ = exp(-dt*ke_*2*M_PI);
  // filter acc
  acc_est_ = acc_est_ + ka_*dt*( acc - acc_est_ );
  ang_est_ = ang_est_ + ke_*dt*( ang - ang_est_ );

  // calculate g error
  Eigen::Vector3d acc_true_s = R_sn*up_n;
  Eigen::Vector3d acc_est_s  = Rb_*Rd_*acc_est_prev;

  g_error_ = kg_*Rb_.transpose()*skew(acc_est_s)*acc_true_s;

  dacc_ = (acc_est_ - acc_est_prev)/dt;

  // estimate east
  Eigen::Vector3d east_est = skew(ang_est_)*acc_est_ + dacc_;
  east_est_z_ = Rd_*east_est;
  east_est_z_.normalize();


  // calculate heading error
  Eigen::Vector3d east_error_s = skew(Rb_*east_est_z_)*R_sn*east_n;
  east_error_ = kw_*Rb_.transpose()*east_error_s;

  // update laws
  Eigen::Matrix3d dR = mat_exp(skew(g_error_ + east_error_)*dt);

  // update R
  Rb_ = Rb_*dR;

  // integrate Rd
  Rd_ = Rd_*mat_exp(skew((ang)*dt));

  // update R_ni
  R_ni = R_sn.transpose()*Rb_*Rd_;


}
