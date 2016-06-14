/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of so3_adap_id.h.
 *
 * class for adaptive id on so3
 */

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <truenorth/helper_funcs.h>
#include <truenorth/so3_adap_id.h>


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
SO3AdapId::SO3AdapId(float k1, float k2,Eigen::Matrix3d R0)
{
  
  k1_ = k1;
  k2_ = k2;
  R << R0;

}

/**
 * Destructor.
 */
SO3AdapId::~SO3AdapId(void)
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
void SO3AdapId::step(Eigen::Vector3d u_a,Eigen::Vector3d y_a, Eigen::Vector3d u_e,Eigen::Vector3d y_e, float dt)
{

  // Calculate error correction
  Eigen::Vector3d y_est_a = R*u_a;
  Eigen::Vector3d err_a = k1_*R.transpose()*y_est_a.cross(y_a);
  Eigen::Matrix3d dt_err_a = dt*skew(err_a);

  Eigen::Vector3d y_est_e = R*u_e;
  Eigen::Vector3d err_e = k2_*R.transpose()*y_est_e.cross(y_e);
  Eigen::Matrix3d dt_err_e = dt*skew(err_e);

  // feedback error correction
  R = R*mat_exp(dt_err_a + dt_err_e);


}
