/**
 * @file
 * @date May 2016
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of so3_adap_id.h.
 *
 * class for adaptive id on so3
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <kvh_1775/helper_funcs.h>
#include <kvh_1775/so3_adap_id.h>


/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

/**
 * Constructor.
 */
SO3AdapId::SO3AdapId(float gain)
{

  k = gain;
  R << 1,0,0,0,1,0,0,0,1;

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
void SO3AdapId::step(Eigen::Vector3d u,Eigen::Vector3d y, float dt)
{

  // Calculate error correction
  Eigen::Vector3d y_est = R*u;
  Eigen::Vector3d err = k*R.transpose()*y_est.cross(y);
  Eigen::Matrix3d dt_err = dt*skew(err);

  // feedback error correction
  R = R*mat_exp(dt_err);


}
