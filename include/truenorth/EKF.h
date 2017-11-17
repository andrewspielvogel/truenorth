/**
 * @file
 * @date Nov. 2017.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for EKF attitude estimation.
 */


#ifndef EKF_H
#define EKF_H

#include <Eigen/Core>


/**
 * @brief Class for adaptive bias identificaiton.
 */
class AttEKF
{
public:

  AttEKF(Eigen::VectorXd x0, Eigen::VectorXd P0, Eigen::VectorXd R0, Eigen::VectorXd Q0, float lat); /**< Constructor. */

  Eigen::VectorXd x; /**< State. */
  Eigen::MatrixXd P; /**< State Covariance. */
  Eigen::MatrixXd R; /**< Measurement Noise. */
  Eigen::MatrixXd Q; /**< Process Noise. */
  Eigen::MatrixXd K; /**< Kalman Gain. */
  double gamma;


  void predict(float dt);

  Eigen::VectorXd calc_f(float dt);
  Eigen::VectorXd calc_h(void);
  double a_mag;
  double w_En_mag;

  void correct(Eigen::Vector3d ang, Eigen::Vector3d acc);

 private:

  void calc_K(void);
  Eigen::MatrixXd dynamics_jacobian(float dt);
  Eigen::MatrixXd measurement_jacobian(void);

};

#endif
