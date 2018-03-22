/**
 * @file
 * @date March 2018.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for attitude adaptive identification on SO(3). 
 */


#ifndef SO3_ATT_H
#define SO3_ATT_H

#include <Eigen/Core>


/**
 * @brief Class for attitude adaptive identificaiton on SO(3).
 */
class SO3Att
{
public:

  /**
   * @brief Constructor.
   *
   * @param k Estimation gains and rolling mean window size (k(0): kg, k(1): kn).
   * @param R0 Initial NED 2 Instrument Alignment estimation.
   * @param lat Latitude.
   */
  SO3Att(Eigen::VectorXd k,Eigen::Matrix3d R0, float lat);

  
  virtual ~SO3Att(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param ang Angular velocity measurement.
   * @param g Gravity vector measurement.
   * @param north North vector measurement.
   * @param dt Time between last two measurements.
   */
  void step(Eigen::Vector3d ang,Eigen::Vector3d g, Eigen::Vector3d north, float d);
  
  Eigen::Matrix3d R_ni; /**< Estimation of NED to instrument rotation. */
 


 private:
  
  Eigen::Vector3d g_error_; /**< Local level error term. */
  Eigen::Vector3d h_error_; /**< Heading error term. */

  float kg_; /**< Gravity vector estimation gain. */
  float kn_; /**< North vector estimation gain. */

  Eigen::Matrix3d P_; /**< Projection matrix: \f${}^N_i\hat{R}^T(t){}^N\bar{a}_g{}^N\bar{a}^T_g{}^N_i\hat{R}(t)\f$. */
  Eigen::Vector3d w_E_n_; /**< \f${}^Nw_E\f$ vector. */
  Eigen::Vector3d a_n_; /**< \f${}^Na_g\f$ Vector. */

};

#endif
