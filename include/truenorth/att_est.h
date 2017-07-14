/**
 * @file
 * @date July 2016.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for attitude adaptive identification on SO(3).
 */


#ifndef ATT_EST_H
#define ATT_EST_H

#include <Eigen/Core>


/**
 * @brief Class for attitude adaptive identificaiton on SO(3).
 */
class AttEst
{
public:
  Eigen::Vector3d g_error_; /**< Local level error term. */
  Eigen::Vector3d h_error_; /**< Heading error term. */
  Eigen::Vector3d east_est_n_; /**< Estimation of east in NED frame. */

  Eigen::Vector3d prev_afilt_;
  Eigen::Vector3d prev_wfilt_;

  
  /**
   * @brief Constructor.
   * @param k Estimation gains/cutoff frequency (k(0): kg, k(1): kw, k(2): cuttoff_freq.
   * @param R0 Initial NED 2 Instrument Alignment estimation.
   * @param lat Latitude.
   * @param hz Sampling hz.
   */
  AttEst(Eigen::VectorXd k,Eigen::Matrix3d R0, float lat, float hz);

  
  virtual ~AttEst(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param w Angular velocity measurement.
   * @param a Linear acceleration measurement.
   * @param dt Time between last two measurements.
   */
  void step(Eigen::Vector3d w,Eigen::Vector3d a, float dt);

  
  Eigen::Matrix3d R_ni; /**< Estimation of NED to instrument rotation. */
  Eigen::Vector3d a_n; /**< Linear Acceleration in the NED frame.*/


 private:

  Eigen::Vector3d prev_acc_; /**< Previous acceleration. */
  float lat_; /**< Latitude. */

  double A_; /**< Butter filter coefficient. */
  double B_; /**< Butter filter coefficient. */

  float kg_; /**< Gravity vector estimation gain. */        
  float kw_; /**< East vector estimation gain. */

  Eigen::Vector3d e_n_; /**< East Direction in the NED frame.*/
  Eigen::Vector3d wearth_n_; /**< Earth's angular velocity in the NED frame. */
  Eigen::Matrix3d P_; /**< Projection matrix onto a_n_ vector. */



};

#endif
