/**
 * @file
 * @date Feb. 2018.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for MEMS IMU bias estimation. 
 */


#ifndef MEMS_BIAS_H
#define MEMS_BIAS_H

#include <Eigen/Core>


/**
 * @brief Class for attitude adaptive identificaiton on SO(3).
 */
class MEMSBias
{
public:

  /**
   * @brief Constructor.
   *
   * @param k Estimation gains and rolling mean window size (k(0): kg, k(1): kw, k(2): kf).
   * @param hz Sampling hz.
   */
  MEMSBias(Eigen::VectorXd k, int hz);

  
  virtual ~MEMSBias(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param ang Angular velocity measurement.
   * @param acc Linear acceleration measurement.
   * @param mag Magnetometer measurement.
   * @param dt Time between last two measurements.
   * @param t Time.
   */
  void step(Eigen::Vector3d ang,Eigen::Vector3d acc, Eigen::Vector3d mag, float dt, float t);

  
  Eigen::Vector3d acc_hat;
  Eigen::Vector3d mag_hat;
  Eigen::Vector3d w_b;
  Eigen::Vector3d a_b;
  Eigen::Vector3d m_b;
  Eigen::Matrix3d Rni;


 private:

  float lat_; /**< Latitude. */

  int hz_; /**< Sampling hz. */
  int start_;

  float kg_;
  float kn_;
  float ka_;
  float km_; 
  float kab_;
  float kmb_;
  float kwb_;

  Eigen::Vector3d a_n_;
  Eigen::Vector3d m_n_;


};

#endif
