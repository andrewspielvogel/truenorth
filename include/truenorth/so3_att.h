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
   * @param k Estimation gains and rolling mean window size (k(0): kg, k(1): kw, k(2): kf).
   * @param R0 Initial NED 2 Instrument Alignment estimation.
   * @param lat Latitude.
   * @param hz Sampling hz.
   */
  SO3Att(Eigen::VectorXd k,Eigen::Matrix3d R0, float lat);

  
  virtual ~SO3Att(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param ang Angular velocity measurement.
   * @param acc Linear acceleration measurement.
   * @param mag Magnetometer measurement.
   * @param dt Time between last two measurements.
   * @param t Time.
   */
  void step(Eigen::Vector3d ang,Eigen::Vector3d g, Eigen::Vector3d north, float d);
  
  Eigen::Matrix3d R_ni; /**< Estimation of NED to instrument rotation. */
 


 private:
  
  Eigen::Vector3d g_error_; /**< Local level error term. */
  Eigen::Vector3d h_error_; /**< Heading error term. */

  float lat_; /**< Latitude. */


  float kg_; /**< Gravity vector estimation gain. */
  float kn_; /**< North vector estimation gain. */

  Eigen::Matrix3d P_; /**< Projection matrix onto a_n_ vector. */
  Eigen::Vector3d w_E_n_;
  Eigen::Vector3d a_n_;

};

#endif
