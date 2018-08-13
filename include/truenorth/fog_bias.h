/**
 * @file
 * @date March 2018.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for adaptive bias identification for FOG IMUs. 
 */


#ifndef FOG_BIAS_H
#define FOG_BIAS_H

#include <Eigen/Core>


/**
 * @brief Class for adaptive bias identificaiton for FOG IMUs.
 */
class FOGBias
{
public:

  /**
   * @brief Constructor.
   *
   * @param k Estimation gains and rolling mean window size (k(0): ka, k(1): kE, k(2): kwb, k(3): kab).
   * @param R0 Initial guess of instrument's attitude.
   * @param lat Latitude.
   */
  FOGBias(config_params params);

  
  virtual ~FOGBias(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param ang Angular velocity measurement.
   * @param acc Linear acceleration measurement.
   * @param dt Time between last two measurements.
   */
  void step(Eigen::Vector3d ang,Eigen::Vector3d acc, float dt);
  
  Eigen::Vector3d acc_hat; /**< Estimated linear acceleration vector. */
  Eigen::Vector3d w_E_north; /**< Estimated North vector. */
  Eigen::Vector3d w_b; /**< Estimated angular rate bias. */
  Eigen::Vector3d a_b; /**< Estimated linear acceleration bias. */
  config_params params;


 private:

  float t_;

  int start_; /**< Start estimator after initializing estimated linear acceleration. */
  
  float gamma_; /**< Magnitude of \f$\frac{\|{}^Nw_{E_d}\|}{\|{}^Na_g\|}\f$. */

  Eigen::Vector3d w_E_n;

};

#endif
