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
   * @param k Estimation gains and rolling mean window size (k(0): kg, k(1): kw, k(2): kf).
   * @param lat Latitude.
   */
  FOGBias(Eigen::VectorXd k, Eigen::Matrix3d R0, float lat);

  
  virtual ~FOGBias(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param ang Angular velocity measurement.
   * @param acc Linear acceleration measurement.
   * @param dt Time between last two measurements.
   */
  void step(Eigen::Vector3d ang,Eigen::Vector3d acc, float dt);
  
  Eigen::Vector3d acc_hat;
  Eigen::Vector3d w_E_north;
  Eigen::Vector3d w_b;
  Eigen::Vector3d a_b;


 private:

  float lat_; /**< Latitude. */

  float ka_; /**< . */
  float kE_;
  float kb_;
  float kab_;

  int start_;
  
  float gamma_;



};

#endif
