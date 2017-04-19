/**
 * @file
 * @date Jan. 2017.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for adaptive bias identification.
 */


#ifndef BIAS_EST_H
#define BIAS_EST_H

#include <Eigen/Core>


/**
 * @brief Class for adaptive bias identificaiton.
 */
class BiasEst
{
public:

  BiasEst(Eigen::VectorXd k, float lat); /**< Constructor. */
  virtual ~BiasEst(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param w Angular velocity measurement.
   * @param a Linear acceleration measurement.
   * @param Rni Instrument attitude.
   * @param dt Time between last two measurements.
   */
  void step(Eigen::Matrix3d Rni, Eigen::Vector3d w,Eigen::Vector3d a, float dt);

  Eigen::Vector3d a_hat; /**< Linear acceleration estimation. */
  Eigen::Vector3d w_b; /**< Angular velocity bias estimation. */
  Eigen::Vector3d a_b; /**< Linear acceleration bias estimation. */
  Eigen::Vector3d z; /**< z bias estimation. */

 private:

  Eigen::Vector3d e_n_; /**< East vector in NED frame. */
  Eigen::Vector3d a_n_; /**< Gravity vector in NED frame. */
  Eigen::Vector3d prev_acc_; /**< Previous acc reading. */
  float kg_; /**< Gravity vector estimation gain. */        
  float kw_; /**< Angular velocity bias estimation gain. */
  float ka_; /**< Linear Acceleration bias estimation gain. */
  float kz_; /**< z bias estimation gain. */
    

};

#endif
