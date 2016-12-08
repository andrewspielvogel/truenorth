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

  AttEst(Eigen::VectorXd k,Eigen::Matrix3d R0, float lat, float hz); /**< Constructor. */
  virtual ~AttEst(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param w Angular velocity measurement.
   * @param a Linear acceleration measurement.
   * @param t Time of measurement.
   * @param dt Time between last two measurements.
   */
  void step(Eigen::Vector3d w,Eigen::Vector3d a, float t, float dt);
  Eigen::Matrix3d R_ni; /**< Estimation of NED to instrument rotation. */
  Eigen::Matrix3d Rb_; /**< Estimatation of Rbar rotation. */
  Eigen::Vector3d east_est_n_; /**< Estimation of east in NED frame. */

 private:

  Eigen::Vector3d prev_acc_; /**< Previous acceleration. */
  Eigen::Matrix3d Rd_; /**< Delta Rotation: zero to instrument rotation. */
  float lat_; /**< Latitude. */

  double A_; /**< Butter filter coefficient. */
  double B_; /**< Butter filter coefficient. */
  
  float kg_; /**< Gravity vector estimation gain. */        
  float kw_; /**< East vector estimation gain. */
    

};

#endif
