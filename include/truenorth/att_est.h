/**
 * @file
 * @date July 2016.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).

 * @brief Class for adaptive identification on SO(3).
 */


#ifndef ATT_EST_H
#define ATT_EST_H

#include <Eigen/Core>


/**
 * @brief Class for adaptive identificaiton on SO(3).
 */
class AttEst
{
public:

  AttEst(Eigen::VectorXd k,Eigen::Matrix3d R0); /**< Constructor. */
  virtual ~AttEst(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * Assume input output relation is \f$y = Ru\f$ where \f$R\in SO(3)\f$ is constant
   * @param u Input measurement.
   * @param y Output measurement.
   * @param dt Time between samples.
   */
  void step(Eigen::Vector3d w,Eigen::Vector3d a, float t, float dt);
  Eigen::Matrix3d R_ni;
 
 private:

  Eigen::Matrix3d Rb_; /**< Estimatation of static rotation. */

  Eigen::Matrix3d Rd_;
  float lat_;
  Eigen::Vector3d acc_est_;
  Eigen::Vector3d east_est_z_;
  
  float kg_;    
  float ka_;    
  float kw_;
  float ke_;
    

};

#endif
