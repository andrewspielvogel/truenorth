/**
 * @file
 * @date July 2016.
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com).
 * @brief Class for attitude adaptive identification on SO(3). 
 */


#ifndef ATT_EST_H
#define ATT_EST_H

#include <Eigen/Core>
#include <truenorth/so3_att.h>


/**
 * @brief Class for attitude adaptive identificaiton on SO(3).
 */
class AttEst
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
  AttEst(Eigen::VectorXd k,Eigen::Matrix3d R0, float lat);

  
  virtual ~AttEst(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param ang Angular velocity measurement.
   * @param acc Linear acceleration measurement.
   * @param mag Magnetometer measurement.
   * @param dt Time between last two measurements.
   * @param t Time.
   */
  void step(Eigen::Vector3d ang,Eigen::Vector3d acc, Eigen::Vector3d mag, float dt);

  
  Eigen::Matrix3d R_ni; /**< Estimation of NED to instrument rotation. */
  Eigen::Vector3d a_n; /**< Linear Acceleration in the NED frame.*/
  
  Eigen::Vector3d acc_hat;
  Eigen::Vector3d w_E_north;
  Eigen::Vector3d w_b;
  Eigen::Vector3d a_b;
  Eigen::Vector3d da_b;


 private:

  SO3Att att_;

  float lat_; /**< Latitude. */


  float kg_; /**< Gravity vector estimation gain. */
  float kw_; /**< East vector estimation gain. */
  float ka_; /**< . */
  float kE_;
  float kb_;
  float kab_;

  int start_;
  
  float gamma_;



};

#endif
