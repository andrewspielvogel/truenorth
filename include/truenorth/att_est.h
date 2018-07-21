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
#include <truenorth/fog_bias.h>


/**
 * @brief Class for attitude adaptive identificaiton on SO(3).
 */
class AttEst
{
public:

  /**
   * @brief Constructor.
   *
   * @param k Estimation gains and rolling mean window size (k(0): kg, k(1): kn, k(2): ka, k(3): KE, k(4): kwb, k(5): kab).
   * @param R0 Initial NED 2 Instrument Alignment estimation.
   * @param lat Latitude.
   */
  AttEst(config_params params);

  
  virtual ~AttEst(void); /**< Destructor. */
    
  /**
   * @brief Cycle estimation once.
   *
   * @param ang Angular velocity measurement.
   * @param acc Linear acceleration measurement.
   * @param dt Time between last two measurements.
   */
  void step(Eigen::Vector3d ang,Eigen::Vector3d acc, float dt);

  config_params params;
  SO3Att att; /**< Attitude estimation object. */
  FOGBias bias; /**< Bias estimation object. */
  

};

#endif
