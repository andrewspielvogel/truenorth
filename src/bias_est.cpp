/**
 * @file
 * @date Jan. 2017
 * @author Andrew Spielvogel (andrewspielvogel@gmail.com)
 * @brief Implementation of bias_est.h.
 *
 */

#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <truenorth/helper_funcs.h>
#include <truenorth/bias_est.h>


/*
 *
 * SEE HEADER FILE FOR DOCUMENTATION
 *
 */

/**
 * @brief Constructor.
 * @param k Estimation gains.
 */

BiasEst::BiasEst(Eigen::VectorXd k, float lat)
{
  kg_ = k(0);
  kw_ = k(1);
  ka_ = k(2);
  kz_ = k(3);

  float earth_rate = 15.04*M_PI/180.0/3600.0;
  float earth_radius = 6371.0*1000.0;
  float g_0 = 9.81;
  Eigen::Vector3d w_e(0,0,earth_rate);
  Eigen::Vector3d a_e(cos(lat)-earth_rate*earth_rate*cos(lat)*earth_radius/g_0,0,sin(lat));
  Eigen::Vector3d e_e = skew(w_e)*a_e;
  
  e_n_ = get_R_en(lat).transpose()*e_e;

  a_hat << 0,0,1;

  w_b << 0,0,0;
  a_b << 0,0,0;
  z << 0,0,0;



}

/**
 * Destructor.
 */
BiasEst::~BiasEst(void)
{
}

/**
 * @brief Cycle estimation once.
 *
 * @param ang Angular velocity measurement.
 * @param acc Linear acceleration measurement.
 * @param t Time of sample.
 * @param dt Time between samples.
 */
void BiasEst::step(Eigen::Matrix3d Rni, Eigen::Vector3d ang,Eigen::Vector3d acc, float dt)
{

  double a = 0.6;
  double b = 1.0 - a;
  
  Eigen::Vector3d da = a_hat - acc;
  Eigen::Vector3d a_dot = Rni.transpose()*e_n_ - skew(ang-w_b)*acc + skew(ang)*a_b - z - kg_*da;
  Eigen::Vector3d w_b_dot = -kw_*skew(acc)*da;
  Eigen::Vector3d a_b_dot = ka_*skew(ang)*da;
  Eigen::Vector3d z_dot   = kz_*da;

  a_hat = a_hat + a_dot*dt;
  w_b   = w_b*a + b*(w_b   + w_b_dot*dt); 
  //w_b   = w_b   + w_b_dot*dt;
  a_b   = a_b   + a_b_dot*dt;
  z     = z     + z_dot*dt;

}
